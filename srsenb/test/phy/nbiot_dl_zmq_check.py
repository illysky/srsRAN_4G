#!/usr/bin/env python3
"""
End to end check of the NB-IoT downlink of a running srsenb.

srsenb is started on the ZMQ virtual radio with expert.nbiot_config set. This script is the other end of the radio:
it answers srsenb's receive requests with silence, pulls the transmitted IQ, cuts it into subframes, runs its own OFDM
demodulator and compares the anchor PRB with an independent model written from the specification (the same model that
checks the composer, enb_dl_nbiot_check.py), including the LTE CRS values that must survive in the anchor PRB. What
this adds to the composer test: the wiring inside srsenb -- where in the pipeline the anchor is written, which TTI and
hyper frame the composer is told, the SIB1/SIB2 content the eNB really built from the carrier file -- and the LTE side:
the PRBs next to the anchor and the RBG the scheduler reserved.

The uplink is checked the same way: the radio injects NPRACH preambles, synthesised by the independent generator
nprach_ref.py at the eNB sample rate, at the subframes the specification puts NPRACH opportunities (period, start time
from the carrier file), with delay, carrier offset, noise, two users at once and empty opportunities, and the detections
srsenb prints must be exactly those: preamble, subframe, timing and frequency offset.

Nothing in it knows how srsenb or the composer is implemented. Requires pyzmq and numpy; exits with 77 (skipped) if
either is missing.

usage: nbiot_dl_zmq_check.py srsenb enb.conf|srsenb_source_dir nbiot.conf nbiot_sib_pack seconds model_dir...

If the second argument is a directory it is taken to be the srsenb source directory: a minimal ZMQ configuration is
generated from the *.conf.example files in it, with the LTE cell (PCI, EARFCN, PRBs) taken from nbiot.conf.
"""
import atexit
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import time

try:
    import numpy as np
    import zmq
except ImportError as e:
    print('SKIP: %s' % e)
    sys.exit(77)

SF_LEN = 7680          # 7.68 MS/s
FFT = 512
CP0, CP = 40, 36
TX_PORT, RX_PORT = 'tcp://localhost:2010', 'tcp://*:2011'


def sym_starts():
    """Start (after the cyclic prefix) of each of the 14 OFDM symbols of a subframe."""
    out, off = [], 0
    for _ in range(2):
        for l in range(7):
            off += CP0 if l == 0 else CP
            out.append(off)
            off += FFT
    assert off == SF_LEN
    return out


SYM = sym_starts()


def demod(sf_samples, nof_prb):
    """14 x (12 nof_prb) resource grid of one subframe (TS 36.211 6.12: no DC subcarrier, bins 1..N/2 above)."""
    n_sc = 12 * nof_prb
    grid = np.zeros((14, n_sc), dtype=complex)
    for l, s in enumerate(SYM):
        spec = np.fft.fft(sf_samples[s:s + FFT])
        grid[l, :n_sc // 2] = spec[FFT - n_sc // 2:]
        grid[l, n_sc // 2:] = spec[1:1 + n_sc // 2]
    return grid


# ------------------------------------------------------------------------------ independent LTE CRS (TS 36.211 6.10.1)
_CRS_CACHE = {}


def lte_crs_values(chk, pci, sf, nof_prb, prb):
    """Port 0 CRS of the PRB after the control region: {(l, k): value}, m' = m + N_RB^max,DL - N_RB^DL."""
    key = (pci, sf, nof_prb, prb)
    if key in _CRS_CACHE:
        return _CRS_CACHE[key]
    vals = {}
    _CRS_CACHE[key] = vals
    vshift = pci % 6
    for slot in (0, 1):
        ns = 2 * sf + slot
        for ls in (0, 4):
            c_init = 1024 * (7 * (ns + 1) + ls + 1) * (2 * pci + 1) + 2 * pci + 1
            c = chk.gold(c_init, 2 * 2 * 110)
            v = 0 if ls == 0 else 3
            for m_in_prb in (0, 1):
                m = 2 * prb + m_in_prb
                mp = m + 110 - nof_prb
                vals[(slot * 7 + ls, 6 * m_in_prb + (v + vshift) % 6)] = \
                    complex(1 - 2 * c[2 * mp], 1 - 2 * c[2 * mp + 1]) / math.sqrt(2)
    return vals


# ------------------------------------------------------------------------------ MIB-NB from TS 36.331 (34 bits)
def mib_nb_bits(sfn_base, hfn, sched, tag, ab, crs_seq):
    def bits(v, n):
        return [(v >> (n - 1 - i)) & 1 for i in range(n)]
    return (bits(sfn_base >> 6, 4) + bits(hfn & 3, 2) + bits(sched, 4) + bits(tag, 5) + [1 if ab else 0]
            + bits(0, 2)          # operationModeInfo: inband-SamePCI
            + bits(crs_seq, 5) + [0] * 11)


def crs_seq_info(nof_prb, prb):
    """TS 36.213 Table 16.8-1, odd N_RB: offset of the anchor from the carrier centre, in the order of the table."""
    assert nof_prb % 2 == 1
    offs = [-35, -30, -25, -20, -15, -10, -5, 5, 10, 15, 20, 25, 30, 35]
    return offs.index(prb - nof_prb // 2)


def conf_int(text, key):
    m = re.search(r'\b%s\s*=\s*(0x[0-9a-fA-F]+|-?\d+)' % key, text)
    return int(m.group(1), 0)


class Radio:
    """The far end of srsenb's ZMQ radio: the uplink is silence or noise plus injected bursts, records the downlink.

    bursts: list of (first sample index in the uplink stream, complex samples). Uplink stream sample 0 is the first
    sample of TTI 10236, four subframes before TTI 0, which is where srsenb starts receiving."""

    def __init__(self, bursts=(), noise_var=0.0, seed=1):
        self.bursts = sorted(bursts, key=lambda b: b[0])
        self.noise_var = noise_var
        self.rng = np.random.default_rng(seed)
        self.ul_chunks = 0
        self.ctx = zmq.Context()
        self.stop = threading.Event()
        self.chunks = []
        self.rx_thread = threading.Thread(target=self._uplink, daemon=True)
        self.tx_thread = threading.Thread(target=self._downlink, daemon=True)

    def start(self):
        self.rx_thread.start()
        self.tx_thread.start()

    def _uplink(self):
        s = self.ctx.socket(zmq.REP)
        s.setsockopt(zmq.RCVTIMEO, 200)
        s.bind(RX_PORT)
        silence = bytes(SF_LEN * 8)
        k = 0
        while not self.stop.is_set():
            try:
                s.recv()
            except zmq.Again:
                continue
            lo, hi = k * SF_LEN, (k + 1) * SF_LEN
            if not self.bursts and self.noise_var == 0.0:
                s.send(silence)
            else:
                blk = np.zeros(SF_LEN, dtype=np.complex64)
                if self.noise_var > 0.0:
                    sig = math.sqrt(self.noise_var / 2.0)
                    blk += (self.rng.normal(0, sig, SF_LEN) + 1j * self.rng.normal(0, sig, SF_LEN)).astype(np.complex64)
                for start, x in self.bursts:
                    if start >= hi:
                        break
                    if start + len(x) > lo:
                        a, b = max(lo, start), min(hi, start + len(x))
                        blk[a - lo:b - lo] += x[a - start:b - start]
                s.send(blk.tobytes())
            k += 1
            self.ul_chunks = k
        s.close(0)

    def _downlink(self):
        s = self.ctx.socket(zmq.REQ)
        s.setsockopt(zmq.RCVTIMEO, 2000)
        s.setsockopt(zmq.SNDTIMEO, 2000)
        s.setsockopt(zmq.REQ_RELAXED, 1)
        s.setsockopt(zmq.REQ_CORRELATE, 1)
        s.connect(TX_PORT)
        pending = False
        while not self.stop.is_set():
            try:
                if not pending:
                    s.send(b'\xff')
                    pending = True
                data = s.recv()
                pending = False
                self.chunks.append(data)
            except zmq.Again:
                continue
        s.close(0)

    def samples(self):
        return np.frombuffer(b''.join(self.chunks), dtype=np.complex64)


# NPRACH injection plan: one entry per opportunity, repeating. Each preamble is (n_init, delay in samples at 1.92 MS/s,
# carrier frequency offset in Hz, SNR per sample after decimation to 1.92 MS/s in dB). An empty entry is an opportunity
# where nobody transmits.
NPRACH_PLAN = [
    [(0, 0.0, 0.0, 12.0)],
    [(5, 37.0, 183.0, 10.0)],
    [],
    [(11, 3.25, -297.0, 12.0)],
    [(7, 100.0, 51.0, 3.0)],
    [(2, 10.0, 127.0, 12.0), (9, 60.5, -203.0, 12.0)],
    [(4, 1.0, 0.0, 6.0)],
    [],
]


def nprach_bursts(ref, nprach, nof_prb, anchor, n_opps, fs):
    """(bursts, expected) for n_opps opportunities. expected: (tti, n_init, toa, cfo, first stream sample, last, TTI
    since the start of hyper frame 0)"""
    fmt = 1 if nprach['cp_us'] > 100.0 else 0
    offset_hz = (2 * anchor + 1 - nof_prb) * 90000
    ratio = fs // 1920000
    bursts, expected = [], []
    for o in range(n_opps):
        t_ms = nprach['start'] + nprach['period'] * o     # since TTI 0 of hyper frame 0
        tti = t_ms % 10240
        # spec: start_time after the first subframe of a frame with n_f mod (period / 10) == 0
        assert ((t_ms - nprach['start']) // 10) % (nprach['period'] // 10) == 0
        base = (t_ms + 4) * SF_LEN                           # stream sample 0 is TTI 10236
        for (n_init, delay, cfo, snr_db) in NPRACH_PLAN[o % len(NPRACH_PLAN)]:
            x, _ = ref.preamble(fs, fmt, nprach['n_rep'], nprach['cell_id'], n_init, nprach['sc_offset'])
            d = int(round(delay * ratio))
            idx = base + d + np.arange(len(x), dtype=np.int64)
            mix = ((offset_hz * idx) % fs) / fs
            # the noise floor is fixed (variance = ratio per sample), which is unit variance after decimation, so the
            # amplitude sets the SNR of this burst
            amp = 10.0 ** (snr_db / 20.0)
            y = amp * x * np.exp(2j * np.pi * (mix + cfo * idx / fs))
            bursts.append((base + d, y.astype(np.complex64)))
            expected.append((tti, n_init, delay, cfo, base, base + d + len(x), t_ms))
    return bursts, expected


ENB_CONF = """[enb]
enb_id = 0x19C
mcc = 234
mnc = 01
mme_addr = 127.0.1.99
gtp_bind_addr = 127.0.1.2
s1c_bind_addr = 127.0.1.2
s1c_bind_port = 0
n_prb = %(nof_prb)d

[enb_files]
sib_config = sib.conf
rr_config  = rr.conf
rb_config = rb.conf

[rf]
tx_gain = 50
rx_gain = 40
device_name = zmq
device_args = fail_on_disconnect=false,tx_port=tcp://*:2010,rx_port=tcp://localhost:2011,id=enb,base_srate=7.68e6

[log]
all_level = %(level)s
filename = %(log)s
file_max_size = -1

[gui]
enable = false

[expert]
lte_sample_rates = true
"""


def make_config_dir(src_dir, nof_prb, earfcn, pci):
    """enb.conf and friends for a ZMQ eNB that never reaches a core network (the MME address is unroutable)."""
    d = tempfile.mkdtemp(prefix='srsenb_nbiot_cfg_')
    atexit.register(shutil.rmtree, d, True)
    with open(os.path.join(d, 'enb.conf'), 'w') as f:
        f.write(ENB_CONF % {'nof_prb': nof_prb, 'log': os.path.join(d, 'enb.log'),
                            'level': os.environ.get('NBIOT_TEST_LOG_LEVEL', 'warning')})
    for name in ('sib', 'rb'):
        shutil.copy(os.path.join(src_dir, name + '.conf.example'), os.path.join(d, name + '.conf'))
    rr = open(os.path.join(src_dir, 'rr.conf.example')).read()
    rr, n1 = re.subn(r'dl_earfcn\s*=\s*\d+;', 'dl_earfcn = %d;' % earfcn, rr, count=1)
    rr, n2 = re.subn(r'\bpci\s*=\s*\d+;', 'pci = %d;' % pci, rr, count=1)
    if n1 != 1 or n2 != 1:
        raise RuntimeError('rr.conf.example has an unexpected layout')
    with open(os.path.join(d, 'rr.conf'), 'w') as f:
        f.write(rr)
    return os.path.join(d, 'enb.conf')


def due_preambles(expected, chunks_served, first_detect_margin):
    """Preambles of opportunities srsenb had received completely, with a margin for the detector thread."""
    served = chunks_served - first_detect_margin
    return [e for e in expected if e[5] + 1200 * SF_LEN // 1000 < served * SF_LEN]


def check_nprach(console, expected, nprach, chunks_served, first_detect_margin):
    """Every injected preamble that srsenb had time to process must be reported once with the right subframe, preamble
    identifier, arrival time and frequency offset, and nothing else may be reported."""
    det = []
    for m in re.finditer(r'NB-IoT: NPRACH preamble (\d+) at TTI (\d+) \(frame (\d+)\), ToA ([-\d.]+) samples = ([-\d.]+) us '
                         r'\(timing advance (\d+)\), CFO ([+-]\d+) Hz, metric (\d+)', console):
        det.append({'n_init': int(m.group(1)), 'tti': int(m.group(2)), 'toa': float(m.group(4)), 'ta': int(m.group(6)),
                    'cfo': float(m.group(7)), 'metric': float(m.group(8))})
    ok = True
    if not re.search(r'NB-IoT: listening for NPRACH', console):
        print('FAIL: srsenb did not report starting the NPRACH receiver')
        ok = False

    due = due_preambles(expected, chunks_served, first_detect_margin)
    matched = set()
    worst_toa = worst_cfo = 0.0
    for (tti, n_init, delay, cfo, base, end, _) in due:
        hits = [i for i, d in enumerate(det) if d['tti'] == tti and d['n_init'] == n_init and i not in matched]
        if not hits:
            print('FAIL: preamble %d at TTI %d (delay %.2f, cfo %+.0f) was not detected' % (n_init, tti, delay, cfo))
            ok = False
            continue
        i = hits[0]
        matched.add(i)
        et, ec = det[i]['toa'] - delay, det[i]['cfo'] - cfo
        if det[i]['ta'] != max(0, int(math.floor(det[i]['toa'] + 0.5))):
            print('FAIL: timing advance %d for a ToA of %.2f samples' % (det[i]['ta'], det[i]['toa']))
            ok = False
        worst_toa, worst_cfo = max(worst_toa, abs(et)), max(worst_cfo, abs(ec))
        if abs(et) > 0.25 or abs(ec) > 15.0:
            print('FAIL: preamble %d at TTI %d: toa %.2f (want %.2f), cfo %+.0f (want %+.0f)' %
                  (n_init, tti, det[i]['toa'], delay, det[i]['cfo'], cfo))
            ok = False
    extra = [d for i, d in enumerate(det) if i not in matched]
    # a detection of an injected burst that was too late to count as "due" is not extra
    allowed = {(e[0], e[1]) for e in expected}
    extra = [d for d in extra if (d['tti'], d['n_init']) not in allowed]
    for d in extra[:8]:
        print('FAIL: unexpected detection %s' % d)
    if extra:
        ok = False
    distinct_users = len({(e[1]) for e in due})
    print('NPRACH: %d preambles injected in %d opportunities that srsenb had processed, %d detections (%d unexpected); '
          'worst ToA error %.2f samples, worst CFO error %.1f Hz, %d preamble identifiers' %
          (len(due), len({e[0] + 10240 * 0 for e in due}), len(det), len(extra), worst_toa, worst_cfo, distinct_users))
    if len(due) < 8:
        print('FAIL: too few NPRACH preambles were exercised (%d)' % len(due))
        ok = False
    return ok


# ------------------------------------------------------------------------------ Msg2: what a UE makes of the responses
def read_responses(console):
    """What srsenb says it sent: one dict per 'random access response' line of the console."""
    out = []
    for m in re.finditer(r'NB-IoT: random access response for preamble (\d+): RA-RNTI (\d+), TC-RNTI 0x([0-9a-f]+), TA (\d+), '
                         r'NPDCCH (\d+)\.\.(\d+), NPDSCH (\d+)\.\.(\d+) \(window (\d+)\.\.(\d+)\)', console):
        out.append({'preamble': int(m.group(1)), 'ra_rnti': int(m.group(2)), 'tc': int(m.group(3), 16),
                    'ta': int(m.group(4)), 'npdcch': (int(m.group(5)), int(m.group(6))),
                    'npdsch': (int(m.group(7)), int(m.group(8))), 'window': (int(m.group(9)), int(m.group(10)))})
    return out


def check_random_access(console, due, cfg, get_grid, uem, dlch, chk, ch, valid, get_model, msg3_expect):
    """Every preamble srsenb detected must be answered in the response window, and what a UE model decodes from the
    captured downlink must be exactly the response the eNB says it sent. The decoded transmissions are entered in the
    composer model (Model.dyn) so the subframe-by-subframe comparison of the anchor PRB accounts for them and finds
    anything the eNB sent besides."""
    ok = True
    rx = uem.Receiver(chk, cfg['pci'], ch.pos, valid, dlch.model_plan_npdsch)
    said = read_responses(console)
    unanswered = re.findall(r'NB-IoT: preamble (\d+) not answered: (.*)', console)
    for p, why in unanswered:
        print('FAIL: preamble %s not answered: %s' % (p, why))
        ok = False

    decoded = {}          # (ra_rnti, npdcch start) -> transmission
    per_preamble = []
    for (tti, n_init, delay, cfo, base, end, t_ms) in due:
        ws = uem.window_first_sf(t_ms, cfg['n_rep'], cfg['fmt'])
        we = ws + cfg['window_pp'] * uem.ss_period(cfg['r_max'], cfg['g_halves']) - 1
        rnti = uem.ra_rnti(t_ms)
        match = None
        for k0 in uem.ss_starts(cfg['r_max'], cfg['g_halves'], cfg['offset_eighths'], ws, we):
            key = (rnti, k0)
            if key not in decoded:
                dci, ts = rx.decode_npdcch(get_grid, k0, cfg['r_max'], rnti)
                if dci is None:
                    decoded[key] = None
                    continue
                f = uem.parse_dci_n1(dci)
                if f is None:
                    print('FAIL: DCI at %d is not a downlink assignment' % k0)
                    ok = False
                    decoded[key] = None
                    continue
                n_sf, n_rep = uem.N_SF[f['i_sf']], uem.N_REP[f['i_rep']]
                k0d = (uem.K0_LO if cfg['r_max'] < 128 else uem.K0_HI)[f['i_delay']]
                tbs = dlch.TBS[f['i_mcs']][f['i_sf']]
                tb, plan = rx.decode_npdsch(get_grid, ts[-1], k0d, n_sf, n_rep, tbs, rnti)
                decoded[key] = {'k0': k0, 'ts': ts, 'dci': dci, 'f': f, 'plan': plan, 'tb': tb, 'rnti': rnti,
                                'tbs': tbs, 'n_sf': n_sf, 'window': (ws, we)}
            d = decoded[key]
            if d is None or d['tb'] is None:
                continue
            res = uem.parse_rar_pdu(d['tb'])
            if res is None:
                continue
            for r in res[1]:
                if r['rapid'] == n_init and match is None:
                    match = (d, r)
        if match is None:
            print('FAIL: no response for preamble %d sent at TTI %d (window %d..%d, RA-RNTI %d)' %
                  (n_init, t_ms, ws, we, rnti))
            ok = False
            continue
        d, r = match
        per_preamble.append((n_init, t_ms, d, r))
        want_ta = max(0, int(math.floor(delay + 0.5)))
        if abs(r['ta'] - want_ta) > 1:
            print('FAIL: preamble %d: timing advance %d for a delay of %.2f samples' % (n_init, r['ta'], delay))
            ok = False
        if (r['sc15'], r['i_sc'], r['i_delay'], r['i_rep'], r['i_mcs']) != msg3_expect:
            print('FAIL: preamble %d: Msg3 grant %s, configured %s' %
                  (n_init, (r['sc15'], r['i_sc'], r['i_delay'], r['i_rep'], r['i_mcs']), msg3_expect))
            ok = False
        # DCI subframe repetition number of R = Rmax (TS 36.213 Table 16.6-3)
        want_dci_rep = {1: 0, 2: 1, 4: 2}.get(cfg['r_max'], 3)
        if d['f']['i_mcs'] != 4 or d['f']['i_sf'] != 0 or d['f']['dci_rep'] != want_dci_rep:
            print('FAIL: DCI N1 of the response has unexpected fields %s' % d['f'])
            ok = False
        if not (d['window'][0] <= d['k0'] <= d['window'][1]):
            print('FAIL: NPDCCH at %d is outside the window %s' % (d['k0'], d['window']))
            ok = False

    # the responses are distinct transmissions with distinct temporary C-RNTIs
    tcs = [r['tc_rnti'] for _, _, _, r in per_preamble]
    if len(set(tcs)) != len(tcs):
        print('FAIL: temporary C-RNTIs repeat: %s' % sorted(tcs))
        ok = False
    if any(t < 0x3D or t > 0xFFF3 for t in tcs):
        print('FAIL: temporary C-RNTI outside the C-RNTI range')
        ok = False

    # every response line of the console is a transmission the UE decoded, at the times claimed, with the claimed fields
    n_match = 0
    for s in said:
        hit = [d for d in decoded.values() if d is not None and d['rnti'] == s['ra_rnti'] and d['k0'] <= s['npdcch'][0] <= d['ts'][-1]]
        if not hit:
            print('FAIL: srsenb says it sent a response %s that the UE model does not find' % s)
            ok = False
            continue
        d = hit[0]
        res = uem.parse_rar_pdu(d['tb']) if d['tb'] else None
        times_ok = (d['ts'][0], d['ts'][-1]) == s['npdcch'] and (d['plan'][0][0], d['plan'][-1][0]) == s['npdsch'] \
            and d['window'] == s['window']
        content_ok = res is not None and any(r['rapid'] == s['preamble'] and r['ta'] == s['ta'] and r['tc_rnti'] == s['tc']
                                              for r in res[1])
        if not (times_ok and content_ok):
            print('FAIL: srsenb says %s, the UE decoded NPDCCH %d..%d NPDSCH %d..%d window %s, %s' %
                  (s, d['ts'][0], d['ts'][-1], d['plan'][0][0], d['plan'][-1][0], d['window'], res))
            ok = False
        else:
            n_match += 1

    # the decoded transmissions are what the composer model has to expect in those subframes
    n_dyn = 0
    for d in decoded.values():
        if d is None or d['tb'] is None:
            continue
        plan = dlch.model_plan_npdcch(valid, d['k0'], cfg['r_max'])
        for t, reinit, pos in plan:
            _, g = ch.npdcch_grid(d['dci'], d['rnti'], reinit, pos)
            get_model(t // 10240).dyn[t] = (chk.HAS_NPDCCH, g)
        for t, cw, psfn, psf in d['plan']:
            _, g = ch.npdsch_grid(d['tb'], d['rnti'], d['n_sf'], cw, psfn, psf)
            get_model(t // 10240).dyn[t] = (chk.HAS_NPDSCH, g)
        n_dyn += 1
    print('random access: %d preambles answered in their window (of %d due), %d responses decoded, %d of the %d '
          'the eNB reports found on the air with the times and content it reports, %d not answered' %
          (len(per_preamble), len(due), n_dyn, n_match, len(said), len(unanswered)))
    if len(per_preamble) < 8:
        print('FAIL: too few random access responses were exercised (%d)' % len(per_preamble))
        ok = False
    return ok


def main():
    if len(sys.argv) < 7:
        print(__doc__)
        return 2
    srsenb, enb_conf, nb_conf, sib_pack, seconds = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], float(sys.argv[5])
    for d in sys.argv[6:]:
        sys.path.insert(0, d)
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import enb_dl_nbiot_check as chk
    import nbiot_dlch_check as dlch
    import nbiot_ra_ue_model as uem
    import nbiot_sched_check as sched_mod
    import nbiot_sync_check as sync

    nb = open(nb_conf).read()
    nof_prb = conf_int(nb, 'n_prb')
    if os.path.isdir(enb_conf):
        enb_conf = make_config_dir(enb_conf, nof_prb, conf_int(nb, 'dl_earfcn'), conf_int(nb, 'pci'))
    anchor = conf_int(nb, 'nbiot_prb')
    pci = conf_int(nb, 'n_id_ncell')
    sched = conf_int(nb, 'sched_info_sib1')
    tag = conf_int(nb, 'sys_info_tag')
    ab = re.search(r'ac_barring\s*=\s*true', nb) is not None
    crs_seq = crs_seq_info(nof_prb, anchor)
    # SI message 0 (SIB2-NB): periodicity, repetition pattern, TB and window from the carrier file
    si_cfg = re.search(r'si_periodicity\s*=\s*(\d+);\s*si_repetition_pattern\s*=\s*(\d+);\s*si_tb\s*=\s*(\d+)', nb)
    si_periodicity, si_pattern, si_tb = (int(si_cfg.group(i)) for i in (1, 2, 3))
    si_window = conf_int(nb, 'si_window_length')
    si_offset = conf_int(nb, 'si_radio_frame_offset')

    with tempfile.TemporaryDirectory() as td:
        sib1_f, sib2_f = os.path.join(td, 'sib1.bin'), os.path.join(td, 'sib2.bin')
        r = subprocess.run([sib_pack, '-c', nb_conf, '-H', '0', '-o', sib1_f, '-s', sib2_f], capture_output=True, text=True)
        if r.returncode != 0:
            print('FAIL: nbiot_sib_pack: %s' % r.stderr)
            return 1
        sib1, sib2 = open(sib1_f, 'rb').read(), open(sib2_f, 'rb').read()

    nprach = {'period': conf_int(nb, 'periodicity_ms'), 'start': conf_int(nb, 'start_time_ms'),
              'sc_offset': conf_int(nb, 'subcarrier_offset'), 'n_sc': conf_int(nb, 'nof_subcarriers'),
              'n_rep': conf_int(nb, 'num_repetitions_per_preamble'), 'cell_id': pci,
              'cp_us': float(re.search(r'cp_length_us\s*=\s*([\d.]+)', nb).group(1))}
    import nprach_ref
    n_opps = int(seconds * 1000 / nprach['period']) + 2
    bursts, expected = nprach_bursts(nprach_ref, nprach, nof_prb, anchor, n_opps, SF_LEN * 1000)
    radio = Radio(bursts, noise_var=float((SF_LEN * 1000) // 1920000))
    radio.start()
    log = tempfile.NamedTemporaryFile(prefix='srsenb_nbiot_', suffix='.log', delete=False)
    enb = subprocess.Popen([srsenb, enb_conf, '--expert.nbiot_config=' + nb_conf, '--expert.lte_sample_rates=true'],
                           stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, cwd=os.path.dirname(enb_conf))
    t_end = time.time() + seconds
    try:
        while time.time() < t_end and enb.poll() is None:
            time.sleep(0.2)
    finally:
        if enb.poll() is None:
            enb.terminate()
        try:
            enb.wait(timeout=15)
        except subprocess.TimeoutExpired:
            enb.kill()
        radio.stop.set()
        time.sleep(0.5)
    log.close()
    console = open(log.name).read()
    if os.environ.get('NBIOT_TEST_LOG_LEVEL'):
        lf = os.path.join(os.path.dirname(enb_conf), 'enb.log')
        if os.path.exists(lf):
            print('--- NB-IoT lines of enb.log')
            print(''.join(l for l in open(lf) if 'NPRACH' in l or 'NB-IoT' in l))
    if 'eNodeB started' not in console:
        print('FAIL: srsenb did not start:\n%s' % console[-2000:])
        return 1

    x = radio.samples()
    nsf = len(x) // SF_LEN
    print('captured %d subframes (%.1f s)' % (nsf, nsf / 1000))
    if nsf < 2000:
        print('FAIL: too little downlink captured')
        return 1
    sfs = x[:nsf * SF_LEN].reshape(nsf, SF_LEN)
    power = np.mean(np.abs(sfs) ** 2, axis=1)
    first = int(np.argmax(power > 1e-8))
    print('first subframe with signal: %d' % first)

    def make_model(hfn):
        mibs = {b: mib_nb_bits(b, hfn, sched, tag, ab, crs_seq) for b in range(0, 1024, 64)}
        m = chk.Model(pci, sched, hfn, sync, sched_mod, mibs, sib1, [([1, si_periodicity, si_offset, si_pattern, si_tb, si_window], sib2)],
                      nof_prb, anchor)
        m.prepare_si(hfn * 1024, hfn * 1024 + 1100)
        return m

    models = {}

    def get_model(hfn):
        if hfn not in models:
            models[hfn] = make_model(hfn)
        return models[hfn]

    grids = {}
    def grid(i):
        if i not in grids:
            grids[i] = demod(sfs[i], nof_prb)
        return grids[i]

    lo, hi = 12 * anchor, 12 * anchor + 12

    def compare(i, tti, full=False):
        """Relative error of anchor PRB of captured subframe i against the model for transmit TTI tti."""
        hfn, sfn, sf = tti // 10240, (tti // 10) % 1024, tti % 10
        g = grid(i)
        pre = np.zeros((14, 12), dtype=complex)
        for (l, k), v in lte_crs_values(chk, pci, sf, nof_prb, anchor).items():
            pre[l, k] = v
        pre[0:3, :] = g[0:3, lo:hi]                      # LTE control region: not under test
        exp, flags = get_model(hfn).expected(sfn, sf, pre)
        rx = g[3:, lo:hi].ravel()
        ex = exp[3:, :].ravel()
        gain = np.vdot(ex, rx) / max(np.vdot(ex, ex).real, 1e-30)
        err = np.linalg.norm(rx - gain * ex) / max(np.linalg.norm(gain * ex), 1e-30)
        return err, abs(gain), flags

    # Transmit TTI of the first captured subframe: the radio starts at TTI 0, but find it instead of assuming it
    best = None
    for t0 in range(0, 400):
        e = sum(compare(first + j, t0 + j)[0] for j in range(40))
        if best is None or e < best[0]:
            best = (e, t0)
    t0 = best[1]
    print('alignment: captured subframe %d is TTI %d (mean error %.4f over 40 subframes)' % (first, t0, best[0] / 40))

    # ---- Msg2: decode the responses the way a UE would, and let the composer model expect them
    lo_, hi_ = lo, hi

    def get_grid(t):
        i = first + (t - t0)
        if i < 0 or i >= nsf:
            return None
        # a UE sees the anchor without the in-band phase of TS 36.211 10.2.8
        return grid(i)[:, lo_:hi_] * np.conj(chk.inband_phase(nof_prb, anchor, t % 10))[:, None]

    ra_cfg = {'pci': pci, 'n_rep': nprach['n_rep'], 'fmt': 1 if nprach['cp_us'] > 100.0 else 0,
              'window_pp': conf_int(nb, 'ra_response_window'), 'r_max': conf_int(nb, 'npdcch_num_repetitions_ra'),
              'g_halves': int(round(2 * float(re.search(r'npdcch_start_sf_css_ra\s*=\s*([\d.]+)', nb).group(1)))),
              'offset_eighths': {'zero': 0, 'oneEighth': 1, 'oneFourth': 2, 'threeEighth': 3}[
                  re.search(r'npdcch_offset_ra\s*=\s*"(\w+)"', nb).group(1)]}
    valid = dlch.make_valid(sched_mod, pci, sched, [(1, si_periodicity, si_offset, si_pattern, si_tb, si_window)])
    chan = dlch.Channels(chk, nof_prb, 0, pci)
    # Preambles srsenb reported, for which the capture holds the whole response window, or at least the whole response
    # srsenb says it sent (a response in the first period of a window that ends after the capture still has to be there)
    seen = {(int(m.group(2)), int(m.group(1))) for m in re.finditer(r'NB-IoT: NPRACH preamble (\d+) at TTI (\d+) ', console)}
    t_last = t0 + (nsf - first) - 1
    said = read_responses(console)

    def eligible(e):
        if (e[0], e[1]) not in seen:
            return False
        ws = uem.window_first_sf(e[6], ra_cfg['n_rep'], ra_cfg['fmt'])
        if ws + ra_cfg['window_pp'] * uem.ss_period(ra_cfg['r_max'], ra_cfg['g_halves']) + 60 < t_last:
            return True
        return any(r['preamble'] == e[1] and r['ra_rnti'] == uem.ra_rnti(e[6]) and r['window'][0] == ws and
                   r['npdsch'][1] < t_last for r in said)

    due = [e for e in expected if eligible(e)]
    ok_ra = check_random_access(console, due, ra_cfg, get_grid, uem, dlch, chk, chan, valid, get_model,
                                (1, 6, 0, 0, 0))

    worst, gains, nbad = 0.0, [], 0
    kinds = {}
    names = {1: 'NPSS', 2: 'NSSS', 4: 'NRS', 8: 'NPBCH', 16: 'SIB1-NB', 32: 'SI(SIB2-NB)', chk.HAS_NPDCCH: 'NPDCCH', chk.HAS_NPDSCH: 'NPDSCH'}
    n = 0
    for i in range(first, nsf):
        tti = t0 + (i - first)
        err, gain, flags = compare(i, tti)
        gains.append(gain)
        worst = max(worst, err)
        if err > 0.02:
            nbad += 1
            if nbad <= 8:
                print('  MISMATCH subframe %d (TTI %d, sfn %d.%d hfn %d): error %.3f flags %d' %
                      (i, tti, (tti // 10) % 1024, tti % 10, tti // 10240, err, flags))
        for k in names:
            if flags & k:
                kinds[k] = kinds.get(k, 0) + 1
        n += 1
    print('anchor PRB compared in %d subframes: worst relative error %.4f, %d above 0.02' % (n, worst, nbad))
    print('gain (scale of the eNB output vs the model): min %.4f max %.4f' % (min(gains), max(gains)))
    print('exercised: ' + ', '.join('%s %d' % (names[k], kinds.get(k, 0)) for k in names))
    ok = nbad == 0 and ok_ra
    for k in (1, 2, 4, 8, 16, 32, chk.HAS_NPDCCH, chk.HAS_NPDSCH):
        if kinds.get(k, 0) == 0:
            print('FAIL: no %s subframe in the capture' % names[k])
            ok = False
    if max(gains) - min(gains) > 0.02 * max(gains):
        print('FAIL: output gain not constant')
        ok = False

    # LTE side: the other PRBs of the anchor's RBG must hold LTE CRS and nothing else -- the LTE scheduler reserved the
    # whole RBG (the allocation unit of LTE downlink type 0), so no LTE data may sit next to the anchor, and none of the
    # NB-IoT signal may leak over. Uses the PRB count to get the RBG size (TS 36.213 Table 7.1.6.1-1).
    p_rbg = 1 if nof_prb <= 10 else 2 if nof_prb <= 26 else 3 if nof_prb <= 63 else 4
    partners = [q for q in range(anchor // p_rbg * p_rbg, min(anchor // p_rbg * p_rbg + p_rbg, nof_prb)) if q != anchor]
    crs_l3 = {(l, k) for (l, k) in chk.crs_positions(pci) if l >= 3}
    mask = np.ones((11, 12), dtype=bool)
    for (l, k) in crs_l3:
        mask[l - 3, k] = False
    leak = checked = 0
    for i in range(first, nsf):
        g = grid(i)
        ref = np.mean(np.abs(g[3:, lo:hi]) ** 2)
        for q in partners:
            blk = np.abs(g[3:, 12 * q:12 * q + 12]) ** 2
            checked += 1
            if np.max(blk[mask]) > 1e-4 * max(ref, 1e-12):
                leak += 1
    print('RBG partner PRBs %s of anchor %d: energy beyond CRS in %d of %d subframes' % (partners, anchor, leak, checked))
    if partners and leak:
        ok = False
        print('FAIL: LTE data or NB-IoT leakage next to the anchor')

    # ---- uplink: NPRACH detections printed by srsenb against what was injected
    ok = check_nprach(console, expected, nprach, radio.ul_chunks, first_detect_margin=1000) and ok

    os.unlink(log.name)
    print('PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
