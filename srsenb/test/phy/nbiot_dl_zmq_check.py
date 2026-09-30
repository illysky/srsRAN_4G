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
    """The far end of srsenb's ZMQ radio: silence on the uplink, records the downlink."""

    def __init__(self):
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
        while not self.stop.is_set():
            try:
                s.recv()
            except zmq.Again:
                continue
            s.send(silence)
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
all_level = warning
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
        f.write(ENB_CONF % {'nof_prb': nof_prb, 'log': os.path.join(d, 'enb.log')})
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


def main():
    if len(sys.argv) < 7:
        print(__doc__)
        return 2
    srsenb, enb_conf, nb_conf, sib_pack, seconds = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], float(sys.argv[5])
    for d in sys.argv[6:]:
        sys.path.insert(0, d)
    import enb_dl_nbiot_check as chk
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

    radio = Radio()
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
        m = chk.Model(pci, sched, hfn, sync, sched_mod, mibs, sib1, [([1, si_periodicity, si_offset, si_pattern, si_tb, si_window], sib2)])
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

    worst, gains, nbad = 0.0, [], 0
    kinds = {}
    names = {1: 'NPSS', 2: 'NSSS', 4: 'NRS', 8: 'NPBCH', 16: 'SIB1-NB', 32: 'SI(SIB2-NB)'}
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
    ok = nbad == 0
    for k in (1, 2, 4, 8, 16, 32):
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
    os.unlink(log.name)
    print('PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
