#!/usr/bin/env python3
"""
Checks the NB-IoT addressed downlink channels (nbiot_dlch.c) and the transmission planning (nbiot_dl_sched.c) against a
model written from the specification text, sharing no code with the C implementation.

    nbiot_dlch_check.py <nbiot_dlch_dump> <enb test dir>

Modelled, each from the clause in brackets
  DCI formats N1 and N0 field layout                 (TS 36.212 6.4.3.1, 6.4.3.2)
  Tables N_SF, N_Rep, k0, TBS                        (TS 36.213 Tables 16.4.1.3-1, 16.4.1.3-2, 16.4.1-1, 16.4.1.5.1-1)
  NPDCCH: CRC-16 masked with the RNTI (msb first), tail-biting code, rate matching to two bits per available RE, scrambling
          that runs on over the (up to) four subframes of a group with c_init = floor(ns/2) 2^9 + N_ID, QPSK, mapping to
          the anchor PRB avoiding NRS and CRS from symbol 3 on         (36.212 5.3.3; 36.211 10.2.5)
  NPDSCH: CRC-24A, tail-biting code, rate matching to N_SF subframes, scrambling with
          c_init = RNTI 2^14 + (n_f mod 2) 2^13 + floor(ns/2) 2^9 + N_ID over the whole codeword, subframe by subframe
                                                                          (36.212 6.4.2; 36.211 10.2.3)
  Search space start                                 (36.213 16.6: (10 n_f + floor(ns/2)) mod T = floor(alpha T), T = Rmax G)
  Planning of NPDCCH and NPDSCH subframes over the valid NB-IoT downlink subframes, the "each subframe min(N_Rep, 4) times,
  then the next" order and the pass structure                          (36.213 16.4.1, 16.6; 36.211 10.2.3.4)

The enb test dir provides the shared primitives (Gold sequence, CRC, convolutional code, rate matching) of the composer
check. The scheduling model of SIB1-NB and SI messages comes from nbiot_sched_check.py in the same directory as this file.
"""
import os
import random
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

TOL = 2e-6


def load_primitives(enb_test_dir):
    sys.path.insert(0, enb_test_dir)
    import enb_dl_nbiot_check as m
    return m


# ------------------------------------------------------------------------------------------------- spec tables
N_SF = [1, 2, 3, 4, 5, 6, 8, 10]
N_REP = [1, 2, 4, 8, 16, 32, 64, 128, 192, 256, 384, 512, 768, 1024, 1536, 2048]
K0_LO = [0, 4, 8, 12, 16, 32, 64, 128]
K0_HI = [0, 16, 32, 64, 128, 256, 512, 1024]
TBS = [
    [16, 32, 56, 88, 120, 152, 208, 256],
    [24, 56, 88, 144, 176, 208, 256, 344],
    [32, 72, 144, 176, 208, 256, 328, 424],
    [40, 104, 176, 208, 256, 328, 440, 568],
    [56, 120, 208, 256, 328, 408, 552, 680],
    [72, 144, 224, 328, 424, 504, 680, 872],
    [88, 176, 256, 392, 504, 600, 808, 1032],
    [104, 224, 328, 472, 584, 680, 968, 1224],
    [120, 256, 392, 536, 680, 808, 1096, 1352],
    [136, 296, 456, 616, 776, 936, 1256, 1544],
    [144, 328, 504, 680, 872, 1032, 1384, 1736],
    [176, 376, 584, 776, 1000, 1192, 1608, 2024],
    [208, 440, 680, 904, 1128, 1352, 1800, 2280],
    [224, 488, 744, 1128, 1256, 1544, 2024, 2536],
]


def bits_of(value, width):
    return [(value >> (width - 1 - i)) & 1 for i in range(width)]


def dci_n1(i_delay, i_sf, i_mcs, i_rep, ndi, harq, dci_rep):
    # flag (1 = N1), NPDCCH order indicator (0), scheduling delay 3, resource assignment 3, MCS 4, repetitions 4,
    # NDI 1, HARQ-ACK resource 4, DCI subframe repetition number 2
    return (bits_of(1, 1) + bits_of(0, 1) + bits_of(i_delay, 3) + bits_of(i_sf, 3) + bits_of(i_mcs, 4) +
            bits_of(i_rep, 4) + bits_of(ndi, 1) + bits_of(harq, 4) + bits_of(dci_rep, 2))


def dci_n0(i_sc, i_ru, i_delay, i_mcs, rv, i_rep, ndi, dci_rep):
    # flag (0 = N0), subcarrier indication 6, resource assignment 3, scheduling delay 2, MCS 4, RV 1, repetitions 3,
    # NDI 1, DCI subframe repetition number 2
    return (bits_of(0, 1) + bits_of(i_sc, 6) + bits_of(i_ru, 3) + bits_of(i_delay, 2) + bits_of(i_mcs, 4) +
            bits_of(rv, 1) + bits_of(i_rep, 3) + bits_of(ndi, 1) + bits_of(dci_rep, 2))


def bstr(bits):
    return ''.join(str(b) for b in bits)


# --------------------------------------------------------------------------------------------- driver to the dump
class Dump:
    def __init__(self, exe):
        self.p = subprocess.Popen([exe], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)

    def ask(self, line, end_marker=None):
        self.p.stdin.write(line + '\n')
        self.p.stdin.flush()
        if end_marker is None:
            return self.p.stdout.readline().strip()
        raise RuntimeError

    def ask_grid(self, line):
        """Commands answering with 'E bits' followed by RE lines: reads until the process is idle via a sentinel."""
        self.p.stdin.write(line + '\n' + 'SENTINEL\n')
        self.p.stdin.flush()
        e = None
        res = {}
        failed = False
        while True:
            ln = self.p.stdout.readline()
            if not ln:
                raise RuntimeError('dump program died')
            ln = ln.strip()
            if ln == 'SENTINEL_DONE':
                break
            if ln.startswith('ERROR'):
                failed = True
                continue
            if ln.startswith('E '):
                e = [int(c) for c in ln[2:]]
            elif ln.startswith('RE '):
                _, l, k, re, im = ln.split()
                res[(int(l), int(k))] = complex(float(re), float(im))
        if failed:
            return None, None
        return e, res

    def close(self):
        self.p.stdin.close()
        self.p.wait()


# ------------------------------------------------------------------------------------------------- channel models
class Channels:
    def __init__(self, prim, nof_prb, anchor, pci):
        self.m = prim
        self.pci, self.anchor, self.nof_prb = pci, anchor, nof_prb
        nrs = {rc for rc in prim.nrs_positions(pci, 1).values()}
        crs = prim.crs_positions(pci)
        self.pos = [(l, k) for l in range(3, 14) for k in range(12) if (l, k) not in nrs and (l, k) not in crs]
        self.e_sf = 2 * len(self.pos)

    def map(self, sym):
        out = {}
        for (l, k), v in zip(self.pos, sym):
            out[(l, self.anchor * 12 + k)] = v
        return out

    def npdcch_e(self, dci, rnti):
        m = self.m
        crc = m.crc16(dci)
        crc = [b ^ ((rnti >> (15 - i)) & 1) for i, b in enumerate(crc)]
        return m.rate_match(m.tbcc(dci + crc), self.e_sf)

    def npdcch_grid(self, dci, rnti, reinit_sf, pos_in_group):
        m = self.m
        e = self.npdcch_e(dci, rnti)
        c = m.gold((reinit_sf << 9) + self.pci, (pos_in_group + 1) * self.e_sf)[pos_in_group * self.e_sf:]
        scr = [a ^ b for a, b in zip(e, c)]
        return e, self.map(m.qpsk(scr))

    def npdsch_e(self, tb_bits, nof_sf):
        m = self.m
        c = tb_bits + m.crc24a(tb_bits)
        return m.rate_match(m.tbcc(c), nof_sf * self.e_sf)

    def npdsch_grid(self, tb_bits, rnti, nof_sf, sf_in_cw, pass_sfn, pass_sf):
        m = self.m
        e = self.npdsch_e(tb_bits, nof_sf)
        c_init = (rnti << 14) + ((pass_sfn & 1) << 13) + (pass_sf << 9) + self.pci
        c = m.gold(c_init, nof_sf * self.e_sf)
        scr = [a ^ b for a, b in zip(e, c)]
        sl = scr[sf_in_cw * self.e_sf:(sf_in_cw + 1) * self.e_sf]
        return e, self.map(m.qpsk(sl))


def grids_equal(got, want):
    if set(got) != set(want):
        return False
    return all(abs(got[k] - want[k]) < TOL for k in want)


# -------------------------------------------------------------------------------------------------- plan models
def make_valid(sm, pci, sched, si_cfgs):
    """valid(t): NB-IoT downlink subframe with no SIB1-NB and no SI message (TS 36.213 16.4, 16.6)."""
    sib1 = sm.sib1_model(pci, sched)
    hfn_cache = {}

    def si_at(t):
        f, sf = divmod(t, 10)
        h = f // 1024
        if h not in hfn_cache:
            lo, hi = h * 1024 - 8, h * 1024 + 1024 + 8
            hfn_cache[h] = [sm.si_model(pci, sched, tuple(cfg), lo, hi) for cfg in si_cfgs]
        return any((f, sf) in s for s in hfn_cache[h])

    def valid(t):
        f, sf = divmod(t, 10)
        sfn = f % 1024
        if not sm.valid_dl(sfn, sf):
            return False
        if (sfn, sf) in sib1:
            return False
        if si_at(t):
            return False
        return True

    return valid


def next_valid(valid, t):
    while not valid(t):
        t += 1
    return t


def model_ss(r_max, g_halves, offset_eighths, t_min):
    if (r_max * g_halves) % 2:
        return 0, 0
    T = r_max * g_halves // 2
    if T < 4:
        return 0, 0
    want = (offset_eighths * T) // 8
    t = t_min
    while (t % 10240) % T != want:
        t += 1
    return t, T


def model_plan_npdcch(valid, k0, r):
    ts = []
    t = next_valid(valid, k0)
    for _ in range(r):
        ts.append(t)
        t = next_valid(valid, t + 1)
    out = []
    for i, t in enumerate(ts):
        first = ts[i - i % 4]
        out.append((t, (first % 10240) % 10, i % 4))
    return out


def model_plan_npdsch(valid, n_last, k0, n_sf, n_rep):
    t = next_valid(valid, n_last + 5)
    for _ in range(k0):
        t = next_valid(valid, t + 1)
    times = []
    for _ in range(n_sf * n_rep):
        times.append(t)
        t = next_valid(valid, t + 1)
    m = min(n_rep, 4)
    out = []
    idx = 0
    for p in range(n_rep // m):
        start = times[p * n_sf * m]
        for cw in range(n_sf):
            for _ in range(m):
                out.append((times[idx], cw, (start % 10240) // 10, (start % 10240) % 10))
                idx += 1
    return out


# ------------------------------------------------------------------------------------------------------- tests
class Failure(Exception):
    pass


def check(cond, msg):
    if not cond:
        raise Failure(msg)


def run_all(dump_exe, enb_dir, seed):
    prim = load_primitives(enb_dir)
    import nbiot_sched_check as sm
    rnd = random.Random(seed)
    d = Dump(dump_exe)
    n_checks = {}

    def count(name):
        n_checks[name] = n_checks.get(name, 0) + 1

    # ---- fields and tables
    for _ in range(300):
        f = (rnd.randrange(8), rnd.randrange(8), rnd.randrange(16), rnd.randrange(16), rnd.randrange(2),
             rnd.randrange(16), rnd.randrange(4))
        got = d.ask('DCIN1 ' + ' '.join(map(str, f)))
        check(got == 'DCI ' + bstr(dci_n1(*f)), 'DCI N1 %s: %s' % (f, got))
        back = d.ask('UNPACKN1 ' + bstr(dci_n1(*f)))
        check(back == 'UNPACK ' + ' '.join(map(str, f)), 'DCI N1 unpack %s: %s' % (f, back))
        count('dci_n1')
        g = (rnd.randrange(64), rnd.randrange(8), rnd.randrange(4), rnd.randrange(16), rnd.randrange(2),
             rnd.randrange(8), rnd.randrange(2), rnd.randrange(4))
        got = d.ask('DCIN0 ' + ' '.join(map(str, g)))
        check(got == 'DCI ' + bstr(dci_n0(*g)), 'DCI N0 %s: %s' % (g, got))
        back = d.ask('UNPACKN0 ' + bstr(dci_n0(*g)))
        check(back == 'UNPACK ' + ' '.join(map(str, g)), 'DCI N0 unpack %s: %s' % (g, back))
        count('dci_n0')
    # out of range values are refused, not truncated
    check(d.ask('DCIN1 8 0 0 0 0 0 0') == 'DCI ERROR', 'N1 delay 8 accepted')
    check(d.ask('DCIN1 0 0 0 16 0 0 0') == 'DCI ERROR', 'N1 rep 16 accepted')
    check(d.ask('DCIN0 0 0 4 0 0 0 0 0') == 'DCI ERROR', 'N0 delay 4 accepted')
    check(d.ask('DCIN0 64 0 0 0 0 0 0 0') == 'DCI ERROR', 'N0 sc 64 accepted')
    # the unpackers refuse the other format's flag
    check(d.ask('UNPACKN1 ' + bstr(dci_n0(0, 0, 0, 0, 0, 0, 0, 0))) == 'UNPACK ERROR', 'N1 unpack took an N0')
    check(d.ask('UNPACKN0 ' + bstr(dci_n1(0, 0, 0, 0, 0, 0, 0))) == 'UNPACK ERROR', 'N0 unpack took an N1')

    for i in range(9):
        check(d.ask('TABLE nsf %d' % i) == 'TABLE %d' % (N_SF[i] if i < 8 else 0), 'N_SF %d' % i)
    for i in range(17):
        check(d.ask('TABLE nrep %d' % i) == 'TABLE %d' % (N_REP[i] if i < 16 else 0), 'N_Rep %d' % i)
    for i in range(8):
        for rmax in (1, 8, 64, 127, 128, 2048):
            want = (K0_LO if rmax < 128 else K0_HI)[i]
            check(d.ask('TABLE k0 %d %d' % (i, rmax)) == 'TABLE %d' % want, 'k0 %d %d' % (i, rmax))
    check(d.ask('TABLE k0 8 8') == 'TABLE -1', 'k0 index 8')
    for t in range(15):
        for s in range(9):
            want = TBS[t][s] if t < 14 and s < 8 else 0
            check(d.ask('TABLE tbs %d %d' % (t, s)) == 'TABLE %d' % want, 'TBS %d %d' % (t, s))
    count('tables')

    # ---- channels, several cells
    for (nof_prb, anchor, pci) in [(25, 17, 1), (25, 2, 7), (50, 14, 300), (100, 44, 503), (15, 7, 0), (25, 22, 41)]:
        cell = Channels(prim, nof_prb, anchor, pci)
        got_nre = d.ask('CELL %d %d %d' % (nof_prb, anchor, pci))
        check(got_nre == 'NOF_RE %d' % len(cell.pos), 'RE count %s vs %d' % (got_nre, len(cell.pos)))

        for _ in range(40):
            rnti = rnd.choice([rnd.randrange(1, 0x10000), 0x0001, 0x0102, 0xFFFF, 0x0100])
            dci = dci_n1(rnd.randrange(8), rnd.randrange(8), rnd.randrange(11), rnd.randrange(16), 0, 0, rnd.randrange(4))
            reinit, pos = rnd.randrange(10), rnd.randrange(4)
            e, grid = d.ask_grid('NPDCCH %d %d %d %s' % (rnti, reinit, pos, bstr(dci)))
            check(e is not None, 'NPDCCH failed')
            we, wg = cell.npdcch_grid(dci, rnti, reinit, pos)
            check(e == we, 'NPDCCH coded bits differ (pci %d rnti %x)' % (pci, rnti))
            check(grids_equal(grid, wg), 'NPDCCH grid differs (pci %d rnti %x reinit %d pos %d)' % (pci, rnti, reinit, pos))
            count('npdcch')

        for _ in range(40):
            rnti = rnd.choice([rnd.randrange(1, 0x10000), 0x0001, 0x0102, 0xFFFF, 0x0100])
            nof_sf = rnd.choice([1, 1, 2, 3, 4, 5, 6, 8, 10])
            i_sf = N_SF.index(nof_sf)
            tbs = TBS[rnd.randrange(11)][i_sf]
            if tbs > 1000:   # beyond what the library's rate matcher holds; refused, checked below
                continue
            tb = bytes(rnd.randrange(256) for _ in range(tbs // 8)) if tbs % 8 == 0 else None
            check(tb is not None, 'TBS %d not byte aligned' % tbs)
            tb_bits = [(b >> (7 - i)) & 1 for b in tb for i in range(8)]
            sf_in_cw = rnd.randrange(nof_sf)
            sfn, sf = rnd.randrange(1024), rnd.randrange(10)
            e, grid = d.ask_grid('NPDSCH %d %d %d %d %d %s' % (rnti, nof_sf, sf_in_cw, sfn, sf, tb.hex()))
            check(e is not None, 'NPDSCH failed')
            we, wg = cell.npdsch_grid(tb_bits, rnti, nof_sf, sf_in_cw, sfn, sf)
            check(e == we, 'NPDSCH codeword differs (pci %d nof_sf %d tbs %d)' % (pci, nof_sf, tbs))
            check(grids_equal(grid, wg), 'NPDSCH grid differs (pci %d rnti %x nsf %d part %d sfn %d sf %d)' %
                  (pci, rnti, nof_sf, sf_in_cw, sfn, sf))
            count('npdsch')

    # ---- search space and plans
    for (r_max, g, off) in [(8, 4, 0), (8, 3, 0), (8, 4, 1), (16, 8, 2), (4, 4, 3), (2, 2, 0), (1, 4, 0), (128, 16, 1),
                            (8, 3, 3), (3, 3, 0), (8, 1, 0)]:
        for _ in range(20):
            t_min = rnd.randrange(0, 5 * 10240)
            got = d.ask('SS %d %d %d %d' % (r_max, g, off, t_min)).split()
            want_t, want_T = model_ss(r_max, g, off, t_min)
            check(got[0] == 'SS' and int(got[1]) == want_t and int(got[2]) == want_T,
                  'search space %s -> %s, want %d %d' % ((r_max, g, off, t_min), got, want_t, want_T))
            count('ss')

    for (pci, sched, si_cfgs) in [
        (1, 4, [(1, 1024, 0, 4, 208, 160)]),
        (1, 4, [(1, 64, 1, 4, 208, 160), (2, 128, 1, 8, 120, 160)]),
        (300, 7, [(1, 64, 1, 4, 208, 160), (2, 128, 1, 8, 120, 160), (3, 2048, 1, 2, 328, 160)]),
        (5, 1, []),
    ]:
        valid = make_valid(sm, pci, sched, si_cfgs)
        d.ask('LAYOUT %d %d %s' % (pci, sched, ' '.join(' '.join(map(str, c)) for c in si_cfgs)))
        for _ in range(150):
            t = rnd.randrange(0, 3 * 10240)
            check(d.ask('VALID %d' % t) == 'VALID %d' % valid(t), 'valid dl subframe %d differs (pci %d)' % (t, pci))
            count('valid')
        # plans, including ones that start inside SI windows and across the frame boundaries
        for _ in range(60):
            k0 = rnd.randrange(0, 2 * 10240)
            r = rnd.choice([1, 2, 4, 8, 16, 32])
            got = d.ask('PLANC %d %d' % (k0, r)).split()[1:]
            want = ['%d,%d,%d' % x for x in model_plan_npdcch(valid, k0, r)]
            check(got == want, 'NPDCCH plan k0=%d r=%d: %s vs %s' % (k0, r, got[:4], want[:4]))
            count('plan_npdcch')
        for _ in range(60):
            n_last = rnd.randrange(0, 2 * 10240)
            k0 = rnd.choice([0, 4, 8, 12, 16, 32])
            n_sf = rnd.choice([1, 2, 3, 4, 6, 8, 10])
            n_rep = rnd.choice([1, 2, 4, 8, 16])
            if n_sf * n_rep > 64:
                continue
            got = d.ask('PLAND %d %d %d %d' % (n_last, k0, n_sf, n_rep)).split()[1:]
            want = ['%d,%d,%d,%d' % x for x in model_plan_npdsch(valid, n_last, k0, n_sf, n_rep)]
            check(got == want, 'NPDSCH plan n_last=%d k0=%d nsf=%d rep=%d: %s vs %s' % (n_last, k0, n_sf, n_rep, got[:4], want[:4]))
            count('plan_npdsch')
    # a transport block the rate matcher cannot hold is refused, not mangled
    d.ask('CELL 25 17 1')
    e, grid = d.ask_grid('NPDSCH 300 10 0 0 0 ' + '00' * 129)
    check(e is None, 'transport block of 1032 bits accepted')
    e, grid = d.ask_grid('NPDSCH 300 1 0 0 0 ' + '00' * 125)
    check(e is not None, 'transport block of 1000 bits refused')
    # plans that do not fit are refused
    check(d.ask('PLAND 0 0 10 8') == 'PLAN ERROR', 'oversized NPDSCH plan accepted')
    check(d.ask('PLANC 0 65') == 'PLAN ERROR', 'oversized NPDCCH plan accepted')

    d.close()
    return n_checks


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 1
    seed = int(os.environ.get('NBIOT_DLCH_SEED', '1'))
    try:
        counts = run_all(sys.argv[1], sys.argv[2], seed)
    except Failure as e:
        print('FAIL:', e)
        return 1
    print('OK', ', '.join('%s %d' % kv for kv in sorted(counts.items())))
    return 0


if __name__ == '__main__':
    sys.exit(main())
