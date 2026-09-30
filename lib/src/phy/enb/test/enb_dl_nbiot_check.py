#!/usr/bin/env python3
"""
Checks the NB-IoT downlink composer (enb_dl_nbiot.c) against a transmitter written from the specification, sharing
no code with the C implementation.

    enb_dl_nbiot_check.py <enb_dl_nbiot_dump> <sync test dir> <phch test dir>

The dump program builds the LTE downlink with the real srsran_enb_dl_t, runs the composer over it for a few hundred
radio frames, and writes the anchor PRB (14 symbols x 12 subcarriers) before and after. The model here produces what
the anchor PRB must look like from the LTE-only grid, the MIB/SIB1/SI bits and the cell parameters, and every one of
the 168 resource elements of every subframe is compared.

Modelled, each from the clause in brackets
  LTE CRS positions                      (36.211 6.10.1.2), cross-checked against the non-zero REs of the real LTE grid
  NRS sequence and positions             (10.2.6.1, 10.2.6.2; sequence init of 6.10.1.1, m' = m + N_RB^max - 1)
  NPSS / NSSS values and placement       (10.2.7.1, 10.2.7.2), values from the sync checker's spec-derived functions
  NPBCH: CRC16 + mask, tail-biting conv. code, rate matching to 1600 bits, scrambling, QPSK, per-frame rotation and
         the 100 RE mapping                (36.212 5.3.1, 5.1.3.1, 5.1.4.2; 36.211 10.2.4)
  BCCH NPDSCH (SIB1-NB and SI): CRC24A, conv. code, rate matching, scrambling with the BCCH initialisation,
         QPSK, mapping over N_SF subframes, NRS/CRS exclusion, l_DataStart   (36.212 5.1.1; 36.211 10.2.3)
  Where SIB1-NB and the SI messages are: the forward simulation of nbiot_sched_check.py
  CRS wins over anything NB-IoT puts on the same RE (10.2.7.1.2)
  With "dyn": NPDCCH and NPDSCH addressed to an RNTI, planned by the eNB over the valid NB-IoT downlink subframes and
  handed to the composer through the shared table -- expected content from nbiot_dlch_check.py (channel coding, scrambling,
  mapping) and its planning model, including refusals of transmissions that overlap one already stored

It also requires that nothing outside the anchor PRB data region changes, that LTE data put there by mistake is
blanked (the "junk" runs) and counted, and that the composer's own report of what each subframe holds matches.
"""
import math
import os
import subprocess
import sys
import tempfile

import numpy as np

TOL = 2e-5
R2 = 1 / math.sqrt(2)

HAS_NPSS, HAS_NSSS, HAS_NRS, HAS_NPBCH, HAS_SIB1, HAS_SI = 1, 2, 4, 8, 16, 32
HAS_NPDCCH, HAS_NPDSCH = 64, 128


# ------------------------------------------------------------------------------------------ TS 36.211 7.2
def gold(c_init, n):
    total = 1600 + n
    x1 = np.zeros(total + 31, dtype=np.uint8)
    x2 = np.zeros(total + 31, dtype=np.uint8)
    x1[0] = 1
    for i in range(31):
        x2[i] = (c_init >> i) & 1
    for i in range(total):
        x1[i + 31] = x1[i + 3] ^ x1[i]
        x2[i + 31] = x2[i + 3] ^ x2[i + 2] ^ x2[i + 1] ^ x2[i]
    return (x1[1600:1600 + n] ^ x2[1600:1600 + n]).tolist()


# ------------------------------------------------------------------------------------------ TS 36.212
def crc(bits, poly, width):
    reg = 0
    top = 1 << (width - 1)
    mask = (1 << width) - 1
    for b in bits:
        fb = (1 if reg & top else 0) ^ b
        reg = (reg << 1) & mask
        if fb:
            reg ^= poly
    return [(reg >> (width - 1 - i)) & 1 for i in range(width)]


def crc16(bits):   # gCRC16(D) = D16 + D12 + D5 + 1
    return crc(bits, 0x1021, 16)


def crc24a(bits):  # gCRC24A(D) = D24+D23+D18+D17+D14+D11+D10+D7+D6+D5+D4+D3+D+1
    return crc(bits, 0x864CFB, 24)


G_CONV = (0o133, 0o171, 0o165)


def tbcc(c):
    """5.1.3.1: rate 1/3, constraint length 7, tail biting. Returns the three streams d0, d1, d2."""
    k_len = len(c)
    out = [[0] * k_len for _ in range(3)]
    for k in range(k_len):
        for i, g in enumerate(G_CONV):
            acc = 0
            for t in range(7):
                if (g >> (6 - t)) & 1:
                    acc ^= c[(k - t) % k_len]
            out[i][k] = acc
    return out


P_CC = [1, 17, 9, 25, 5, 21, 13, 29, 3, 19, 11, 27, 7, 23, 15, 31,
        0, 16, 8, 24, 4, 20, 12, 28, 2, 18, 10, 26, 6, 22, 14, 30]   # Table 5.1.4-2


def subblock(d):
    n = len(d)
    rows = -(-n // 32)
    n_d = 32 * rows - n
    y = [None] * n_d + list(d)
    v = []
    for j in range(32):
        col = P_CC[j]
        v.extend(y[r * 32 + col] for r in range(rows))
    return v


def rate_match(streams, e_len):
    w = []
    for s in streams:
        w.extend(subblock(s))
    out = []
    k = 0
    while len(out) < e_len:
        b = w[k % len(w)]
        k += 1
        if b is not None:
            out.append(b)
    return out


def qpsk(bits):
    return np.array([complex(1 - 2 * bits[2 * i], 1 - 2 * bits[2 * i + 1]) * R2 for i in range(len(bits) // 2)])


# ------------------------------------------------------------------------------------------ positions
def crs_positions(pci):
    """LTE CRS, one antenna port, inside one PRB after the control region: (l, k)."""
    vshift = pci % 6
    out = set()
    for slot in (0, 1):
        for ls in (0, 4):
            v = 0 if ls == 0 else 3          # port 0
            for m in (0, 1):
                out.add((slot * 7 + ls, 6 * m + (v + vshift) % 6))
    return out


def crs_positions_4ports(pci):
    vshift = pci % 6
    out = set()
    for slot in (0, 1):
        for p in range(4):
            syms = (0, 4) if p < 2 else (1,)
            for ls in syms:
                if p == 0:
                    v = 0 if ls == 0 else 3
                elif p == 1:
                    v = 3 if ls == 0 else 0
                elif p == 2:
                    v = 3 * slot
                else:
                    v = 3 + 3 * slot
                for m in (0, 1):
                    out.add((slot * 7 + ls, 6 * m + (v + vshift) % 6))
    return out


def nrs_positions(pci, nports):
    vshift = pci % 6
    out = {}
    for p in range(nports):
        for slot in (0, 1):
            for ls in (5, 6):
                if p == 0:
                    v = 0 if ls == 5 else 3
                else:
                    v = 3 if ls == 5 else 0
                for m in (0, 1):
                    out[(p, slot, ls, m)] = (slot * 7 + ls, 6 * m + (v + vshift) % 6)
    return out


def nrs_values(pci, sf):
    """r_{l,ns}(m') for m = 0, 1 (10.2.6.1 via 6.10.1.1, N_RB^max,DL = 110), both slots, symbols 5 and 6."""
    vals = {}
    for slot in (0, 1):
        ns = 2 * sf + slot
        for ls in (5, 6):
            c_init = 1024 * (7 * (ns + 1) + ls + 1) * (2 * pci + 1) + 2 * pci + 1
            c = gold(c_init, 2 * (109 + 2))
            for m in (0, 1):
                mp = m + 109
                vals[(slot, ls, m)] = complex(1 - 2 * c[2 * mp], 1 - 2 * c[2 * mp + 1]) * R2
    return vals


class Model:
    def __init__(self, pci, sched, hfn, sync, sched_mod, mib_bits, sib1, si_list):
        self.dyn = {}          # absolute subframe -> (flag, {(l, k): value})
        self.pci, self.sched, self.hfn = pci, sched, hfn
        self.sync = sync
        self.sm = sched_mod
        self.mib_bits = mib_bits          # sfn_base -> 34 bits
        self.sib1 = sib1
        self.si_list = si_list            # [(cfg, bytes)]
        self.crs = crs_positions(pci)
        self.nrs_pos = nrs_positions(pci, 1)
        self.nrs_cache = {}
        self.sib1_sched = sched_mod.sib1_model(pci, sched)
        self.si_sched = {}
        self.sym_cache = {}
        self.npbch_cache = None
        self.b = [[1 if ch == '+' else -1 for ch in row] for row in sync.BQ_EMBEDDED]

        nrs_res = {rc for rc in self.nrs_pos.values()}
        self.npdsch_pos = [(l, k) for l in range(3, 14) for k in range(12)
                           if (l, k) not in nrs_res and (l, k) not in self.crs]
        crs4 = crs_positions_4ports(pci)
        nrs2 = {rc for rc in nrs_positions(pci, 2).values()}
        self.npbch_pos = [(l, k) for l in range(3, 14) for k in range(12) if (l, k) not in nrs2 and (l, k) not in crs4]
        assert len(self.npbch_pos) == 100, len(self.npbch_pos)

    def prepare_si(self, frame_lo, frame_hi):
        for idx, (cfg, _) in enumerate(self.si_list):
            self.si_sched[idx] = self.sm.si_model(self.pci, self.sched, tuple(cfg), frame_lo, frame_hi)

    # ---- pieces
    def nrs_grid(self, sf):
        if sf not in self.nrs_cache:
            vals = nrs_values(self.pci, sf)
            g = {}
            for (p, slot, ls, m), (l, k) in self.nrs_pos.items():
                g[(l, k)] = vals[(slot, ls, m)]
            self.nrs_cache[sf] = g
        return self.nrs_cache[sf]

    def npbch_symbols(self, sfn_base):
        if sfn_base not in self.sym_cache.setdefault('npbch', {}):
            payload = self.mib_bits[sfn_base]
            c = payload + crc16(payload)            # one NRS port: CRC mask of zeros
            e = rate_match(tbcc(c), 1600)
            scr = [a ^ b for a, b in zip(e, gold(self.pci, 1600))]
            self.sym_cache['npbch'][sfn_base] = qpsk(scr)
        return self.sym_cache['npbch'][sfn_base]

    def theta(self, nf):
        c_init = (self.pci + 1) * ((nf % 8) + 1) ** 3 * 2 ** 9 + self.pci
        c = gold(c_init, 200)
        th = {(0, 0): 1, (0, 1): -1, (1, 0): 1j, (1, 1): -1j}
        return np.array([th[(c[2 * i], c[2 * i + 1])] for i in range(100)])

    def bcch_symbols(self, key, payload, start_sfn, nof_sf):
        if key not in self.sym_cache:
            bits = []
            for byte in payload:
                bits.extend((byte >> (7 - i)) & 1 for i in range(8))
            c = bits + crc24a(bits)
            n_re = len(self.npdsch_pos)
            e_len = 2 * n_re * nof_sf
            e = rate_match(tbcc(c), e_len)
            c_init = (0xFFFF << 15) + (self.pci + 1) * ((start_sfn % 61) + 1)
            scr = [a ^ b for a, b in zip(e, gold(c_init, e_len))]
            self.sym_cache[key] = qpsk(scr)
        return self.sym_cache[key]

    # ---- expected anchor PRB
    def expected(self, sfn, sf, pre):
        """Returns (grid 14x12 complex, flag mask)."""
        g = np.zeros((14, 12), dtype=complex)
        flags = 0
        nsss_sf = (sf == 9 and sfn % 2 == 0)

        if sf == 5:
            for l in range(3, 14):
                for n in range(11):
                    g[l, n] = self.sync.npss_d(l, n)
            flags |= HAS_NPSS
        if nsss_sf:
            theta_idx = (sfn // 2) % 4
            for n in range(132):
                g[3 + n // 12, n % 12] = self.sync.nsss_d(n, self.pci, theta_idx, self.b)
            flags |= HAS_NSSS
        if sf != 5 and not nsss_sf:
            for (l, k), v in self.nrs_grid(sf).items():
                g[l, k] = v
            flags |= HAS_NRS

        if sf == 0:
            base = sfn - sfn % 64
            y = self.npbch_symbols(base)
            blk = (sfn % 64) // 8
            yf = y[100 * blk:100 * blk + 100] * self.theta(sfn)
            for i, (l, k) in enumerate(self.npbch_pos):
                g[l, k] = yf[i]
            flags |= HAS_NPBCH

        pos = self.sib1_sched.get((sfn, sf))
        if pos is not None:
            start, idx, nof = pos
            sym = self.bcch_symbols(('sib1', start), self.sib1, start, nof)
            n_re = len(self.npdsch_pos)
            for i, (l, k) in enumerate(self.npdsch_pos):
                g[l, k] = sym[idx * n_re + i]
            flags |= HAS_SIB1
        else:
            f = self.hfn * 1024 + sfn
            for i, (cfg, payload) in enumerate(self.si_list):
                p = self.si_sched[i].get((f, sf))
                if p is not None:
                    start, idx, nof = p
                    sym = self.bcch_symbols(('si', i, start), payload, start, nof)
                    n_re = len(self.npdsch_pos)
                    for j, (l, k) in enumerate(self.npdsch_pos):
                        g[l, k] = sym[idx * n_re + j]
                    flags |= HAS_SI
                    break

        t_abs = (self.hfn * 1024 + sfn) * 10 + sf
        if t_abs in self.dyn:
            assert flags & (HAS_NPSS | HAS_NSSS | HAS_NPBCH | HAS_SIB1 | HAS_SI) == 0, \
                'dynamic transmission planned onto a broadcast subframe %d.%d' % (sfn, sf)
            dflag, cells = self.dyn[t_abs]
            for (l, k), v in cells.items():
                g[l, k] = v
            flags |= dflag

        # CRS wins
        for (l, k) in self.crs:
            g[l, k] = pre[l, k]
        # symbols of the LTE control region are LTE's
        g[0:3, :] = pre[0:3, :]
        return g, flags


# ------------------------------------------------------------------------------------------ driver
def parse_stdout(text):
    out = {'mib': {}, 'sib1': None, 'si': [], 'flags': {}, 'coll': None, 'outside': None}
    for line in text.splitlines():
        p = line.split()
        if not p:
            continue
        if p[0] == 'MIB':
            out['mib'][int(p[1])] = [int(ch) for ch in p[2]]
        elif p[0] == 'SIB1':
            out['sib1'] = bytes.fromhex(p[1])
        elif p[0] == 'SI':
            out['si'].append(([int(x) for x in p[1:7]], bytes.fromhex(p[7])))
        elif p[0] == 'FLAGS':
            out['flags'][(int(p[1]), int(p[2]))] = int(p[3])
        elif p[0] == 'COLL':
            out['coll'] = int(p[1])
        elif p[0] == 'DYN_CONFLICTS':
            out['dyn_conflicts'] = int(p[1])
        elif p[0] == 'DYN':
            out.setdefault('dyn', []).append(p[1:])
        elif p[0] == 'OUTSIDE_DIFF':
            out['outside'] = int(p[1])
    return out


def register_dynamic(model, info, dlch_mod, prim, nof_prb, name):
    """Replays the DYN lines of the dump with the planning model; returns the number of failures found."""
    valid = dlch_mod.make_valid(model.sm, model.pci, model.sched, [tuple(c) for c, _ in model.si_list])
    ch = dlch_mod.Channels(prim, nof_prb, 0, model.pci)
    occupied = set()
    bad = 0
    stored_pdcch = 0
    stored_pdsch = 0
    refused = 0
    last_ok = None
    for item in info.get('dyn', []):
        if item[0] == 'NPDCCH':
            rnti, dci = int(item[1]), [int(c) for c in item[2]]
            k0, r, ok = int(item[3]), int(item[4]), int(item[5])
            plan = dlch_mod.model_plan_npdcch(valid, k0, r)
            want_ok = not any(t in occupied for t, _, _ in plan)
            if ok != want_ok:
                print('  FAIL %s: NPDCCH at %d (R=%d): table said %d, model expects %d' % (name, k0, r, ok, want_ok))
                bad += 1
            last_ok = None
            if ok:
                stored_pdcch += 1
                last_ok = (rnti, plan, dci)
                for t, reinit, pos in plan:
                    occupied.add(t)
                    _, grid = ch.npdcch_grid(dci, rnti, reinit, pos)
                    model.dyn[t] = (HAS_NPDCCH, grid)
            else:
                refused += 1
        else:
            rnti, tb = int(item[1]), bytes.fromhex(item[2])
            n_last, k0, n_sf, n_rep, ok = int(item[3]), int(item[4]), int(item[5]), int(item[6]), int(item[7])
            plan = dlch_mod.model_plan_npdsch(valid, n_last, k0, n_sf, n_rep)
            want_ok = not any(t in occupied for t, _, _, _ in plan)
            if ok != want_ok:
                print('  FAIL %s: NPDSCH after %d: table said %d, model expects %d' % (name, n_last, ok, want_ok))
                bad += 1
            tb_bits = [(b >> (7 - i)) & 1 for b in tb for i in range(8)]
            if ok:
                stored_pdsch += 1
                for t, cw, psfn, psf in plan:
                    occupied.add(t)
                    _, grid = ch.npdsch_grid(tb_bits, rnti, n_sf, cw, psfn, psf)
                    model.dyn[t] = (HAS_NPDSCH, grid)
            else:
                refused += 1
    return bad, stored_pdcch, stored_pdsch, refused


def run_case(dump, sync, sched_mod, name, nof_prb, anchor, pci, sched, hfn, nframes, junk, dyn=0):
    with tempfile.TemporaryDirectory() as td:
        binf = os.path.join(td, 'grid.bin')
        res = subprocess.run([dump, binf, str(nof_prb), str(anchor), str(pci), str(sched), str(hfn), str(nframes),
                              str(junk)] + (['1'] if dyn else []), capture_output=True, text=True)
        if res.returncode != 0:
            print('FAIL %s: dump exited %d: %s' % (name, res.returncode, res.stderr.strip()))
            return None
        info = parse_stdout(res.stdout)
        data = np.fromfile(binf, dtype=np.complex64).reshape(nframes * 10, 2, 14, 12)

    model = Model(pci, sched, hfn, sync, sched_mod, info['mib'], info['sib1'], info['si'])
    model.prepare_si(hfn * 1024, hfn * 1024 + nframes + 4)

    stats = {'re': 0, 'bad': 0, 'sf': 0, 'kinds': {HAS_NPSS: 0, HAS_NSSS: 0, HAS_NPBCH: 0, HAS_SIB1: 0, HAS_SI: 0,
                                                   HAS_NPDCCH: 0, HAS_NPDSCH: 0}}
    if dyn:
        import nbiot_dlch_check as dlch_mod
        prim = sys.modules['__main__'] if 'enb_dl_nbiot_check' not in sys.modules else sys.modules['enb_dl_nbiot_check']
        bad, n_pdcch, n_pdsch, n_ref = register_dynamic(model, info, dlch_mod, prim, nof_prb, name)
        stats['bad'] += bad
        if n_pdcch == 0 or n_pdsch == 0 or n_ref == 0:
            print('  FAIL %s: dyn run exercised %d NPDCCH, %d NPDSCH, %d refusals' % (name, n_pdcch, n_pdsch, n_ref))
            stats['bad'] += 1
        if info.get('dyn_conflicts') != 0:
            print('  FAIL %s: composer dropped %s planned transmissions' % (name, info.get('dyn_conflicts')))
            stats['bad'] += 1
    shown = 0
    crs_model = {(l, k) for (l, k) in crs_positions(pci) if l >= 3}   # after the LTE control region
    for sfn in range(nframes):
        for sf in range(10):
            pre = data[sfn * 10 + sf, 0]
            post = data[sfn * 10 + sf, 1]
            exp, flags = model.expected(sfn, sf, pre)
            stats['sf'] += 1
            for kind in stats['kinds']:
                if flags & kind:
                    stats['kinds'][kind] += 1

            # the CRS model must agree with what the real LTE downlink put in the grid (symbols after the control region)
            pre_nz = {(l, k) for l in range(3, 14) for k in range(12) if abs(pre[l, k]) > 1e-6}
            if pre_nz != crs_model:
                print('FAIL %s: CRS model differs from the LTE grid at %d.%d: %s' % (name, sfn, sf, sorted(pre_nz ^ crs_model)))
                stats['bad'] += 1

            diff = np.abs(post.astype(complex) - exp)
            stats['re'] += 168
            nbad = int(np.count_nonzero(diff > TOL))
            if nbad:
                stats['bad'] += nbad
                if shown < 6:
                    shown += 1
                    l, k = np.argwhere(diff > TOL)[0]
                    print('  MISMATCH %s at %d.%d (%d REs), first at l=%d k=%d: library %s, model %s' %
                          (name, sfn, sf, nbad, l, k, post[l, k], exp[l, k]))

            got_flags = info['flags'].get((sfn, sf), 0)
            if got_flags != flags:
                stats['bad'] += 1
                if shown < 6:
                    shown += 1
                    print('  MISMATCH %s at %d.%d: composer reports flags %d, model %d' % (name, sfn, sf, got_flags, flags))

    if info['outside'] != 0:
        stats['bad'] += 1
        print('  FAIL %s: %s REs outside the anchor PRB data region changed' % (name, info['outside']))
    expected_coll = nframes * 10 if junk else 0
    if info['coll'] != expected_coll:
        stats['bad'] += 1
        print('  FAIL %s: lte_collisions %s, expected %d' % (name, info['coll'], expected_coll))
    return stats


def main():
    dump, sync_dir, sched_dir = sys.argv[1], sys.argv[2], sys.argv[3]
    sys.path.insert(0, sync_dir)
    sys.path.insert(0, sched_dir)
    import nbiot_sync_check as sync
    import nbiot_sched_check as sched_mod

    cases = [
        # name, nof_prb, anchor, pci, sched_info, hfn, frames, junk
        ('25prb/a17/pci1/sched0/hfn0', 25, 17, 1, 0, 0, 320, 0),
        ('25prb/a17/pci1/sched0/hfn0/junk', 25, 17, 1, 0, 0, 320, 1),
        ('25prb/a22/pci7/sched5/hfn1', 25, 22, 7, 5, 1, 320, 0),
        ('50prb/a14/pci251/sched10/hfn3', 50, 14, 251, 10, 3, 320, 0),
        ('100prb/a44/pci503/sched3/hfn2', 100, 44, 503, 3, 2, 320, 0),
        ('25prb/a2/pci0/sched8/hfn0', 25, 2, 0, 8, 0, 320, 0),
        # NPDCCH / NPDSCH from the MAC's table; 480 frames = 4800 subframes take the table's ring around once
        ('25prb/a17/pci1/sched4/hfn0/dyn', 25, 17, 1, 4, 0, 480, 0, 1),
        ('25prb/a17/pci1/sched4/hfn0/dyn/junk', 25, 17, 1, 4, 0, 120, 1, 1),
        ('50prb/a14/pci251/sched10/hfn3/dyn', 50, 14, 251, 10, 3, 200, 0, 1),
        ('100prb/a44/pci503/sched3/hfn2/dyn', 100, 44, 503, 3, 2, 200, 0, 1),
    ]
    # Every v_shift, and ids on both sides of 5 and 6 (the old NPDSCH mapping special-cased ids <= 5): shorter runs
    for pci in (2, 3, 4, 5, 6, 11, 12, 13, 17, 100, 255, 256, 257, 502):
        cases.append(('25prb/a7/pci%d/sched%d' % (pci, pci % 12), 25, 7, pci, pci % 12, pci % 4, 96, 0))
    total_bad = 0
    total_re = 0
    kinds = {HAS_NPSS: 0, HAS_NSSS: 0, HAS_NPBCH: 0, HAS_SIB1: 0, HAS_SI: 0, HAS_NPDCCH: 0, HAS_NPDSCH: 0}
    for c in cases:
        st = run_case(dump, sync, sched_mod, *c)
        if st is None:
            return 1
        total_bad += st['bad']
        total_re += st['re']
        for k in kinds:
            kinds[k] += st['kinds'][k]
        print('%-34s %5d subframes, %7d REs, %d mismatches' % (c[0], st['sf'], st['re'], st['bad']))

    names = {HAS_NPSS: 'NPSS', HAS_NSSS: 'NSSS', HAS_NPBCH: 'NPBCH', HAS_SIB1: 'SIB1-NB', HAS_SI: 'SI message',
             HAS_NPDCCH: 'NPDCCH', HAS_NPDSCH: 'NPDSCH'}
    print('subframes exercised: ' + ', '.join('%s %d' % (names[k], kinds[k]) for k in kinds))
    for k, n in kinds.items():
        if n == 0:
            print('FAIL: no %s subframe was exercised' % names[k])
            return 1
    print('%d REs compared, %d mismatches' % (total_re, total_bad))
    return 0 if total_bad == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
