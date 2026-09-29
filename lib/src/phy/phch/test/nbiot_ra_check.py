#!/usr/bin/env python3
"""
Independent check of the NB-IoT random access module (nbiot_ra.c).

The model below is written from the specification text, not from the C code: the RAR MAC PDU is built as a string of
bits by concatenating fields in the order of TS 36.321 Figure 6.1.5-3b (measured from the figure: R | TA 11 | grant 15 |
R x5 | TC-RNTI 16), the Msg3 subcarrier sets are enumerated from the set expressions of TS 36.213 Table 16.5.1.1-1
(3(Isc-12)+{0,1,2}, 6(Isc-16)+{0..5}), and Table 16.3.3-1 is a literal dictionary. The C code packs with shifts and
masks and looks the tables up differently, so a slip in either one shows up as a difference.

usage: nbiot_ra_check.py <nbiot_ra_dump> [<npusch_rx_dump>]

With the NPUSCH receiver as a second argument, every kind of Msg3 grant is also carried through the independent Python
NPUSCH transmitter and decoded, which shows that the grant is translated into a transmission the receiver can decode.
"""
import math
import random
import subprocess
import sys

K0 = [12, 16, 32, 64]                      # Table 16.5.1-1 with k0 = 12 for I_delay = 0 (16.3.3)
NREP = [1, 2, 4, 8, 16, 32, 64, 128]       # Table 16.5.1.1-3
MSG3_MCS = {0: (4, 'pi/2 BPSK'), 1: (3, 'pi/4 QPSK'), 2: (1, 'pi/4 QPSK')}   # Table 16.3.3-1: I_MCS -> (N_RU, mod)


def b(v, n):
    assert 0 <= v < (1 << n)
    return format(v, '0%db' % n)


def allocated_set(sc15, isc):
    """Table 16.5.1.1-1 (and 'n_sc = I_sc' for 3.75 kHz). None for reserved indications."""
    if not sc15:
        return [isc] if isc <= 47 else None
    if isc <= 11:
        return [isc]
    if 12 <= isc <= 15:
        return [3 * (isc - 12) + d for d in (0, 1, 2)]
    if 16 <= isc <= 17:
        return [6 * (isc - 16) + d for d in (0, 1, 2, 3, 4, 5)]
    if isc == 18:
        return list(range(12))
    return None


def grant_valid(g):
    sc15, isc, delay, rep, mcs = g
    return sc15 in (0, 1) and allocated_set(sc15, isc) is not None and 0 <= delay <= 3 and 0 <= rep <= 7 and mcs in MSG3_MCS


def grant_bits(g):
    sc15, isc, delay, rep, mcs = g
    return b(sc15, 1) + b(isc, 6) + b(delay, 2) + b(rep, 3) + b(mcs, 3)


def expected_npusch(g):
    sc15, isc, delay, rep, mcs = g
    s = allocated_set(sc15, isc)
    n_ru, _ = MSG3_MCS[mcs]
    qm = 1 if (len(s) == 1 and mcs == 0) else 2
    return [len(s), 15000 if sc15 else 3750, min(s), n_ru, NREP[rep], qm, 88, K0[delay]]


def pdu_bits(bi, rars):
    """rars: list of (rapid, ta, grant, tc). Returns hex string or None if the model says the request is invalid."""
    if bi is not None and not (0 <= bi <= 15):
        return None
    if bi is None and not rars:
        return None
    if len(rars) > 8:
        return None
    subs = []
    if bi is not None:
        subs.append(('bi', bi))
    for r in rars:
        rapid, ta, g, tc = r
        if rapid > 47 or ta > 1282 or tc > 0xffff or not grant_valid(g):
            return None
        subs.append(('rapid', rapid))
    out = ''
    for i, (kind, v) in enumerate(subs):
        e = '1' if i + 1 < len(subs) else '0'
        out += e + ('0' + '00' + b(v, 4) if kind == 'bi' else '1' + b(v, 6))
    for rapid, ta, g, tc in rars:
        out += '0' + b(ta, 11) + grant_bits(g) + '00000' + b(tc, 16)
    assert len(out) % 8 == 0
    return '%0*x' % (len(out) // 4, int(out, 2))


class Dump:
    def __init__(self, exe):
        self.p = subprocess.Popen([exe], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)

    def ask(self, line):
        self.p.stdin.write(line + '\n')
        self.p.stdin.flush()
        return self.p.stdout.readline().strip()

    def close(self):
        self.p.stdin.close()
        self.p.wait()


fails = 0
checks = 0


def check(cond, msg):
    global fails, checks
    checks += 1
    if not cond:
        fails += 1
        if fails <= 30:
            print('FAIL', msg)


def msg3_end_to_end(d, rx_exe):
    """grant -> NPUSCH parameters (C) -> waveform (independent Python transmitter) -> decoded transport block (C)"""
    import os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import numpy as np
    import npusch_rx_check as C

    rng = np.random.default_rng(3)
    cases = []
    for mcs in (0, 1, 2):
        for sc15, isc in ((0, 0), (0, 47), (1, 0), (1, 11), (1, 12), (1, 15), (1, 16), (1, 17), (1, 18)):
            cases.append(((sc15, isc, 0, 0, mcs), 14.0))
    for mcs in (0, 1, 2):                                # repetitions, at a lower SNR
        cases.append(((1, 5, 1, 2, mcs), 3.0))
        cases.append(((1, 17, 2, 3, mcs), -2.0))
    n = 0
    for g, snr in cases:
        got = d.ask('G %d %d %d %d %d' % g)
        n_sc, spacing, sc, n_ru, n_rep, qm, tbs, k0 = (int(x) for x in got.split())
        cfg = C.base_cfg(n_sc=n_sc, spacing=spacing, sc=sc, n_ru=n_ru, n_rep=n_rep, qm=qm, rnti=0x4321,
                         cell_id=(sc * 13 + 5 + g[4]) % 504)
        # a few trials with different noise: a single unlucky one must not fail a healthy configuration
        ok = 0
        for k in range(3):
            r = C.one(rx_exe, cfg, tbs, snr, float(rng.uniform(-100, 100)), float(rng.uniform(-3, 3)), 0, rng)
            ok += 1 if r["ok"] else 0
        check(ok >= 2, 'Msg3 grant %s (%s): decoded %d of 3' % (g, got, ok))
        n += 1
    print('  Msg3: %d grants carried through the NPUSCH transmitter and receiver' % n)


def main():
    d = Dump(sys.argv[1])
    rng = random.Random(20260929)

    # ---- RA-RNTI
    for sfn in range(0, 1024):
        for car in (0, 1):
            got = int(d.ask('R %d %d' % (sfn, car)))
            check(got == 1 + sfn // 4 + 256 * car, 'RA-RNTI sfn %d carrier %d: %d' % (sfn, car, got))

    # ---- timing advance
    for toa in [-5.0, -0.4, 0.0, 0.25, 0.49, 0.5, 1.5, 2.5, 7.0, 100.49, 100.5, 1281.4, 1281.5, 1282.0, 1282.4, 1283.0, 5000.0]:
        exp = 0 if toa <= 0 else min(1282, int(math.floor(toa + 0.5)))
        got = int(d.ask('T %r' % toa))
        check(got == exp, 'TA for ToA %r: %d, expected %d' % (toa, got, exp))
    check(int(d.ask('T nan')) == 0, 'TA for NaN')

    # ---- Msg3 grant -> NPUSCH parameters, every 15-bit combination of the six-bit fields that the model calls valid,
    #      and every reserved one for rejection
    n_valid = 0
    for sc15 in (0, 1):
        for isc in range(64):
            for delay in range(4):
                for rep in range(8):
                    for mcs in range(8):
                        g = (sc15, isc, delay, rep, mcs)
                        got = d.ask('G %d %d %d %d %d' % g)
                        if grant_valid(g):
                            n_valid += 1
                            check(got == ' '.join(str(x) for x in expected_npusch(g)), 'grant %s -> %s, expected %s' % (g, got, expected_npusch(g)))
                        else:
                            check(got == 'ERR', 'reserved grant %s accepted: %s' % (g, got))
    print('  grants: %d valid combinations checked' % n_valid)

    # ---- RAR PDU packing against the bit-string model
    def rand_grant(valid=True):
        while True:
            g = (rng.randint(0, 1), rng.randint(0, 63), rng.randint(0, 3), rng.randint(0, 7), rng.randint(0, 7))
            if grant_valid(g) == valid:
                return g

    def rand_rar():
        return (rng.randint(0, 47), rng.choice([0, 1, 1282, rng.randint(0, 1282)]), rand_grant(), rng.choice([0x0001, 0xffff, rng.randint(0, 0xffff)]))

    def ask_pack(bi, rars):
        line = 'P %d %d' % (-1 if bi is None else bi, len(rars))
        for rapid, ta, g, tc in rars:
            line += ' %d %d %d %d %d %d %d %d' % ((rapid, ta) + g + (tc,))
        return line

    n_pack = 0
    for _ in range(3000):
        bi = rng.choice([None, None, rng.randint(0, 15)])
        n = rng.randint(1 if bi is None else 0, 4)
        rars = [rand_rar() for _ in range(n)]
        exp = pdu_bits(bi, rars)
        got = d.ask(ask_pack(bi, rars))
        check(exp is not None and got == exp, 'pack bi=%s rars=%s -> %s, model %s' % (bi, rars, got, exp))
        n_pack += 1

        # unpack of the model's PDU, with random padding and random values in the reserved bits
        pdu = bytearray.fromhex(exp)
        body = bytearray(pdu)
        n_sub = n + (1 if bi is not None else 0)
        if bi is not None:
            body[0] |= rng.randint(0, 3) << 4                 # R R of the BI subheader
        for i in range(n):
            o = n_sub + 6 * i
            body[o] |= rng.randint(0, 1) << 7                 # R before TA
            body[o + 3] |= rng.randint(0, 31)                 # R x5 after the grant
        body += bytes(rng.randint(0, 255) for _ in range(rng.choice([0, 0, 1, 3])))   # padding
        gotu = d.ask('U ' + body.hex())
        want = '%d %d' % (-1 if bi is None else bi, n)
        for rapid, ta, g, tc in rars:
            want += ' %d %d %s %d' % (rapid, ta, ' '.join(str(x) for x in g), tc)
        check(gotu == want, 'unpack %s -> %s, expected %s' % (body.hex(), gotu, want))

        # every truncation below the full length is malformed
        for cut in range(0, len(pdu)):
            r = d.ask('U ' + pdu[:cut].hex()) if cut > 0 else 'ERR'
            check(r == 'ERR', 'truncated PDU (%d of %d octets) accepted: %s' % (cut, len(pdu), r))
    print('  RAR PDUs: %d built and parsed' % n_pack)

    # ---- requests the model calls invalid must be refused
    bad = [
        (None, []),
        (16, [rand_rar()]),
        (None, [(48, 0, rand_grant(), 1)]),
        (None, [(0, 1283, rand_grant(), 1)]),
        (None, [(0, 0, rand_grant(False), 1)]),
        (None, [(0, 0, rand_grant(), 0x10000)]),
        (None, [rand_rar() for _ in range(9)]),
    ]
    for bi, rars in bad:
        got = d.ask(ask_pack(bi, rars))
        check(got == 'ERR', 'invalid request accepted: bi=%s rars=%s -> %s' % (bi, rars, got))

    # ---- a Backoff Indicator anywhere but first is malformed
    r1 = rand_rar()
    good = pdu_bits(None, [r1, r1])
    hdr_bad = bytearray.fromhex(good)
    hdr_bad[1] = 0x00 | 0x05                  # second subheader turned into a BI (E = 0, T = 0)
    check(d.ask('U ' + hdr_bad.hex()) == 'ERR', 'BI as second subheader accepted')

    if len(sys.argv) > 2:
        msg3_end_to_end(d, sys.argv[2])

    d.close()
    print('%d checks, %d failures' % (checks, fails))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
