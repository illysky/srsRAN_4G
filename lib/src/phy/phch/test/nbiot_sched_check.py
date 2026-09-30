#!/usr/bin/env python3
"""
Checks the eNB's SIB1-NB and SI-message placement (nbiot_sched.c) against a model built from the specification text
by *forward* simulation: enumerate the repetitions, walk the subframes, and write down what lands where. The library
answers the opposite question ("what is in this subframe?") with a backward lookup, so the two share no structure.

Spec rules modelled
  TS 36.213 Tables 16.4.1.3-3/-4, TS 36.331 5.2.1.2a
    SIB1-NB: subframe 4 of every other frame in 16 continuous frames; the repetition count N (4, 8, 16) follows
    schedulingInfoSIB1 mod 3; the first frame (n_f mod 256) is {0,16,32,48}[N_ID mod 4] for N=4, {0,16}[N_ID mod 2]
    for N=8 and {0,1}[N_ID mod 2] for N=16; the N repetitions are equally spaced in a 256-frame period.
  TS 36.331 5.2.3a
    SI-window starts at subframe 0 of the frame with (H-SFN*1024+SFN) mod T = floor((n-1)*w/10) + offset; the message
    goes out in repetitions starting every si-RepetitionPattern frames within the window, each over 2 (TB 56/120) or
    8 consecutive NB-IoT downlink subframes (not NPBCH, NPSS, NSSS, SIB1-NB) beginning at subframe 0 of the frame.

usage: nbiot_sched_check.py ./nbiot_sched_dump
"""
import subprocess
import sys

WRAP = 1 << 20


def sib1_model(n_id, sched):
    n_rep = (4, 8, 16)[sched % 3]
    if n_rep == 4:
        first = (0, 16, 32, 48)[n_id % 4]
    elif n_rep == 8:
        first = (0, 16)[n_id % 2]
    else:
        first = (0, 1)[n_id % 2]
    out = {}
    for period in range(4):
        for i in range(n_rep):
            g = period * 256 + first + i * (256 // n_rep)
            for k in range(0, 16, 2):
                out[(g + k, 4)] = (g, k // 2, 8)
    return out


def valid_dl(sfn, sf):
    return not (sf == 0 or sf == 5 or (sf == 9 and sfn % 2 == 0))


def si_model(n_id, sched, cfg, lo, hi):
    n, t, off, rep, tb, w = cfg
    nof = 2 if tb in (56, 120) else 8
    sib1 = sib1_model(n_id, sched)
    base = ((n - 1) * w) // 10 + off
    out = {}
    # window starts: every frame counter congruent to base mod T; start a few windows early so a window that began
    # before `lo` and still runs into the block is included
    first_window = ((lo - 3 * t) // t) * t + base
    f = first_window
    while f < hi:
        if f + (w // 10 + 4) >= lo:
            i = 0
            while i * rep * 10 < w:
                start = f + i * rep
                cnt = 0
                fr = start
                while cnt < nof:
                    for sf in range(10):
                        if valid_dl(fr % 1024, sf) and (fr % 1024, sf) not in sib1:
                            out[(fr, sf)] = (start % 1024, cnt, nof)
                            cnt += 1
                            if cnt == nof:
                                break
                    fr += 1
                i += 1
        f += t
    return out


def run_dump(exe, n_id, sched, cfg):
    args = [exe, str(n_id), str(sched)] + [str(x) for x in cfg]
    res = subprocess.run(args, capture_output=True, text=True, check=True)
    s1, si = {}, {}
    for line in res.stdout.splitlines():
        v = line.split()
        if v[0] == 'S':
            s1[(int(v[1]), int(v[2]))] = tuple(int(x) for x in v[3:6])
        else:
            hfn, sfn, sf = int(v[1]), int(v[2]), int(v[3])
            si[(hfn * 1024 + sfn, sf)] = tuple(int(x) for x in v[4:7])
    return s1, si


def in_block(counter, lo, hi):
    return any(lo <= counter + m * WRAP < hi for m in (0, 1))


def compare(name, model, got, lo, hi):
    """Both directions, restricted to the frame-counter block [lo, hi); counters are compared modulo 2^20."""
    bad = 0
    m = {((f % WRAP), sf): v for (f, sf), v in model.items() if lo <= f < hi}
    g = {k: v for k, v in got.items() if in_block(k[0], lo, hi)}
    for key in sorted(set(m) | set(g)):
        if m.get(key) != g.get(key):
            bad += 1
            if bad <= 5:
                print('  MISMATCH %s at frame counter %d sf %d: model %s, library %s' % (name, key[0], key[1], m.get(key), g.get(key)))
    return bad, len(m)


def main():
    exe = sys.argv[1]
    errors = 0
    checks = 0
    tried = 0

    sib1_cfgs = [(n_id, sched) for n_id in (0, 1, 2, 3, 4, 7, 255, 256, 503) for sched in range(12)]
    for n_id, sched in sib1_cfgs:
        s1, _ = run_dump(exe, n_id, sched, (1, 256, 1, 4, 208, 160))
        model = sib1_model(n_id, sched)
        # SIB1 repeats every 1024 frames: compare on sfn
        bad, n = compare('SIB1 n_id=%d sched=%d' % (n_id, sched), model, s1, 0, 1024)
        errors += bad
        checks += n
        tried += 1
        if sched % 3 == 0:
            # spot property: a SIB1 repetition is 8 subframes and there are N of them per period
            per_period = [k for k in model if k[0] < 256]
            assert len(per_period) == 8 * (4, 8, 16)[sched % 3], 'model self-check'

    # SI messages: a spread of positions in the scheduling list, periodicities, patterns, TBs, windows, offsets, cells
    si_cfgs = []
    for (n, t, off, rep, tb, w) in [
        (1, 64, 1, 2, 56, 160), (1, 64, 1, 4, 208, 160), (1, 128, 1, 8, 120, 320), (1, 256, 5, 16, 328, 640),
        (2, 256, 2, 4, 440, 320), (3, 512, 3, 2, 680, 480), (1, 1024, 1, 4, 208, 160), (2, 1024, 15, 2, 56, 160),
        (1, 2048, 1, 8, 208, 1280), (1, 4096, 1, 16, 680, 960), (4, 4096, 9, 4, 552, 1600), (2, 64, 7, 2, 256, 160),
    ]:
        si_cfgs.append((n, t, off, rep, tb, w))

    for n_id, sched in ((0, 0), (1, 0), (1, 2), (2, 4), (3, 9), (257, 5), (503, 11)):
        for cfg in si_cfgs:
            _, si = run_dump(exe, n_id, sched, cfg)
            for lo, hi in ((0, 4096), (WRAP - 2048, WRAP + 2048)):
                model = si_model(n_id, sched, cfg, lo, hi)
                bad, n = compare('SI n_id=%d sched=%d cfg=%s' % (n_id, sched, cfg), model, si, lo, hi)
                errors += bad
                checks += n
                tried += 1

    print('%d configurations, %d scheduled subframes compared, %d mismatches' % (tried, checks, errors))
    # the comparison must have had something to compare
    if checks < 10000:
        print('FAIL: suspiciously few scheduled subframes (%d)' % checks)
        return 1
    return 0 if errors == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
