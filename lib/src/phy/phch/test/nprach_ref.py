#!/usr/bin/env python3
"""
Independent reference model for NB-IoT NPRACH (TS 36.211 v14.2.0 clause 10.1.6).

Written straight from the specification text, sharing no code with the C
implementation in lib/src/phy/phch/nprach.c, so that the two can be compared.

  hops(cell_id, n_init, n_groups)  -> subcarrier index (0..11) of every symbol group
  preamble(...)                    -> baseband samples of the whole preamble

Run directly to check the hop pattern against the table shipped with the third
party NPRACH_DETECTOR project (cell id 1, 12 starting subcarriers, 128 groups).
"""
import sys
import numpy as np

N_RA = 12        # N_sc^RA: hopping range
DF_RA = 3750.0   # Hz
TS = 1.0 / 30.72e6


def gold(c_init, length, nc=1600):
    """Pseudo random sequence of TS 36.211 clause 7.2."""
    n = nc + length + 31
    x1 = np.zeros(n, dtype=np.uint8)
    x2 = np.zeros(n, dtype=np.uint8)
    x1[0] = 1
    for i in range(31):
        x2[i] = (c_init >> i) & 1
    for i in range(n - 31):
        x1[i + 31] = (x1[i + 3] + x1[i]) & 1
        x2[i + 31] = (x2[i + 3] + x2[i + 2] + x2[i + 1] + x2[i]) & 1
    return (x1[nc:nc + length] + x2[nc:nc + length]) & 1


def hops(cell_id, n_init, n_groups):
    """n~sc^RA(i) for i in 0..n_groups-1 (clause 10.1.6.1)."""
    n_t = (n_groups + 3) // 4 + 1
    c = gold(cell_id, 10 * n_t + 10)

    # f(t), with f(-1) = 0
    f = {-1: 0}
    for t in range(0, n_t):
        s = 0
        for n in range(10 * t + 1, 10 * t + 10):
            s += int(c[n]) << (n - (10 * t + 1))
        f[t] = (f[t - 1] + (s % (N_RA - 1)) + 1) % N_RA

    n0 = n_init % N_RA
    out = [0] * n_groups
    for i in range(n_groups):
        if i == 0:
            out[i] = n0
        elif i % 4 == 0:
            out[i] = (n0 + f[i // 4]) % N_RA
        elif i % 4 in (1, 3):
            out[i] = out[i - 1] + 1 if out[i - 1] % 2 == 0 else out[i - 1] - 1
        else:  # i % 4 == 2
            out[i] = out[i - 1] + 6 if out[i - 1] < 6 else out[i - 1] - 6
    return out


def n_sc_ra(cell_id, n_init, n_groups, n_sc_offset=0):
    """n_sc^RA(i) = n_start + n~(i), n_start = N_scoffset + floor(n_init/12)*12."""
    n_start = n_sc_offset + (n_init // N_RA) * N_RA
    return [n_start + h for h in hops(cell_id, n_init, n_groups)]


def preamble(fs, fmt, n_rep, cell_id, n_init, n_sc_offset=0, n_sc_ul=12, amp=1.0):
    """
    Baseband preamble (clause 10.1.6.2):
        s_i(t) = amp * exp(j 2 pi (n_sc(i) + K k0 + 1/2) df (t - T_CP)),  0 <= t < T_SEQ + T_CP
    concatenated over all symbol groups, with the 40 ms gap after every 64
    repetitions (clause 10.1.6.1). Returns (samples, group_starts) where
    group_starts is the sample index of each symbol group's first sample
    (including its CP).
    """
    t_cp = (2048 if fmt == 0 else 8192) * TS
    t_seq = 5 * 8192 * TS
    k = 4.0                     # df / df_RA = 15 kHz / 3.75 kHz
    k0 = -n_sc_ul / 2.0
    n_groups = 4 * n_rep
    sc = n_sc_ra(cell_id, n_init, n_groups, n_sc_offset)

    glen = int(round((t_cp + t_seq) * fs))
    gap = int(round(40e-3 * fs))
    total = n_groups * glen + gap * ((n_rep - 1) // 64)
    x = np.zeros(total, dtype=np.complex128)
    starts = []
    pos = 0
    for i in range(n_groups):
        if i > 0 and i % (4 * 64) == 0:
            pos += gap
        t = np.arange(glen) / fs
        f = (sc[i] + k * k0 + 0.5) * DF_RA
        x[pos:pos + glen] = amp * np.exp(2j * np.pi * f * (t - t_cp))
        starts.append(pos)
        pos += glen
    return x, starts


def _load_reference_table(path):
    rows = [int(v) for v in open(path).read().replace("\n", "").split(",") if v.strip()]
    assert len(rows) == 12 * 128, len(rows)
    return np.array(rows).reshape(12, 128)


if __name__ == "__main__":
    ref_path = sys.argv[1] if len(sys.argv) > 1 else \
        "/home/illysky/src/reference/NPRACH_DETECTOR/NPRACH_C/freqHops.txt"
    ref = _load_reference_table(ref_path)
    bad = 0
    for n_init in range(12):
        mine = hops(1, n_init, 128)
        diff = [i for i in range(128) if mine[i] != ref[n_init][i]]
        print("n_init %2d: %s" % (n_init, "match" if not diff else
              "MISMATCH at groups %s" % diff[:8]))
        bad += bool(diff)
    sys.exit(1 if bad else 0)
