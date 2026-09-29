#!/usr/bin/env python3
"""
Independent reference transmitter for NB-IoT NPUSCH format 1 (TS 36.211 v14.2.0 clause 10.1.3 / 10.1.4 / 10.1.5,
TS 36.212 v14.2.0 clauses 5.1, 5.2.2 and 6.3.2).

Written straight from the specification text and sharing no code with the C implementation, so that the C receiver
can be tested on waveforms it did not produce. The tables are extracted from the spec text by a script
(npusch_tables.py).

  transmit(tb_bits, cfg) -> dict with the baseband samples at 1.92 MS/s and the intermediate bit sequences

cfg keys
  n_sc          1, 3, 6 or 12 subcarriers
  spacing       3750 or 15000 Hz (n_sc > 1 is 15 kHz only)
  sc            first subcarrier k(-): 0..47 (3.75 kHz) or 0..11 (15 kHz)
  n_ru, n_rep   resource units and repetitions
  rnti, cell_id, frame (n_f), slot (n_s of the first slot)
  rv            0 or 2
  qm            1 (pi/2 BPSK) or 2 (pi/4 QPSK), single tone only; multi tone is QPSK
  group_hopping bool, delta_ss
  base_seq, cyclic_shift   multi tone DMRS (None: derived from the cell id, cyclic shift 0)

Not modelled: the 256 ms / 40 ms gap and postponement around NPRACH resources.
"""
import math
import sys
import os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nprach_ref import gold                                   # TS 36.211 clause 7.2 sequence
from npusch_tables import QPP_PARAMS, PHI_12, PHI_3, PHI_6, W16

FS = 1.92e6
TS_PER_SAMPLE = 16          # one 1.92 MS/s sample is 16 Ts
NULL = -1

# ---------------------------------------------------------------- coding, TS 36.212

def crc24a(bits):
    """gCRC24A(D) = D24+D23+D18+D17+D14+D11+D10+D7+D6+D5+D4+D3+D+1 (5.1.1); returns the 24 parity bits."""
    reg = 0
    for b in bits:
        fb = ((reg >> 23) & 1) ^ int(b)
        reg = (reg << 1) & 0xFFFFFF
        if fb:
            reg ^= 0x864CFB
    return [(reg >> (23 - i)) & 1 for i in range(24)]


def _rsc(bits):
    """One 8 state constituent encoder, g0 = 1 + D^2 + D^3 (feedback), g1 = 1 + D + D^3. Returns parity list and the
    three tail (systematic, parity) pairs that terminate the trellis (5.1.3.2.2)."""
    s = [0, 0, 0]                    # a(k-1), a(k-2), a(k-3)
    par = []
    for c in bits:
        a = c ^ s[1] ^ s[2]
        par.append(a ^ s[0] ^ s[2])
        s = [a, s[0], s[1]]
    tail = []
    for _ in range(3):
        x = s[1] ^ s[2]              # input that makes a = 0
        tail.append((x, s[0] ^ s[2]))
        s = [0, s[0], s[1]]
    assert s == [0, 0, 0]
    return par, tail


def turbo_encode(c):
    """Rate 1/3 turbo encoder (5.1.3.2). Returns d0, d1, d2 of length K + 4."""
    K = len(c)
    f1, f2 = QPP_PARAMS[K]
    ci = [c[(f1 * i + f2 * i * i) % K] for i in range(K)]
    z, t1 = _rsc(c)
    zp, t2 = _rsc(ci)
    (x0, z0), (x1, z1), (x2, z2) = t1            # x_K, z_K ; x_K+1, z_K+1 ; x_K+2, z_K+2
    (xp0, zp0), (xp1, zp1), (xp2, zp2) = t2      # x'_K ...
    d0 = list(c) + [x0, z1, xp0, zp1]
    d1 = z + [z0, x2, zp0, xp2]
    d2 = zp + [x1, z2, xp1, zp2]
    return d0, d1, d2


_P = [0, 16, 8, 24, 4, 20, 12, 28, 2, 18, 10, 26, 6, 22, 14, 30, 1, 17, 9, 25, 5, 21, 13, 29, 3, 19, 11, 27, 7, 23, 15, 31]


def rate_match(d0, d1, d2, e_len, rv):
    """5.1.4.1: sub-block interleaving, circular buffer, bit selection. Returns e (length e_len)."""
    D = len(d0)
    R = -(-D // 32)
    KP = 32 * R
    nd = KP - D

    def block(d):
        y = [NULL] * nd + list(d)
        mat = [y[r * 32:(r + 1) * 32] for r in range(R)]
        return [mat[r][_P[c]] for c in range(32) for r in range(R)]

    v0 = block(d0)
    v1 = block(d1)
    y2 = [NULL] * nd + list(d2)
    v2 = [y2[(_P[k // R] + 32 * (k % R) + 1) % KP] for k in range(KP)]
    w = v0 + [0] * (2 * KP)
    for k in range(KP):
        w[KP + 2 * k] = v1[k]
        w[KP + 2 * k + 1] = v2[k]
    ncb = 3 * KP
    k0 = R * (2 * (-(-ncb // (8 * R))) * rv + 2)
    e = []
    j = 0
    while len(e) < e_len:
        b = w[(k0 + j) % ncb]
        if b != NULL:
            e.append(b)
        j += 1
    return e


def channel_interleave(bits, qm, c_mux):
    """5.2.2.8 without control information, applied to the bits of one resource unit: write the qm-bit vectors row by
    row into a matrix of c_mux columns, read it out column by column."""
    n = len(bits) // qm
    assert n % c_mux == 0
    rows = n // c_mux
    g = [bits[i * qm:(i + 1) * qm] for i in range(n)]
    out = []
    for c in range(c_mux):
        for r in range(rows):
            out.extend(g[r * c_mux + c])
    return out


# ---------------------------------------------------------------- resource unit structure, TS 36.211 10.1.2.3

def ru_geometry(n_sc, spacing):
    """(slots per RU, data symbols per RU)"""
    if spacing == 3750:
        assert n_sc == 1
        n_slots = 16
    else:
        n_slots = {1: 16, 3: 8, 6: 4, 12: 2}[n_sc]
    return n_slots, 6 * n_slots * n_sc


# ---------------------------------------------------------------- reference signal, TS 36.211 10.1.4

def dmrs_single_tone(cell_id, n_slots_total, group_hopping=False, delta_ss=0, slots_per_ru=16, first_slot_abs=0,
                     slots_per_frame=20):
    """r_u(n), one value per slot (10.1.4.1.1)."""
    c = gold(35, 2 * n_slots_total + 16)
    n_seq = 16
    r = []
    f_ss = (cell_id + delta_ss) % n_seq
    for n in range(n_slots_total):
        if group_hopping:
            # f_gh uses n_s' = the first slot of the resource unit, the generator restarts there
            ru = n // slots_per_ru
            ns_first = first_slot_abs + ru * slots_per_ru
            cc = gold(cell_id // n_seq, 8 * (ns_first + 1) + 8)
            f_gh = sum(int(cc[8 * ns_first + i]) << i for i in range(8)) % n_seq
            u = (f_gh + f_ss) % n_seq
        else:
            u = cell_id % 16
        w = W16[u][n % 16]
        r.append((1 + 1j) / math.sqrt(2) * (1 - 2 * int(c[n])) * w)
    return np.array(r)


def dmrs_multi_tone(n_sc, cell_id, base_seq=None, cyclic_shift=0):
    """r_u(n), n = 0..n_sc-1 (10.1.4.1.2), no group hopping."""
    table, mod = {3: (PHI_3, 12), 6: (PHI_6, 14), 12: (PHI_12, 30)}[n_sc]
    u = cell_id % mod if base_seq is None else base_seq
    alpha = {3: [0, 2 * math.pi / 3, 4 * math.pi / 3], 6: [0, 2 * math.pi / 6, 4 * math.pi / 6, 8 * math.pi / 6],
             12: [0]}[n_sc][cyclic_shift]
    return np.array([np.exp(1j * alpha * n) * np.exp(1j * table[u][n] * math.pi / 4) for n in range(n_sc)])


# ---------------------------------------------------------------- transmit chain

def _modulate(bits, qm):
    out = []
    if qm == 1:
        for b in bits:
            out.append((1 + 1j) / math.sqrt(2) if b == 0 else (-1 - 1j) / math.sqrt(2))
    else:
        for i in range(0, len(bits), 2):
            re = 1 if bits[i] == 0 else -1
            im = 1 if bits[i + 1] == 0 else -1
            out.append((re + 1j * im) / math.sqrt(2))
    return np.array(out)


def transmit(tb, cfg):
    n_sc, spacing = cfg["n_sc"], cfg["spacing"]
    n_ru, n_rep = cfg["n_ru"], cfg["n_rep"]
    n_slots_ru, data_syms_ru = ru_geometry(n_sc, spacing)
    qm = cfg.get("qm", 2) if n_sc == 1 else 2
    rv = cfg.get("rv", 0)
    cell = cfg["cell_id"]
    slots_per_frame = 5 if spacing == 3750 else 20
    n_slots_unit = 1 if spacing == 3750 else 2                       # N_slots of 10.1.3.6
    m_ident = 1 if n_sc == 1 else min(-(-n_rep // 2), 4)
    assert n_rep % m_ident == 0
    n_pass = n_rep // m_ident

    # ---- transport block coding
    a = list(int(b) for b in tb)
    c = a + crc24a(a)
    d0, d1, d2 = turbo_encode(c)
    g_bits = n_ru * data_syms_ru * qm
    e = rate_match(d0, d1, d2, g_bits, rv)
    # ---- per resource unit channel interleaver
    c_mux = 6 * n_slots_ru
    per_ru = data_syms_ru * qm
    f = []
    for i in range(n_ru):
        f.extend(channel_interleave(e[i * per_ru:(i + 1) * per_ru], qm, c_mux))

    # ---- symbols of one pass, in the order they are mapped: slot by slot, symbol by symbol, tone by tone
    total_slots = n_rep * n_ru * n_slots_ru
    first_abs = cfg["frame"] * slots_per_frame + cfg["slot"]
    sym_per_pass = len(f) // qm
    if cfg.get("group_hopping") and n_sc > 1:
        raise NotImplementedError("group hopping for multi tone is not modelled")
    if n_sc == 1:
        rs1 = dmrs_single_tone(cell, total_slots, cfg.get("group_hopping", False), cfg.get("delta_ss", 0),
                               n_slots_ru, first_abs, slots_per_frame)
    else:
        rs = dmrs_multi_tone(n_sc, cell, cfg.get("base_seq"), cfg.get("cyclic_shift", 0))

    dmrs_l = 4 if spacing == 3750 else 3
    slot_out = []          # per transmitted slot: list of per-symbol tone vectors (n_sc values) before precoding info
    # build the pass sequence of slots: RU slots grouped in blocks of n_slots_unit, each block sent m_ident times
    pass_slots = []        # entries: (index of the slot inside the codeword, for data mapping)
    for blk in range(n_ru * n_slots_ru // n_slots_unit):
        for _ in range(m_ident):
            for s in range(n_slots_unit):
                pass_slots.append(blk * n_slots_unit + s)

    slots = []             # each slot: list of 7 arrays (n_sc complex values)
    slot_abs_counter = 0
    for p in range(n_pass):
        abs0 = first_abs + p * len(pass_slots)
        nf, ns = (abs0 // slots_per_frame) % 1024, abs0 % slots_per_frame
        cinit = cfg["rnti"] * 2 ** 14 + (nf % 2) * 2 ** 13 + (ns // 2) * 2 ** 9 + cell
        cs = gold(cinit, len(f))
        b = [int(f[i]) ^ int(cs[i]) for i in range(len(f))]
        z = _modulate(b, qm)                         # codeword modulation symbols, one per tone position
        for slot_in_cw in pass_slots:
            syms = []
            for l in range(7):
                if l == dmrs_l:
                    if n_sc == 1:
                        syms.append(np.array([rs1[slot_abs_counter]]))
                    else:
                        syms.append(rs.copy())
                else:
                    ld = l if l < dmrs_l else l - 1        # data symbol index within the slot
                    base = (slot_in_cw * 6 + ld) * n_sc
                    v = z[base:base + n_sc]
                    if n_sc > 1:                            # transform precoding, 5.3.3
                        v = np.fft.fft(v) / math.sqrt(n_sc)
                    syms.append(v)
            slots.append(syms)
            slot_abs_counter += 1
    assert slot_abs_counter == total_slots

    # ---- SC-FDMA waveform
    if spacing == 3750:
        n_fft_ts, cps = 8192, [256] * 7
        slot_samples = 61440 // TS_PER_SAMPLE
        k_of = lambda sc: (cfg["sc"] - 24)
    else:
        n_fft_ts, cps = 2048, [160] + [144] * 6
        slot_samples = 15360 // TS_PER_SAMPLE
        k_of = lambda sc: sc
    out = np.zeros(total_slots * slot_samples, dtype=np.complex128)
    rho = math.pi / 2 if qm == 1 else math.pi / 4
    phi_hat = 0.0
    lt = 0                                          # symbol counter l~ (single tone)
    df = float(spacing)
    for si, syms in enumerate(slots):
        pos = si * slot_samples
        for l in range(7):
            ncp = cps[l]
            n_samp = (ncp + n_fft_ts) // TS_PER_SAMPLE
            t = (np.arange(n_samp) - ncp // TS_PER_SAMPLE) / FS
            if n_sc == 1:
                k = cfg["sc"] - (24 if spacing == 3750 else 6)
                if lt > 0:
                    phi_hat += 2 * math.pi * df * (k + 0.5) * (n_fft_ts + ncp) / 30.72e6
                phi = rho * (lt % 2) + phi_hat
                sym = syms[l][0] * np.exp(1j * phi) * np.exp(1j * 2 * math.pi * (k + 0.5) * df * t)
                lt += 1
            else:
                sym = np.zeros(n_samp, dtype=np.complex128)
                for i in range(n_sc):
                    k = cfg["sc"] + i - 6
                    sym += syms[l][i] * np.exp(1j * 2 * math.pi * (k + 0.5) * df * t)
            out[pos:pos + n_samp] = sym
            pos += n_samp
    return {"samples": out, "slot_samples": slot_samples, "n_slots": total_slots, "crc_tb": c, "d": (d0, d1, d2),
            "e": e, "f": f, "qm": qm, "n_pass": n_pass, "m_ident": m_ident}


if __name__ == "__main__":
    # quick self check of the pieces that have known answers
    assert crc24a([0] * 40) == [0] * 24
    d0, d1, d2 = turbo_encode([0] * 40)
    assert not any(d0) and not any(d1) and not any(d2)
    print("npusch_ref self check ok")
