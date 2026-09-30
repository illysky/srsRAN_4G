#!/usr/bin/env python3
"""
What an NB-IoT UE does to get its random access response, written from the specification and sharing no code with the
eNB: it is the receiving end for the checks of the eNB's Msg2.

Given the downlink resource grid of the anchor PRB of every subframe (as a receiver would demodulate it), the broadcast
configuration, and the preamble it sent, it

  * works out the random access response window (TS 36.321 5.1.4: the subframe containing the end of the last preamble
    repetition plus 4 subframes, or plus 41 if there were 64 or more repetitions; ra-ResponseWindowSize periods of the
    common search space) and the RA-RNTI,
  * looks at every start of the Type-2 common search space in the window (TS 36.213 16.6: (10 n_f + floor(n_s / 2)) mod
    T = floor(alpha T), T = Rmax G), collects the NPDCCH candidate of R = Rmax (every subframe of it carries the same
    coded bits, scrambled from the start of each group of four, TS 36.211 10.2.5.2), and decodes it: soft demapping,
    descrambling, combining, rate recovery, exhaustive tail-biting Viterbi decoding, CRC-16 with the RNTI mask,
  * reads DCI format N1 (TS 36.212 6.4.3.2), finds the NPDSCH n + 5 valid subframes later (TS 36.213 16.4.1),
    combines its repetitions (TS 36.211 10.2.3.4), decodes it (CRC-24A) and parses the MAC PDU (TS 36.321 6.1.5).

Everything is decoded from the received values: channel estimate from the NRS, no knowledge of what was sent.
"""
import math
from fractions import Fraction

import numpy as np

R2 = 1.0 / math.sqrt(2.0)

N_SF = [1, 2, 3, 4, 5, 6, 8, 10]
N_REP = [1, 2, 4, 8, 16, 32, 64, 128, 192, 256, 384, 512, 768, 1024, 1536, 2048]
K0_LO = [0, 4, 8, 12, 16, 32, 64, 128]      # Table 16.4.1-1, Rmax < 128
K0_HI = [0, 16, 32, 64, 128, 256, 512, 1024]
DCI_REP_RMAX = {1: [1], 2: [1, 2], 4: [1, 2, 4], 8: [1, 2, 4, 8]}   # Table 16.6-1/-3 for Rmax up to 8 (simplified)


# ---------------------------------------------------------------------------------------------------------- NPRACH
def preamble_end_sf(start, n_rep, fmt):
    """Subframe containing the last sample of a preamble that started with subframe 'start' (TS 36.211 10.1.6)."""
    t_cp = Fraction(2048 if fmt == 0 else 8192, 30720)          # ms
    t_seq = Fraction(5 * 8192, 30720)
    groups = 4 * n_rep
    gaps = (n_rep - 1) // 64                                    # 40 ms after every 64 repetitions, if more follow
    total = groups * (t_cp + t_seq) + 40 * gaps                 # ms
    return start + math.ceil(total) - 1


def window_first_sf(start, n_rep, fmt):
    return preamble_end_sf(start, n_rep, fmt) + (41 if n_rep >= 64 else 4)


def ra_rnti(start):
    return 1 + ((start // 10) % 1024) // 4


# -------------------------------------------------------------------------------------------------- search space
def ss_period(r_max, g_halves):
    return (r_max * g_halves) // 2


def ss_starts(r_max, g_halves, offset_eighths, lo, hi):
    T = ss_period(r_max, g_halves)
    want = (offset_eighths * T) // 8
    return [t for t in range(lo, hi + 1) if (t % 10240) % T == want]


# ------------------------------------------------------------------------------------------------ decoding blocks
G_CONV = (0o133, 0o171, 0o165)


def _trellis():
    out = np.zeros((64, 2, 3), dtype=np.int8)
    for s in range(64):
        for b in range(2):
            win = [b] + [(s >> j) & 1 for j in range(6)]          # c[k], c[k-1] ... c[k-6]
            for i, g in enumerate(G_CONV):
                acc = 0
                for t in range(7):
                    if (g >> (6 - t)) & 1:
                        acc ^= win[t]
                out[s, b, i] = acc
    return out


_TRELLIS = _trellis()


def tbcc_decode(llr):
    """Exhaustive tail-biting decoding of the rate 1/3 code of TS 36.212 5.1.3.1: llr is (3, K), positive = bit 0.
    Tries all 64 start states, each forced to come back to itself; returns (bits, metric)."""
    K = llr.shape[1]
    # branch metric per step, state and input bit
    sign = 1 - 2 * _TRELLIS.astype(np.float64)                   # (64, 2, 3)
    ns = np.arange(64)
    pred0, pred1 = ns >> 1, (ns >> 1) | 32
    b_in = ns & 1
    metric = np.full((64, 64), -1e30)
    metric[np.arange(64), np.arange(64)] = 0.0                   # start state s0 is state s0
    choice = np.zeros((K, 64, 64), dtype=np.uint8)
    for k in range(K):
        bm = sign @ llr[:, k]                                    # (64, 2)
        c0 = metric[:, pred0] + bm[pred0, b_in]
        c1 = metric[:, pred1] + bm[pred1, b_in]
        pick1 = c1 > c0
        choice[k] = pick1
        metric = np.where(pick1, c1, c0)
    final = np.array([metric[s, s] for s in range(64)])
    s0 = int(np.argmax(final))
    bits = [0] * K
    state = s0
    for k in range(K - 1, -1, -1):
        bits[k] = state & 1
        state = int(pred1[state] if choice[k, s0, state] else pred0[state])
    return bits, float(final[s0])


def crc(bits, poly, width):
    reg, top, mask = 0, 1 << (width - 1), (1 << width) - 1
    for b in bits:
        fb = (1 if reg & top else 0) ^ b
        reg = (reg << 1) & mask
        if fb:
            reg ^= poly
    return [(reg >> (width - 1 - i)) & 1 for i in range(width)]


def rate_recovery_index(chk, k_len, e_len):
    """For each of the e_len transmitted bits, which of the 3 * k_len coded bits it is (the inverse of 5.1.4.2)."""
    streams = [[s * k_len + i for i in range(k_len)] for s in range(3)]
    return np.array(chk.rate_match(streams, e_len), dtype=np.int64)


def recover(chk, llr_e, k_len):
    idx = rate_recovery_index(chk, k_len, len(llr_e))
    acc = np.zeros(3 * k_len)
    np.add.at(acc, idx, llr_e)
    return acc.reshape(3, k_len)


# -------------------------------------------------------------------------------------------------- the receiver
class Receiver:
    def __init__(self, chk, pci, anchor_positions, valid, plan_npdsch):
        """chk: module with gold(), nrs_positions(), nrs_values(); anchor_positions: [(l, k)] of the REs NPDCCH /
        NPDSCH use, in mapping order; valid(t): NB-IoT downlink subframe; plan_npdsch(valid, n_last, k0, n_sf,
        n_rep): list of (t, part of codeword, frame and subframe number that started the pass)."""
        self.chk, self.pci, self.pos, self.valid = chk, pci, anchor_positions, valid
        self.plan_npdsch = plan_npdsch
        self.nrs_pos = chk.nrs_positions(pci, 1)
        self.e_sf = 2 * len(anchor_positions)

    def channel(self, sf, grid):
        """Complex gain from the NRS of a subframe (grid: 14 x 12)."""
        vals = self.chk.nrs_values(self.pci, sf)
        num = den = 0
        for (p, slot, ls, m), (l, k) in self.nrs_pos.items():
            ref = vals[(slot, ls, m)]
            num += grid[l, k] * np.conj(ref)
            den += abs(ref) ** 2
        return num / den

    def soft(self, t, grid):
        """Soft bits (positive = 0) of the e_sf coded bits a subframe carries, in mapping order."""
        h = self.channel(t % 10, grid)
        y = np.array([grid[l, k] for (l, k) in self.pos]) / h
        llr = np.empty(2 * len(y))
        llr[0::2] = y.real * math.sqrt(2) * 2
        llr[1::2] = y.imag * math.sqrt(2) * 2
        return llr

    def next_valid(self, t):
        while not self.valid(t):
            t += 1
        return t

    def npdcch_subframes(self, k0, r):
        ts, t = [], self.next_valid(k0)
        for _ in range(r):
            ts.append(t)
            t = self.next_valid(t + 1)
        return ts

    def decode_npdcch(self, get_grid, k0, r, rnti):
        """Decodes the candidate of R = r subframes starting at the search space start k0 for DCI of 23 bits and the
        given RNTI; returns the DCI bits or None. Also returns the subframes it used."""
        ts = self.npdcch_subframes(k0, r)
        acc = np.zeros(self.e_sf)
        for i, t in enumerate(ts):
            g = get_grid(t)
            if g is None:
                return None, ts
            first = ts[i - i % 4]
            sf_first = (first % 10240) % 10
            c = self.chk.gold((sf_first << 9) + self.pci, (i % 4 + 1) * self.e_sf)[(i % 4) * self.e_sf:]
            acc += self.soft(t, g) * (1 - 2 * np.array(c))
        k_len = 23 + 16
        bits, _ = tbcc_decode(recover(self.chk, acc, k_len))
        dci, got = bits[:23], bits[23:]
        want = crc(dci, 0x1021, 16)
        want = [b ^ ((rnti >> (15 - i)) & 1) for i, b in enumerate(want)]
        return (dci if got == want else None), ts

    def decode_npdsch(self, get_grid, n_last, k0, n_sf, n_rep, tbs, rnti):
        plan = self.plan_npdsch(self.valid, n_last, k0, n_sf, n_rep)
        acc = np.zeros(n_sf * self.e_sf)
        cache = {}
        for t, cw, psfn, psf in plan:
            g = get_grid(t)
            if g is None:
                return None, plan
            c_init = (rnti << 14) + ((psfn & 1) << 13) + (psf << 9) + self.pci
            if c_init not in cache:
                cache[c_init] = 1 - 2 * np.array(self.chk.gold(c_init, n_sf * self.e_sf))
            acc[cw * self.e_sf:(cw + 1) * self.e_sf] += self.soft(t, g) * cache[c_init][cw * self.e_sf:(cw + 1) * self.e_sf]
        k_len = tbs + 24
        bits, _ = tbcc_decode(recover(self.chk, acc, k_len))
        tb, got = bits[:tbs], bits[tbs:]
        return (tb if got == crc(tb, 0x864CFB, 24) else None), plan


# ------------------------------------------------------------------------------------------------ DCI and MAC
def bits_value(bits):
    v = 0
    for b in bits:
        v = (v << 1) | b
    return v


def parse_dci_n1(bits):
    """DCI format N1 without the NPDCCH order (TS 36.212 6.4.3.2); None if it is not a plain downlink assignment."""
    if len(bits) != 23 or bits[0] != 1 or bits[1] != 0:
        return None
    p = 2
    f = {}
    for name, w in (('i_delay', 3), ('i_sf', 3), ('i_mcs', 4), ('i_rep', 4), ('ndi', 1), ('harq', 4), ('dci_rep', 2)):
        f[name] = bits_value(bits[p:p + w])
        p += w
    return f


TBS_ROWS = {4: [56, 120, 208, 256, 328, 408, 552, 680]}     # only what the tests use: I_TBS 4 (more rows come from the caller)


def parse_rar_pdu(bits):
    """MAC PDU of TS 36.321 6.1.5 for an NB-IoT UE. Returns (backoff or None, [rar dicts]) or None if malformed."""
    p, subs, bi = 0, [], None
    while True:
        if p + 8 > len(bits):
            return None
        e, t = bits[p], bits[p + 1]
        if t == 1:
            subs.append(bits_value(bits[p + 2:p + 8]))
        else:
            bi = bits_value(bits[p + 4:p + 8])
        p += 8
        if not e:
            break
    rars = []
    for rapid in subs:
        if p + 48 > len(bits):
            return None
        r = bits[p:p + 48]
        rars.append({
            'rapid': rapid,
            'ta': bits_value(r[1:12]),
            'sc15': r[12],
            'i_sc': bits_value(r[13:19]),
            'i_delay': bits_value(r[19:21]),
            'i_rep': bits_value(r[21:24]),
            'i_mcs': bits_value(r[24:27]),
            'tc_rnti': bits_value(r[32:48]),
        })
        p += 48
    return bi, rars
