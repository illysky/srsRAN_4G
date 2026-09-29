#!/usr/bin/env python3
#
# Copyright 2013-2023 Software Radio Systems Limited
#
# This file is part of srsRAN.
#
# srsRAN is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as
# published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# srsRAN is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.
#
# A copy of the GNU Affero General Public License can be found in
# the LICENSE file in the top-level directory of this distribution
# and at http://www.gnu.org/licenses/.
#

"""
Independent check of an in-band NB-IoT capture written by npdsch_enodeb.

The srsRAN encoder and decoder share their RE-mapping code, so a decode round trip cannot tell a correct mapping
from a symmetrically wrong one. (It once "passed" while in-band data was walking across the neighbouring PRBs of
the LTE carrier.) This script re-derives where the energy has to be straight from the specifications, using its own
FFT and none of the srsRAN mapping code:

  * every occupied RE lies inside the NB-IoT anchor PRB, none in any other PRB of the LTE carrier
  * NRS sit on subcarriers 6m + ((v + PCI) mod 6), v in {0, 3}          (TS 36.211 10.2.6)
  * one NPBCH subframe carries exactly 100 QPSK data REs                (TS 36.211 10.2.4.4: 1600 bits / 8 sf)
  * an NPDCCH/NPDSCH subframe carries 12*(14-l_start) minus NRS minus LTE CRS REs, for 1 LTE CRS port

Assumes a 25 PRB LTE carrier at the standard 7.68 MS/s rate (generate the capture with -S), normal CP, 1 NRS port.
"""

import argparse
import sys

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("capture")
    ap.add_argument("--prb", type=int, default=17, help="NB-IoT anchor PRB index")
    ap.add_argument("--pci", type=int, default=1, help="LTE / NB-IoT cell id (same-PCI in-band)")
    ap.add_argument("--frame", type=int, default=20)
    args = ap.parse_args()

    n_prb, N, slot, cp0, cpn = 25, 512, 3840, 40, 36
    n_sc = 12 * n_prb
    raw = np.fromfile(args.capture, dtype=np.float32)
    x = raw[0::2] + 1j * raw[1::2]
    need = (args.frame + 1) * 10 * 2 * slot
    if len(x) < need:
        print("FAIL: capture has %d samples, need at least %d (wrong sample rate? use -S)" % (len(x), need))
        return 1

    def sym_start(sf, l):
        s = sf * 2 * slot + (l // 7) * slot
        for k in range(l % 7):
            s += N + (cp0 if k == 0 else cpn)
        return s + (cp0 if l % 7 == 0 else cpn)

    k = np.arange(N) - N // 2  # subcarrier offset from the LTE carrier centre (DC bin = 0)

    def sc_to_k(s):  # resource-grid subcarrier index -> offset from DC; the DC subcarrier is not in the grid
        return s - n_sc // 2 if s < n_sc // 2 else s - n_sc // 2 + 1

    prb_ks = [sc_to_k(12 * args.prb + i) for i in range(12)]

    def active_res(sf, l):
        s = sym_start(args.frame * 10 + sf, l)
        p = np.abs(np.fft.fftshift(np.fft.fft(x[s : s + N]))) ** 2
        if p.max() < 1e-6:
            return []
        return [int(v) for v in k[p > p.max() * 1e-3] if v != 0]

    failures = []

    def check(cond, msg):
        print(("ok    " if cond else "FAIL  ") + msg)
        if not cond:
            failures.append(msg)

    # ---- 1. nothing outside the anchor PRB, in any downlink subframe of the frame
    outside = 0
    for sf in range(10):
        for l in range(14):
            outside += sum(1 for v in active_res(sf, l) if v not in prb_ks)
    check(outside == 0, "no occupied RE outside PRB %d in frame %d (found %d)" % (args.prb, args.frame, outside))

    # ---- 2. NRS positions (sf2 carries only NRS + possibly nothing else)
    exp_nrs = sorted(prb_ks[0] + (6 * m + (v + args.pci) % 6) for v in (0, 3) for m in (0, 1))
    seen = sorted(set(v for l in (5, 6, 12, 13) for v in active_res(2, l)))
    check(seen == exp_nrs, "NRS subcarriers %s == spec %s" % (seen, exp_nrs))

    # ---- 3. NPBCH: 100 data REs in subframe 0 (all occupied REs in symbols 3..13, minus the 8 NRS pilots)
    npbch = sum(len(active_res(0, l)) for l in range(3, 14)) - 8
    check(npbch == 100, "NPBCH subframe carries %d data REs (spec: 100)" % npbch)

    # ---- 4. NPDCCH/NPDSCH subframe: find one (subframe 1 is the fixed NPDCCH position) and count REs.
    #         l_start = 3 (control region), 1 LTE CRS port -> CRS in symbols 4, 7, 11 (2 REs each).
    total = sum(len(active_res(1, l)) for l in range(3, 14))
    data = total - 8
    exp = 12 * (14 - 3) - 8 - 6
    check(data == exp, "NPDCCH subframe carries %d data REs (spec-derived: %d)" % (data, exp))

    # ---- 5. control region (symbols 0-2) must be left to the LTE cell
    ctl = sum(len(active_res(sf, l)) for sf in range(10) for l in range(3))
    check(ctl == 0, "LTE control region (symbols 0-2) untouched (found %d occupied REs)" % ctl)

    print("RESULT:", "PASS" if not failures else "FAIL (%d)" % len(failures))
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())
