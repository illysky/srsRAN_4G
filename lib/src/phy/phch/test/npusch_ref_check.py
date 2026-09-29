#!/usr/bin/env python3
"""
Compares the independent Python NPUSCH model (npusch_ref.py) with the library's own coding blocks, bit for bit:
CRC24A, the rate 1/3 turbo encoder and the rate matching (sub-block interleaver, circular buffer, RV 0 and 2) over
transport block sizes of the NB-IoT tables, both ends of the E range (E much smaller and much larger than the mother
code) and random data.

  npusch_ref_check.py PATH_TO_npusch_fec_dump
"""
import subprocess
import sys
import os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import npusch_ref as R

# NB-IoT UL transport block sizes (TS 36.213 Table 16.5.1.2-2) that appear with single tone / few resource units
TBS = [16, 24, 32, 40, 56, 72, 88, 104, 120, 136, 144, 176, 208, 224, 256, 328, 424, 536, 600, 712, 808, 936, 1000]

def main():
    exe = sys.argv[1]
    rng = np.random.default_rng(20260929)
    failures = 0
    checks = 0
    for a in TBS:
        k = a + 24
        if k not in R.QPP_PARAMS:
            print("TBS %d + 24 = %d is not a turbo block size, skipped" % (a, k))
            continue
        d = k + 4
        for e in (96, 192, 3 * d - 40, 3 * d, 3 * d + 57, 5 * d + 3, 1536):
            for rv in (0, 2):
                tb = rng.integers(0, 2, a).tolist()
                out = subprocess.run([exe, str(a), str(e), str(rv)], input="".join(map(str, tb)), capture_output=True,
                                     text=True)
                if out.returncode != 0:
                    print("tool failed for A=%d E=%d rv=%d: %s" % (a, e, rv, out.stderr.strip()))
                    failures += 1
                    continue
                lines = dict(l.split(": ") for l in out.stdout.strip().split("\n"))
                crc = R.crc24a(tb)
                d0, d1, d2 = R.turbo_encode(tb + crc)
                turbo = "".join("%d%d%d" % (d0[i], d1[i], d2[i]) for i in range(d))
                rm = "".join(map(str, R.rate_match(d0, d1, d2, e, rv)))
                for name, mine, theirs in (("crc", "".join(map(str, crc)), lines["crc"]), ("turbo", turbo, lines["turbo"]),
                                           ("rate matching", rm, lines["rm"])):
                    checks += 1
                    if mine != theirs:
                        failures += 1
                        first = next(i for i in range(min(len(mine), len(theirs))) if mine[i] != theirs[i])
                        print("MISMATCH %s: A=%d E=%d rv=%d first difference at bit %d" % (name, a, e, rv, first))
    print("npusch_ref_check: %d comparisons, %d failures" % (checks, failures))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
