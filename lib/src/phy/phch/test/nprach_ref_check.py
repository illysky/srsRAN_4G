#!/usr/bin/env python3
"""
Compares the C NPRACH implementation with the independent Python model in nprach_ref.py.

  nprach_ref_check.py PATH_TO_nprach_test

Hopping (clause 10.1.6.1) is compared for a spread of cells / starting subcarriers / lengths. The generated
baseband preamble (clause 10.1.6.2) is compared sample by sample for both formats, several repetition counts
(including 128, which contains the 40 ms gap) and subcarrier offsets.

The Python side implements the specification text directly and shares nothing with the C code, so an error in
reading the standard would have to be made twice, in two different ways, to go unnoticed. (It is not evidence
against a misreading common to both; that is what the third party test vectors are for.)
"""
import os
import subprocess
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import nprach_ref as ref  # noqa: E402


def c_hops(exe, cell, n_init, groups):
    out = subprocess.run([exe, "-H", "%d,%d,%d" % (cell, n_init, groups)], capture_output=True, text=True, check=True)
    return [int(v) for v in out.stdout.split()]


def main():
    exe = sys.argv[1]
    failures = 0

    # --- hopping ------------------------------------------------------------------------------------------------
    n = 0
    for cell in (0, 1, 2, 137, 255, 256, 400, 503):
        for n_init in (0, 1, 5, 11, 12, 30, 47):
            for groups in (4, 8, 64, 512):
                py = ref.hops(cell, n_init, groups)
                c = c_hops(exe, cell, n_init, groups)
                n += 1
                if py != c:
                    failures += 1
                    first = next(i for i in range(groups) if py[i] != c[i])
                    print("FAIL hops cell %d n_init %d groups %d: first difference at group %d (python %d, C %d)"
                          % (cell, n_init, groups, first, py[first], c[first]))
    print("hops: %d combinations compared" % n)

    # --- waveform -----------------------------------------------------------------------------------------------
    cases = [
        (1, 0, 1, 0, 12, 0),
        (1, 0, 4, 0, 48, 25),
        (7, 1, 2, 36, 12, 5),
        (33, 0, 8, 12, 24, 17),
        (300, 1, 4, 34, 12, 11),
        (503, 0, 2, 2, 12, 3),
        (9, 0, 128, 0, 12, 4),  # contains the 40 ms gap after 64 repetitions
    ]
    with tempfile.TemporaryDirectory() as d:
        for cell, fmt, nrep, off, nsc, n_init in cases:
            path = os.path.join(d, "x.cf32")
            subprocess.run([exe, "-W", "%s,%d,%d,%d,%d,%d,%d" % (path, cell, fmt, nrep, off, nsc, n_init)], check=True)
            c = np.fromfile(path, dtype=np.complex64)
            py, _ = ref.preamble(1.92e6, fmt, nrep, cell, n_init, n_sc_offset=off)
            if len(c) != len(py):
                failures += 1
                print("FAIL waveform cell %d fmt %d nrep %d: length C %d python %d" % (cell, fmt, nrep, len(c), len(py)))
                continue
            err = np.max(np.abs(c - py))
            status = "ok" if err < 2e-4 else "FAIL"
            if err >= 2e-4:
                failures += 1
            print("waveform cell %3d fmt %d nrep %3d offset %2d nsc %2d n_init %2d: %d samples, max |C - python| = %.2e  %s"
                  % (cell, fmt, nrep, off, nsc, n_init, len(c), err, status))

    print("nprach_ref_check: %d failures" % failures)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
