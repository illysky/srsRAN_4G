#!/usr/bin/env python3
"""
Check the NPBCH transmitter against TS 36.211 10.2.4.4 (v13.6.0 / v14.x), independently of the library.

    npbch_tx_check.py <npbch_tx_dump>

What is checked, for several cell ids and 192 frames (three 64-frame periods, 24 blocks):

 1. Rotation. In frame f the transmitted symbols are y_f(i) = theta_f(i) * y(K*floor(f/8) + i): the SAME block y is sent
    in 8 consecutive frames, each multiplied by its own theta_f. So for frames f = 8b + j the ratio
    stream[f] / stream[8b] must equal theta_j(i) / theta_0(i), whatever the modulated data are. theta comes from a
    Gold sequence implemented here from TS 36.211 7.2, initialised per radio frame with
    c_init = (N+1) * ((nf mod 8) + 1)^3 * 2^9 + N.  (A transmitter that rotates its cached block in place applies
    theta_0 .. theta_j cumulatively and fails this for j >= 1: that was the pre-fix behaviour.)
 2. All symbols are unit QPSK points (|re|,|im| = 1/sqrt2 before rotation; rotation by 1, -1, j, -j keeps that).
 3. The stateless encoder, called for every frame in shuffled order, reproduces the streaming encoder exactly.
 4. The stateless encoder really depends on the payload: frames of different 64-frame periods carry different data
    (the dump changes the SFN MSBs in the MIB per period), so a stale-block bug cannot pass.
"""
import cmath
import subprocess
import sys

TOL = 1e-4
NFRAMES = 192


def gold(cinit, n):
    """TS 36.211 7.2: c(n) = x1(n+Nc) + x2(n+Nc) mod 2, Nc = 1600."""
    total = 1600 + n
    x1 = [0] * (total + 31)
    x2 = [0] * (total + 31)
    x1[0] = 1
    for i in range(31):
        x2[i] = (cinit >> i) & 1
    for i in range(total):
        x1[i + 31] = (x1[i + 3] + x1[i]) & 1
        x2[i + 31] = (x2[i + 3] + x2[i + 2] + x2[i + 1] + x2[i]) & 1
    return [(x1[i + 1600] + x2[i + 1600]) & 1 for i in range(n)]


THETA = {(0, 0): 1, (0, 1): -1, (1, 0): 1j, (1, 1): -1j}


def theta(n_id, nf):
    cinit = (n_id + 1) * ((nf % 8) + 1) ** 3 * 2 ** 9 + n_id
    c = gold(cinit, 200)
    return [THETA[(c[2 * i], c[2 * i + 1])] for i in range(100)]


def parse(text):
    stream, lite, payload = {}, {}, {}
    for line in text.splitlines():
        p = line.split()
        if not p:
            continue
        tag, f = p[0], int(p[1])
        if tag in ("S", "L"):
            v = [complex(float(p[2 + 2 * i]), float(p[3 + 2 * i])) for i in range(100)]
            (stream if tag == "S" else lite)[f] = v
        elif tag == "P":
            payload[f] = tuple(int(x) for x in p[2:])
    return stream, lite, payload


def main():
    dump = sys.argv[1]
    checks = 0
    fails = []

    def ok(cond, msg):
        nonlocal checks
        checks += 1
        if not cond:
            fails.append(msg)

    for pci in (1, 0, 5, 17, 255, 256, 503):
        out = subprocess.run([dump, str(pci), str(NFRAMES)], capture_output=True, text=True, check=True).stdout
        stream, lite, payload = parse(out)
        ok(len(stream) == NFRAMES and len(lite) == NFRAMES, f"pci {pci}: expected {NFRAMES} frames from each encoder")
        th = {j: theta(pci, j) for j in range(8)}

        for f in range(NFRAMES):
            # 2. unit QPSK points
            for i in range(100):
                a = abs(stream[f][i])
                ok(abs(a - 1.0) < TOL, f"pci {pci} f {f} re {i}: |x| = {a}")
            # 3. stateless == streaming
            worst = max(abs(stream[f][i] - lite[f][i]) for i in range(100))
            ok(worst < TOL, f"pci {pci} f {f}: stateless differs from streaming by {worst}")

        # 1. rotation relative to the first frame of each block
        for b in range(NFRAMES // 8):
            base = stream[8 * b]
            for j in range(1, 8):
                f = 8 * b + j
                worst = 0.0
                for i in range(100):
                    got = stream[f][i] / base[i]
                    exp = th[j][i] / th[0][i]
                    worst = max(worst, abs(got - exp))
                ok(worst < TOL, f"pci {pci} block {b} frame {f} (j={j}): rotation off by {worst}")

        # 4. data differ between 64-frame periods, and are constant in payload within one
        for f in range(NFRAMES):
            ok(payload[f] == payload[f & ~63], f"pci {pci}: payload changed inside a period at frame {f}")
        ok(payload[0] != payload[64] and payload[64] != payload[128], f"pci {pci}: payload should change per period")
        # different periods -> different modulated blocks: compare de-rotated frame 0/64/128 (all have j=0 -> theta_0)
        ok(any(abs(stream[0][i] - stream[64][i]) > 0.1 for i in range(100)), f"pci {pci}: period 1 repeats period 0")
        ok(any(abs(stream[64][i] - stream[128][i]) > 0.1 for i in range(100)), f"pci {pci}: period 2 repeats period 1")

    if fails:
        for m in fails[:20]:
            print("FAIL:", m)
        print(f"{len(fails)} of {checks} checks failed")
        return 1
    print(f"npbch_tx_check: {checks} checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
