#!/usr/bin/env python3
"""
Check the NPSS / NSSS transmitter against TS 36.211 10.2.7 (v14.2.0), independently of the library.

    nbiot_sync_check.py <nbiot_sync_dump> [<spec_txt>]

The table of b_q(m) (Table 10.2.7.2.1-1) is NOT taken from the library: it is embedded here as parsed from the
specification text. If <spec_txt> (TS 36.211 as text, pdftotext -layout) is given, the table is parsed from it again and
must equal the embedded one.

Checked, for several cell ids (all four q = floor(N/126) classes) and an in-band anchor in a 25-PRB carrier:

 * NPSS values d_l(n) = S(l) exp(-j pi u n(n+1)/11), u = 5, n = 0..10, S(3..13) = 1 1 1 1 -1 -1 1 1 1 -1 1.
 * NSSS values d(n) = b_q(m) exp(-j 2 pi theta_f n) exp(-j pi u n'(n'+1)/131), n = 0..131, n' = n mod 131,
   m = n mod 128, u = N mod 126 + 3, q = floor(N/126), theta_f = 33/132 * ((nf/2) mod 4). The four generated
   sequences must be the four theta_f = 0, 1/4, 1/2, 3/4 versions (and therefore DIFFER from each other).
 * NPSS placement: symbol l = 3..13, subcarriers 0..10 of the anchor PRB, in increasing k then l, nothing elsewhere.
 * NSSS placement: sequence index n -> subcarrier n mod 12, symbol 3 + n div 12, 12 subcarriers of the anchor PRB,
   nothing elsewhere; the sequence used in radio frame nf is theta index (nf/2) mod 4.
"""
import cmath
import math
import re
import subprocess
import sys

TOL = 2e-5
NOF_PRB = 25
ANCHOR = 17
W = 12 * NOF_PRB
S_L = [1, 1, 1, 1, -1, -1, 1, 1, 1, -1, 1]  # S(3)..S(13)


# b_q(m) of TS 36.211 v14.2.0 Table 10.2.7.2.1-1, as parsed from the specification text ('+' = 1, '-' = -1).
# The check parses the live spec text as well when it is given and requires the two to agree.
BQ_EMBEDDED = [
    "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++",
    "+--+-++--++-+--+-++-+--++--+-++-+--+-++--++-+--+-++-+--++--+-++-+--+-++--++-+--+-++-+--++--+-++-+--+-++--++-+--+-++-+--++--+-++-",
    "+--+-++--++-+--+-++-+--++--+-++--++-+--++--+-++-+--+-++--++-+--++--+-++--++-+--+-++-+--++--+-++--++-+--++--+-++-+--+-++--++-+--+",
    "+--+-++--++-+--+-++-+--++--+-++--++-+--++--+-++-+--+-++--++-+--+-++-+--++--+-++-+--+-++--++-+--++--+-++--++-+--+-++-+--++--+-++-",
]


def npss_d(l, n):
    return S_L[l - 3] * cmath.exp(-1j * math.pi * 5 * n * (n + 1) / 11)


def nsss_d(n, pci, theta_idx, b):
    u = pci % 126 + 3
    q = pci // 126
    theta = 33.0 / 132.0 * theta_idx
    npr = n % 131
    m = n % 128
    return b[q][m] * cmath.exp(-2j * math.pi * theta * n) * cmath.exp(-1j * math.pi * u * npr * (npr + 1) / 131)


def parse_bq(spec_txt):
    """Extract b_q(m), q = 0..3, 128 values each, from the text of Table 10.2.7.2.1-1."""
    try:
        text = open(spec_txt, encoding="utf-8", errors="replace").read()
    except OSError:
        return None
    marker = text.rfind("Table 10.2.7.2.1-1: Definition of")
    if marker < 0:
        return None
    end = text.find("10.2.7.2.2", marker)
    block = text[marker:end]
    # drop the page break furniture and the standalone row labels ("0", "1", "2", "3" on a line of their own)
    lines = [ln for ln in block.splitlines()
             if not re.fullmatch(r"\s*\d\s*", ln) and "ETSI" not in ln and "3GPP TS" not in ln]
    block = "\n".join(lines)
    groups = re.findall(r"\[([^\]]*)\]", block)
    if len(groups) != 4:
        return None
    out = []
    for g in groups:
        s = re.sub(r"\s+", "", g)
        vals = [int(t) for t in re.findall(r"-?1", s)]
        if len(vals) != 128:
            return None
        out.append(vals)
    return out


def parse(text):
    gen_npss, gen_nsss, npss_put, nsss_put = {}, {}, {}, {}
    for line in text.splitlines():
        p = line.split()
        if not p:
            continue
        if p[0] == "G" and p[1] == "NPSS":
            gen_npss[int(p[2])] = complex(float(p[3]), float(p[4]))
        elif p[0] == "G" and p[1] == "NSSS":
            gen_nsss[(int(p[2]), int(p[3]))] = complex(float(p[4]), float(p[5]))
        elif p[0] == "N":
            npss_put[int(p[2])] = complex(float(p[3]), float(p[4]))
        elif p[0] == "S":
            nsss_put.setdefault(int(p[1]), {})[int(p[2])] = complex(float(p[3]), float(p[4]))
    return gen_npss, gen_nsss, npss_put, nsss_put


def main():
    dump = sys.argv[1]
    spec = sys.argv[2] if len(sys.argv) > 2 else None
    b = [[1 if c == "+" else -1 for c in row] for row in BQ_EMBEDDED]
    checks, fails = 0, []

    def ok(cond, msg):
        nonlocal checks
        checks += 1
        if not cond:
            fails.append(msg)

    if spec:
        live = parse_bq(spec)
        if live is None:
            print("FAIL: could not parse Table 10.2.7.2.1-1 from", spec)
            return 1
        ok(live == b, "embedded b_q differs from the table in " + spec)
    else:
        print("note: no spec text given, using the embedded b_q table only")
    # sanity of the parsed table: q = 0 is all ones, every entry is +-1, and rows are distinct
    ok(all(v == 1 for v in b[0]), "b_0 should be all ones")
    ok(len({tuple(r) for r in b}) == 4, "b_q rows should be distinct")

    for pci in (0, 1, 17, 125, 126, 251, 252, 300, 377, 378, 503):
        out = subprocess.run([dump, str(pci), str(NOF_PRB), str(ANCHOR), "16"], capture_output=True, text=True,
                             check=True).stdout
        gen_npss, gen_nsss, npss_put, nsss_put = parse(out)

        # NPSS sequence: 11 symbols x 11 values, symbol-major
        ok(len(gen_npss) == 121, f"pci {pci}: NPSS length {len(gen_npss)}")
        for l in range(3, 14):
            for n in range(11):
                got = gen_npss[(l - 3) * 11 + n]
                ok(abs(got - npss_d(l, n)) < TOL, f"pci {pci}: NPSS d_{l}({n}) = {got}, want {npss_d(l, n)}")

        # NSSS sequences for the four cyclic shifts
        for t in range(4):
            for n in range(132):
                got = gen_nsss[(t, n)]
                want = nsss_d(n, pci, t, b)
                ok(abs(got - want) < TOL, f"pci {pci}: NSSS theta idx {t} d({n}) = {got}, want {want}")
        # and they really are different from each other
        for t in range(1, 4):
            ok(max(abs(gen_nsss[(t, n)] - gen_nsss[(0, n)]) for n in range(132)) > 0.5,
               f"pci {pci}: NSSS shift {t} equals shift 0 (no phase ramp)")

        # NPSS placement: exactly the 11 x 11 elements, none elsewhere
        want_idx = {}
        for l in range(3, 14):
            for k in range(11):
                want_idx[l * W + ANCHOR * 12 + k] = gen_npss[(l - 3) * 11 + k]
        ok(set(npss_put) == set(want_idx), f"pci {pci}: NPSS occupies {len(npss_put)} elements, want {len(want_idx)}")
        for idx, v in want_idx.items():
            ok(idx in npss_put and abs(npss_put[idx] - v) < TOL, f"pci {pci}: NPSS element at {idx}")

        # NSSS placement and the frame -> shift mapping
        for nf in range(16):
            t = (nf // 2) % 4
            want = {}
            for n in range(132):
                l, k = 3 + n // 12, n % 12
                want[l * W + ANCHOR * 12 + k] = nsss_d(n, pci, t, b)
            got = nsss_put.get(nf, {})
            ok(set(got) == set(want), f"pci {pci} nf {nf}: NSSS occupies {len(got)} elements, want {len(want)}")
            worst = max((abs(got.get(i, 1e9) - v) for i, v in want.items()), default=0)
            ok(worst < TOL, f"pci {pci} nf {nf}: NSSS values off by {worst} (want theta idx {t})")

    if fails:
        for m in fails[:20]:
            print("FAIL:", m)
        print(f"{len(fails)} of {checks} checks failed")
        return 1
    print(f"nbiot_sync_check: {checks} checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
