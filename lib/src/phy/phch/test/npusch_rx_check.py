#!/usr/bin/env python3
"""
End to end check of the NPUSCH format 1 receiver (npusch.c) on waveforms made by the independent Python transmitter
(npusch_ref.py): random transport blocks are coded and modulated, sent through a channel (static phase, frequency
offset, integer sample timing offset, complex Gaussian noise) and decoded by the C receiver through npusch_rx_dump.

  npusch_rx_check.py PATH_TO_npusch_rx_dump [quick|full|explore]

Noise is specified per tone: the SNR is the power of one tone over the noise power in the FFT bin of that tone (the
number the receiver reports as snr_db).
"""
import os
import subprocess
import sys
import tempfile
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import npusch_ref as R

FS = 1.92e6


def fft_len(spacing):
    return 512 if spacing == 3750 else 128


def channel(x, cfg, snr_db, cfo_hz, phase, delay, rng):
    """Static phase and gain 1, frequency offset, delay in samples (positive = late), noise"""
    n = len(x)
    if delay > 0:
        x = np.concatenate([np.zeros(delay, dtype=x.dtype), x[:n - delay]])
    elif delay < 0:
        x = np.concatenate([x[-delay:], np.zeros(-delay, dtype=x.dtype)])
    t = np.arange(n) / FS
    y = x * np.exp(1j * (2 * np.pi * cfo_hz * t + phase))
    if snr_db is not None:
        sigma2 = fft_len(cfg["spacing"]) * 10 ** (-snr_db / 10.0)          # per complex sample
        y = y + (rng.standard_normal(n) + 1j * rng.standard_normal(n)) * np.sqrt(sigma2 / 2)
    return y


def run_rx(exe, cfg, samples, tbs, extra=()):
    with tempfile.NamedTemporaryFile(suffix=".cf32", delete=False) as f:
        samples.astype(np.complex64).tofile(f)
        path = f.name
    args = [exe, path,
            "n_sc=%d" % cfg["n_sc"], "spacing=%d" % cfg["spacing"], "sc=%d" % cfg["sc"], "n_ru=%d" % cfg["n_ru"],
            "n_rep=%d" % cfg["n_rep"], "rv=%d" % cfg.get("rv", 0), "qm=%d" % cfg.get("qm", 2), "tbs=%d" % tbs,
            "rnti=%d" % cfg["rnti"], "cell=%d" % cfg["cell_id"], "frame=%d" % cfg["frame"], "slot=%d" % cfg["slot"],
            "gh=%d" % int(cfg.get("group_hopping", False)), "dss=%d" % cfg.get("delta_ss", 0),
            "cs=%d" % cfg.get("cyclic_shift", 0)]
    if cfg.get("base_seq") is not None:
        args.append("base=%d" % cfg["base_seq"])
    args.extend(extra)
    try:
        out = subprocess.run(args, capture_output=True, text=True)
    finally:
        os.unlink(path)
    if out.returncode != 0:
        return None, out.stderr.strip()
    res = dict(l.split(" ", 1) for l in out.stdout.strip().split("\n"))
    return res, ""


def one(exe, cfg, tbs, snr_db, cfo_hz, phase, delay, rng, rx_override=None, extra=()):
    tb = rng.integers(0, 2, tbs).tolist()
    tx = R.transmit(tb, cfg)
    y = channel(tx["samples"], cfg, snr_db, cfo_hz, phase, delay, rng)
    rxcfg = dict(cfg)
    if rx_override:
        rxcfg.update(rx_override)
    res, err = run_rx(exe, rxcfg, y, tbs, extra)
    if res is None:
        return {"ok": False, "crc": False, "err": err}
    ok = res["crc_ok"] == "1" and res.get("tb") == "".join(map(str, tb))
    return {"ok": ok, "crc": res["crc_ok"] == "1", "snr": float(res["snr_db"]), "cfo": float(res["cfo_hz"]),
            "iters": int(res["iters"]), "err": err}


def base_cfg(**kw):
    c = dict(n_sc=1, spacing=15000, sc=0, n_ru=1, n_rep=1, rnti=0x1234, cell_id=1, frame=0, slot=0, rv=0, qm=2)
    c.update(kw)
    return c


# (description, cfg, tbs)
def geometry_cases():
    cases = []
    for sp, sc in ((3750, 0), (3750, 17), (3750, 47), (15000, 0), (15000, 5), (15000, 11)):
        for qm in (1, 2):
            cases.append(("1 tone %d Hz sc %d qm %d" % (sp, sc, qm), base_cfg(n_sc=1, spacing=sp, sc=sc, qm=qm, n_ru=2 if qm == 1 else 1,
                                                                                    cell_id=(sc * 7 + 3) % 504), 32))
    for n_sc, scs in ((3, (0, 3, 9)), (6, (0, 6)), (12, (0,))):
        for sc in scs:
            cases.append(("%d tones sc %d" % (n_sc, sc), base_cfg(n_sc=n_sc, sc=sc, n_ru=1, cell_id=(sc * 5 + 11) % 504), 56))
    for cs in (1, 2):
        cases.append(("3 tones cyclic shift %d" % cs, base_cfg(n_sc=3, sc=3, cyclic_shift=cs, cell_id=77), 56))
    for cs in (1, 3):
        cases.append(("6 tones cyclic shift %d" % cs, base_cfg(n_sc=6, sc=6, cyclic_shift=cs, cell_id=78), 56))
    cases.append(("12 tones base sequence 7", base_cfg(n_sc=12, sc=0, base_seq=7, cell_id=79), 56))
    cases.append(("3 tones base sequence 5", base_cfg(n_sc=3, sc=0, base_seq=5, cell_id=79), 56))
    return cases


def coding_cases():
    cases = []
    cases.append(("rv 2, 1 tone", base_cfg(n_sc=1, sc=3, n_ru=2, rv=2, cell_id=9), 40))
    cases.append(("rv 2, 6 tones", base_cfg(n_sc=6, sc=0, n_ru=2, rv=2, cell_id=9), 88))
    cases.append(("larger block, 12 tones, 4 RU", base_cfg(n_sc=12, sc=0, n_ru=4, cell_id=301), 424))
    cases.append(("larger block, 3 tones, 5 RU", base_cfg(n_sc=3, sc=0, n_ru=5, cell_id=301), 208))
    cases.append(("high rate, 12 tones 1 RU (punctured)", base_cfg(n_sc=12, sc=0, n_ru=1, cell_id=5), 104))
    cases.append(("repetitions 2, 1 tone", base_cfg(n_sc=1, sc=2, n_ru=1, n_rep=2, cell_id=33), 32))
    cases.append(("repetitions 4, 1 tone 3.75 kHz", base_cfg(n_sc=1, spacing=3750, sc=9, n_ru=1, n_rep=4, cell_id=34), 32))
    cases.append(("repetitions 2, 3 tones (M_ident 1)", base_cfg(n_sc=3, sc=6, n_ru=1, n_rep=2, cell_id=35), 56))
    cases.append(("repetitions 4, 6 tones (M_ident 2)", base_cfg(n_sc=6, sc=0, n_ru=2, n_rep=4, cell_id=36), 88))
    cases.append(("repetitions 8, 12 tones (M_ident 4)", base_cfg(n_sc=12, sc=0, n_ru=1, n_rep=8, cell_id=37), 88))
    cases.append(("frame / slot offset, 15 kHz", base_cfg(n_sc=1, sc=1, n_ru=1, n_rep=3 - 1, frame=513, slot=7, rnti=0xFFF3, cell_id=503), 40))
    cases.append(("frame / slot offset, 3.75 kHz", base_cfg(n_sc=1, spacing=3750, sc=30, n_ru=1, n_rep=2, frame=1023, slot=3, rnti=0x0101, cell_id=444), 40))
    cases.append(("group hopping 15 kHz", base_cfg(n_sc=1, sc=4, n_ru=2, group_hopping=True, delta_ss=5, slot=6, frame=77, cell_id=200), 40))
    cases.append(("group hopping 3.75 kHz", base_cfg(n_sc=1, spacing=3750, sc=4, n_ru=1, n_rep=2, group_hopping=True, delta_ss=11, slot=3, frame=2, cell_id=17), 24))
    return cases


def main():
    exe = sys.argv[1]
    mode = sys.argv[2] if len(sys.argv) > 2 else "quick"
    rng = np.random.default_rng(20260930)
    failures = 0
    total = 0

    def report(name, r, want=True):
        nonlocal failures, total
        total += 1
        good = r["ok"] == want
        if not good:
            failures += 1
        print("%-4s %-46s %s" % ("ok" if good else "FAIL", name,
                                 "" if r.get("snr") is None else "snr %.1f dB cfo %.1f Hz iters %d" % (r["snr"], r["cfo"], r["iters"])
                                 + ("" if not r["err"] else " " + r["err"])))

    if mode == "explore":
        for name, cfg, tbs in geometry_cases() + coding_cases():
            report(name + " (clean)", one(exe, cfg, tbs, None, 0.0, 0.0, 0, rng))
        return 1 if failures else 0

    print("noiseless")
    for name, cfg, tbs in geometry_cases() + coding_cases():
        report(name, one(exe, cfg, tbs, None, 0.0, 0.0, 0, rng))
    print("channel: phase, frequency offset, timing offset, 12 dB per tone SNR")
    for name, cfg, tbs in geometry_cases():
        if "1 tone" in name or "12 tones sc 0" == name:
            report(name, one(exe, cfg, tbs, 12.0, float(rng.uniform(-40, 40)), float(rng.uniform(-3, 3)), int(rng.integers(-3, 4)), rng))
    print("frequency and timing offsets at the edge of what the receiver supports")
    cfg = base_cfg(n_sc=1, sc=3, n_ru=1, n_rep=2, cell_id=21)
    for cfo in (-200.0, 200.0):
        report("1 tone 15 kHz, CFO %+.0f Hz" % cfo, one(exe, cfg, 32, 12.0, cfo, 0.7, 0, rng))
    cfg375 = base_cfg(n_sc=1, spacing=3750, sc=3, n_ru=1, n_rep=2, cell_id=21)
    for cfo in (-100.0, 100.0):
        report("1 tone 3.75 kHz, CFO %+.0f Hz" % cfo, one(exe, cfg375, 32, 12.0, cfo, 0.7, 0, rng))
    cfg12 = base_cfg(n_sc=12, sc=0, n_ru=1, cell_id=21)
    for delay in (-4, 5):
        report("1 tone 15 kHz, timing offset %+d samples" % delay, one(exe, cfg, 32, 15.0, 0.0, 0.3, delay, rng))
        report("12 tones 15 kHz, timing offset %+d samples" % delay, one(exe, cfg12, 88, 15.0, 0.0, 0.3, delay, rng))
    for delay in (-8, 8):
        report("1 tone 3.75 kHz, timing offset %+d samples" % delay, one(exe, cfg375, 32, 15.0, 0.0, 0.3, delay, rng))
    print("repetitions combine below the SNR of a single transmission (10 blocks each)")

    def combine(name, mk, tbs, snr, reps, cfo):
        good = sum(one(exe, mk(reps), tbs, snr, cfo, float(rng.uniform(-3, 3)), 0, rng)["ok"] for _ in range(10))
        return good

    for name, mk, tbs, snr, reps in (
            ("1 tone 15 kHz", lambda r: base_cfg(n_sc=1, sc=3, n_ru=1, n_rep=r, cell_id=21), 32, -7.0, 8),
            ("1 tone 3.75 kHz", lambda r: base_cfg(n_sc=1, spacing=3750, sc=20, n_ru=1, n_rep=r, cell_id=21), 32, -4.0, 4),
            ("3 tones", lambda r: base_cfg(n_sc=3, sc=3, n_ru=2, n_rep=r, cell_id=21), 88, -4.0, 4),
            ("12 tones", lambda r: base_cfg(n_sc=12, sc=0, n_ru=1, n_rep=r, cell_id=21), 88, -4.0, 8)):
        many = combine(name, mk, tbs, snr, reps, 15.0)
        single = combine(name, mk, tbs, snr, 1, 15.0)
        total += 1
        good = many >= 9 and single <= 1
        failures += 0 if good else 1
        print("%-4s %-46s %d/10 blocks with %d repetitions, %d/10 with one, at %+.0f dB" %
              ("ok" if good else "FAIL", name, many, reps, single, snr))
    print("negative controls")
    cfg = base_cfg(n_sc=6, sc=0, n_ru=2, cell_id=9)
    report("wrong RNTI is rejected", one(exe, cfg, 88, None, 0.0, 0.0, 0, rng, rx_override={"rnti": 0x1235}), want=False)
    report("wrong cell id is rejected", one(exe, cfg, 88, None, 0.0, 0.0, 0, rng, rx_override={"cell_id": 10}), want=False)
    report("wrong slot (scrambling) is rejected", one(exe, cfg, 88, None, 0.0, 0.0, 0, rng, rx_override={"slot": 2}), want=False)
    cfg = base_cfg(n_sc=1, sc=3, n_ru=1, cell_id=9)
    report("noise only is rejected", one(exe, cfg, 32, -60.0, 0.0, 0.0, 0, rng), want=False)
    print("npusch_rx_check: %d cases, %d failures" % (total, failures))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
