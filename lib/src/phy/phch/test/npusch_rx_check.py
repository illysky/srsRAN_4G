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


def trials(exe, cfg, tbs, snr_db, cfo_hz, n, rng, extra=()):
    """One transport block, coded once; n independent noise realisations (and static phase). Returns the number of
    correct decodes."""
    tb = rng.integers(0, 2, tbs).tolist()
    tx = R.transmit(tb, cfg)["samples"]
    want = "".join(map(str, tb))
    good = 0
    for _ in range(n):
        y = channel(tx, cfg, snr_db, cfo_hz, float(rng.uniform(-3, 3)), 0, rng)
        res, _ = run_rx(exe, cfg, y, tbs, extra)
        good += 1 if (res is not None and res["crc_ok"] == "1" and res.get("tb") == want) else 0
    return good


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
    print("near the decoding threshold: 20 noise realisations each, about 1.5 dB above the 50% point")
    # (description, config, tbs, SNR per tone, repetitions of the many-repetition case)
    points = (
        ("1 tone 15 kHz QPSK, 8 rep", lambda r: base_cfg(n_sc=1, sc=3, n_ru=1, n_rep=r, cell_id=21), 32, -8.0, 8),
        ("1 tone 15 kHz BPSK, 4 rep", lambda r: base_cfg(n_sc=1, sc=3, n_ru=2, n_rep=r, qm=1, cell_id=21), 32, -8.0, 4),
        ("1 tone 3.75 kHz BPSK, 4 rep", lambda r: base_cfg(n_sc=1, spacing=3750, sc=3, n_ru=2, n_rep=r, qm=1, cell_id=21), 32, -9.0, 4),
        ("1 tone 3.75 kHz QPSK, 4 rep", lambda r: base_cfg(n_sc=1, spacing=3750, sc=20, n_ru=1, n_rep=r, cell_id=21), 32, -5.0, 4),
        ("3 tones, 2 RU, 4 rep", lambda r: base_cfg(n_sc=3, sc=3, n_ru=2, n_rep=r, cell_id=21), 88, -7.0, 4),
        ("6 tones, 2 RU, rv 2, 4 rep", lambda r: base_cfg(n_sc=6, sc=6, n_ru=2, n_rep=r, cell_id=21, rv=2), 88, -7.0, 4),
        ("12 tones, 1 RU, 8 rep", lambda r: base_cfg(n_sc=12, sc=0, n_ru=1, n_rep=r, cell_id=21), 88, -6.0, 8),
    )
    for name, mk, tbs, snr, reps in points:
        many = trials(exe, mk(reps), tbs, snr, 15.0, 20, rng)
        single = trials(exe, mk(1), tbs, snr, 15.0, 4, rng)
        total += 1
        good = many >= 15 and single == 0
        failures += 0 if good else 1
        print("%-4s %-46s %2d/20 with %d repetitions, %d/4 with one, at %+.0f dB" %
              ("ok" if good else "FAIL", name, many, reps, single, snr))
    print("large frequency offsets (they matter for the rotation of the channel estimate within a slot)")
    for name, cfg, cfo in (("1 tone 15 kHz, CFO -700 Hz", base_cfg(n_sc=1, sc=3, n_ru=1, n_rep=2, cell_id=21), -700.0),
                           ("1 tone 15 kHz, CFO +700 Hz", base_cfg(n_sc=1, sc=3, n_ru=1, n_rep=2, cell_id=21), 700.0),
                           ("1 tone 3.75 kHz, CFO -220 Hz", base_cfg(n_sc=1, spacing=3750, sc=3, n_ru=1, n_rep=2, cell_id=21), -220.0),
                           ("1 tone 3.75 kHz, CFO +220 Hz", base_cfg(n_sc=1, spacing=3750, sc=3, n_ru=1, n_rep=2, cell_id=21), 220.0),
                           ("12 tones 15 kHz, CFO +700 Hz", base_cfg(n_sc=12, sc=0, n_ru=1, n_rep=2, cell_id=21), 700.0)):
        report(name, one(exe, cfg, 32 if cfg["n_sc"] == 1 else 88, 12.0, cfo, 0.4, 0, rng))
    print("estimators")
    # the reported SNR must stay close to the truth at high SNR even at the edge of the timing window, where a
    # window that starts too late would pick up the next symbol
    for name, cfg, delay in (("1 tone 15 kHz, 30 dB, early by 4 samples", base_cfg(n_sc=1, sc=3, n_ru=1, n_rep=2, cell_id=21), -4),
                             ("1 tone 15 kHz, 30 dB, late by 5 samples", base_cfg(n_sc=1, sc=3, n_ru=1, n_rep=2, cell_id=21), 5),
                             ("1 tone 3.75 kHz, 30 dB, early by 8 samples", base_cfg(n_sc=1, spacing=3750, sc=3, n_ru=1, n_rep=2, cell_id=21), -8)):
        r = one(exe, cfg, 32, 30.0, 0.0, 0.2, delay, rng)
        total += 1
        good = r["ok"] and r["snr"] >= 26.0
        failures += 0 if good else 1
        print("%-4s %-46s reported %.1f dB (true 30)" % ("ok" if good else "FAIL", name, r.get("snr", float("nan"))))
    # a short window leaves 1 - 1/W of the noise in the residual; the estimate has to correct for it
    cfg = base_cfg(n_sc=1, sc=3, n_ru=1, n_rep=8, cell_id=21)
    r = one(exe, cfg, 32, 12.0, 10.0, 0.2, 0, rng, extra=("win=2",))
    total += 1
    good = r["ok"] and 10.5 <= r["snr"] <= 13.5
    failures += 0 if good else 1
    print("%-4s %-46s reported %.1f dB (true 12)" % ("ok" if good else "FAIL", "SNR estimate with a 2 slot window", r.get("snr", float("nan"))))
    print("high code rate (no redundancy to hide errors), 25 dB per tone, frequency, phase and timing offsets")
    hard = (
        ("1 tone 15 kHz QPSK, CFO +700 Hz", base_cfg(n_sc=1, sc=3, n_ru=1, cell_id=21), 104, 700.0),
        ("1 tone 15 kHz QPSK, CFO -700 Hz", base_cfg(n_sc=1, sc=9, n_ru=1, cell_id=22), 104, -700.0),
        ("1 tone 15 kHz BPSK, CFO +500 Hz", base_cfg(n_sc=1, sc=1, n_ru=2, qm=1, cell_id=23), 104, 500.0),
        ("1 tone 3.75 kHz QPSK, CFO +220 Hz", base_cfg(n_sc=1, spacing=3750, sc=3, n_ru=1, cell_id=24), 104, 220.0),
        ("1 tone 3.75 kHz BPSK, CFO -220 Hz", base_cfg(n_sc=1, spacing=3750, sc=30, n_ru=2, qm=1, cell_id=25), 104, -220.0),
        ("3 tones, CFO +500 Hz", base_cfg(n_sc=3, sc=3, n_ru=1, cell_id=26), 144, 500.0),
        ("6 tones, CFO -500 Hz", base_cfg(n_sc=6, sc=0, n_ru=1, cell_id=27), 144, -500.0),
        ("12 tones, CFO +700 Hz", base_cfg(n_sc=12, sc=0, n_ru=1, cell_id=28), 144, 700.0),
        ("group hopping 15 kHz, 2 RU, CFO +300 Hz",
         base_cfg(n_sc=1, sc=4, n_ru=2, group_hopping=True, delta_ss=5, slot=6, frame=77, cell_id=200), 208, 300.0),
        ("group hopping 15 kHz, 2 RU, frame 3 slot 13",
         base_cfg(n_sc=1, sc=4, n_ru=2, group_hopping=True, delta_ss=3, slot=13, frame=3, cell_id=333), 208, -300.0),
        ("group hopping 3.75 kHz, 1 RU",
         base_cfg(n_sc=1, spacing=3750, sc=4, n_ru=1, group_hopping=True, delta_ss=11, slot=2, frame=3, cell_id=17), 104, 100.0),
        ("group hopping 3.75 kHz, 2 RU, frame 1 slot 4",
         base_cfg(n_sc=1, spacing=3750, sc=4, n_ru=2, qm=1, group_hopping=True, delta_ss=2, slot=4, frame=1, cell_id=444), 104, -100.0),
    )
    for name, cfg, tbs, cfo in hard:
        report(name, one(exe, cfg, tbs, 25.0, cfo, float(rng.uniform(-3, 3)), int(rng.integers(-2, 3)), rng))
    print("group hopping with a 2 slot channel estimate window (a wrong reference can not be averaged away)")
    for name, cfg in (("group hopping 15 kHz, 2 RU", base_cfg(n_sc=1, sc=4, n_ru=2, group_hopping=True, delta_ss=5, slot=6, frame=77, cell_id=200)),
                      ("group hopping 3.75 kHz, 2 RU", base_cfg(n_sc=1, spacing=3750, sc=4, n_ru=1, n_rep=2, group_hopping=True, delta_ss=11, slot=3, frame=2, cell_id=17))):
        report(name, one(exe, cfg, 24, 20.0, 5.0, 0.3, 0, rng, extra=("win=2",)))
    print("frequency offset accuracy at half a periodogram bin (7.8 Hz bins for 32 slots at 15 kHz)")
    cfg = base_cfg(n_sc=1, sc=3, n_ru=1, n_rep=2, cell_id=21)
    for cfo in (3.9, 35.2, -27.3):
        r = one(exe, cfg, 32, 25.0, cfo, 0.2, 0, rng)
        total += 1
        good = r["ok"] and abs(r["cfo"] - cfo) <= 2.0
        failures += 0 if good else 1
        print("%-4s %-46s reported %.1f Hz" % ("ok" if good else "FAIL", "CFO %+.1f Hz" % cfo, r.get("cfo", float("nan"))))
    print("timing window at very high SNR: early arrival leaves a next-symbol contribution of d/N of the amplitude")
    for name, cfg, delay in (("1 tone 15 kHz, 55 dB, early by 4 samples", base_cfg(n_sc=1, sc=3, n_ru=1, n_rep=2, cell_id=21), -4),
                             ("1 tone 3.75 kHz, 55 dB, early by 8 samples", base_cfg(n_sc=1, spacing=3750, sc=3, n_ru=1, n_rep=2, cell_id=21), -8)):
        r = one(exe, cfg, 32, 55.0, 0.0, 0.2, delay, rng)
        total += 1
        good = r["ok"] and r["snr"] >= 45.0
        failures += 0 if good else 1
        print("%-4s %-46s reported %.1f dB (true 55)" % ("ok" if good else "FAIL", name, r.get("snr", float("nan"))))
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
