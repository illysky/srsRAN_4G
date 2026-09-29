/**
 * Copyright 2013-2023 Software Radio Systems Limited
 *
 * This file is part of srsRAN.
 *
 * srsRAN is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsRAN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

/*
 * NPRACH generator/detector tests.
 *
 *   nprach_test                       self checks (run by ctest)
 *   nprach_test -H cell,n_init,groups print the hopping pattern (compared with nprach_ref.py by ctest)
 *   nprach_test -W file,cell,fmt,nrep,offset,nsc,n_init    write the generated preamble (ditto)
 *   nprach_test -E                    print detection statistics versus SNR (to choose thresholds)
 */

#include "srsran/phy/phch/nprach.h"
#include "srsran/phy/utils/vector.h"
#include <complex.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures = 0;
static int checks   = 0;

#define CHECK(cond, ...)                                                                                               \
  do {                                                                                                                 \
    checks++;                                                                                                          \
    if (!(cond)) {                                                                                                     \
      failures++;                                                                                                      \
      printf("FAIL %s:%d: ", __FILE__, __LINE__);                                                                      \
      printf(__VA_ARGS__);                                                                                             \
      printf("\n");                                                                                                    \
    }                                                                                                                  \
  } while (0)

static cf_t* cf_zalloc(uint32_t n)
{
  cf_t* p = srsran_vec_cf_malloc(n);
  if (p) {
    bzero(p, sizeof(cf_t) * n);
  }
  return p;
}

/* --- deterministic noise ------------------------------------------------------------------------------------- */
static uint64_t rng_state = 88172645463325252ull;
static double   rnd(void)
{
  rng_state ^= rng_state << 13;
  rng_state ^= rng_state >> 7;
  rng_state ^= rng_state << 17;
  return ((double)(rng_state >> 11) + 0.5) / 9007199254740992.0;
}
static void seed(uint64_t s)
{
  rng_state = s ? s : 1;
  for (int i = 0; i < 8; i++) {
    (void)rnd();
  }
}
static void add_noise(cf_t* x, uint32_t n, double sigma2)
{
  double s = sqrt(sigma2 / 2.0);
  for (uint32_t i = 0; i < n; i++) {
    double u = rnd(), v = rnd();
    double r = sqrt(-2.0 * log(u));
    x[i] += (float)(s * r * cos(2 * M_PI * v)) + I * (float)(s * r * sin(2 * M_PI * v));
  }
}

/**
 * Preamble as received after a delay of tau samples (>= 0, fractional allowed) with residual CFO cfo_hz, added
 * onto 'x' with the given amplitude. Written separately from srsran_nprach_gen (which has no delay) so the
 * detector is exercised with a signal it did not produce itself.
 */
static void add_received(const srsran_nprach_cfg_t* cfg,
                         uint32_t                   n_init,
                         double                     tau,
                         double                     cfo_hz,
                         double                     amp,
                         cf_t*                      x,
                         uint32_t                   x_len)
{
  uint32_t n_groups = 4 * cfg->n_rep;
  uint8_t  hop[SRSRAN_NPRACH_MAX_GROUPS];
  srsran_nprach_hops(cfg->cell_id, n_init, n_groups, hop);
  uint32_t cp   = srsran_nprach_cp_len(cfg->format);
  uint32_t glen = srsran_nprach_group_len(cfg->format);
  uint32_t base = cfg->n_sc_offset + (n_init / 12) * 12;
  uint32_t ti   = (uint32_t)floor(tau);
  double   tf   = tau - ti;
  for (uint32_t g = 0; g < n_groups; g++) {
    double   f     = ((double)(base + hop[g]) - 24.0 + 0.5) * 3750.0;
    uint32_t start = srsran_nprach_group_start(cfg, g) + ti;
    for (uint32_t m = 0; m < glen && start + m < x_len; m++) {
      double t_tone = ((double)m - cp - tf) / 1.92e6;
      double t_abs  = (double)(start + m) / 1.92e6;
      double ph     = 2 * M_PI * (f * t_tone + cfo_hz * t_abs);
      x[start + m] += (float)(amp * cos(ph)) + I * (float)(amp * sin(ph));
    }
  }
}

static double pow_db_to_lin(double db)
{
  return pow(10.0, db / 10.0);
}

/* --- checks ---------------------------------------------------------------------------------------------- */

static void test_config_validation(void)
{
  srsran_nprach_cfg_t ok = {.cell_id = 1, .format = 0, .n_rep = 4, .n_sc_offset = 0, .n_sc_nprach = 48};
  CHECK(srsran_nprach_check_cfg(&ok) == SRSRAN_SUCCESS, "valid config rejected");

  srsran_nprach_cfg_t c;
  c = ok; c.format = 2;        CHECK(srsran_nprach_check_cfg(&c) != SRSRAN_SUCCESS, "format 2 accepted");
  c = ok; c.cell_id = 504;     CHECK(srsran_nprach_check_cfg(&c) != SRSRAN_SUCCESS, "cell id 504 accepted");
  c = ok; c.n_rep = 3;         CHECK(srsran_nprach_check_cfg(&c) != SRSRAN_SUCCESS, "3 repetitions accepted");
  c = ok; c.n_rep = 0;         CHECK(srsran_nprach_check_cfg(&c) != SRSRAN_SUCCESS, "0 repetitions accepted");
  c = ok; c.n_rep = 256;       CHECK(srsran_nprach_check_cfg(&c) != SRSRAN_SUCCESS, "256 repetitions accepted");
  c = ok; c.n_sc_nprach = 18;  CHECK(srsran_nprach_check_cfg(&c) != SRSRAN_SUCCESS, "18 subcarriers accepted");
  c = ok; c.n_sc_offset = 5;   c.n_sc_nprach = 12;
  CHECK(srsran_nprach_check_cfg(&c) != SRSRAN_SUCCESS, "offset 5 accepted");
  c = ok; c.n_sc_offset = 12;  c.n_sc_nprach = 48;
  CHECK(srsran_nprach_check_cfg(&c) != SRSRAN_SUCCESS, "offset 12 + 48 subcarriers accepted");
  c = ok; c.n_sc_offset = 36;  c.n_sc_nprach = 12;
  CHECK(srsran_nprach_check_cfg(&c) == SRSRAN_SUCCESS, "offset 36 + 12 rejected");
}

static void test_timing(void)
{
  srsran_nprach_cfg_t c = {.cell_id = 1, .format = 0, .n_rep = 4, .n_sc_offset = 0, .n_sc_nprach = 12};
  CHECK(srsran_nprach_cp_len(0) == 128 && srsran_nprach_cp_len(1) == 512, "CP lengths");
  // 2048 Ts + 5 * 8192 Ts = 43008 Ts = 2688 samples at 16 Ts; format 1: 8192 + 40960 = 49152 Ts = 3072 samples
  CHECK(srsran_nprach_group_len(0) == 2688, "format 0 group %d", srsran_nprach_group_len(0));
  CHECK(srsran_nprach_group_len(1) == 3072, "format 1 group %d", srsran_nprach_group_len(1));
  CHECK(srsran_nprach_nof_samples(&c) == 16 * 2688, "4 reps = %d samples", srsran_nprach_nof_samples(&c));

  // 64 repetitions = 256 groups, then a 40 ms gap (76800 samples) before the next
  c.n_rep = 128;
  CHECK(srsran_nprach_group_start(&c, 255) == 255 * 2688, "no gap inside the first 64 reps");
  CHECK(srsran_nprach_group_start(&c, 256) == 256 * 2688 + 76800, "gap after 64 reps: %d",
        srsran_nprach_group_start(&c, 256));
  CHECK(srsran_nprach_nof_samples(&c) == 512 * 2688 + 76800, "128 reps = %d samples", srsran_nprach_nof_samples(&c));
  c.n_rep = 64;
  CHECK(srsran_nprach_nof_samples(&c) == 256 * 2688, "64 reps has no trailing gap");
}

static void test_hop_structure(void)
{
  // Properties that hold for every cell and starting subcarrier, independent of the pseudo random part:
  //  - within a repetition the hops are +-1, +-6, +-1 (i mod 4 = 1,2,3)
  //  - the jump between repetitions is never zero (f(t) advances by 1..11)
  //  - all values are within 0..11
  uint8_t hop[SRSRAN_NPRACH_MAX_GROUPS];
  for (uint32_t cell = 0; cell < 504; cell += 37) {
    for (uint32_t n_init = 0; n_init < 48; n_init += 5) {
      CHECK(srsran_nprach_hops(cell, n_init, 512, hop) == SRSRAN_SUCCESS, "hops failed");
      CHECK(hop[0] == n_init % 12, "cell %d n_init %d: first hop %d", cell, n_init, hop[0]);
      for (uint32_t i = 1; i < 512; i++) {
        CHECK(hop[i] < 12, "hop out of range");
        int d = (int)hop[i] - (int)hop[i - 1];
        if (i % 4 == 1 || i % 4 == 3) {
          CHECK(abs(d) == 1 && ((hop[i - 1] % 2 == 0) == (d == 1)), "cell %d group %d step %d", cell, i, d);
        } else if (i % 4 == 2) {
          CHECK(abs(d) == 6, "cell %d group %d step %d", cell, i, d);
        } else if (i >= 8) {
          // f(t) - f(t-1) is in 1..11, so consecutive repetitions never start on the same subcarrier. (Not
          // checked for i = 4: n~(0) is n_init mod 12 itself, and f(1) can legitimately be 0.)
          CHECK(hop[i] != hop[i - 4], "cell %d group %d: repetitions start on the same subcarrier", cell, i);
        }
      }
    }
  }
  // Different cells hop differently (the sequence is seeded with the cell id)
  uint8_t a[64], b[64];
  srsran_nprach_hops(1, 0, 64, a);
  srsran_nprach_hops(2, 0, 64, b);
  CHECK(memcmp(a, b, 64) != 0, "cells 1 and 2 hop identically");
  // The first repetition carries no pseudo random part
  CHECK(memcmp(a, b, 4) == 0, "first repetition depends on the cell");

  CHECK(srsran_nprach_hops(1, 0, 0, a) != SRSRAN_SUCCESS, "0 groups accepted");
  CHECK(srsran_nprach_hops(1, 0, 1000, a) != SRSRAN_SUCCESS, "1000 groups accepted");
  CHECK(srsran_nprach_hops(600, 0, 8, a) != SRSRAN_SUCCESS, "cell 600 accepted");
}

static void stats(const srsran_nprach_cfg_t* cfg, double snr_db, int trials, double* pd, double* toa_rmse, double* cfo_rmse,
                  double* max_other);

typedef struct {
  srsran_nprach_cfg_t cfg;
  uint32_t            n_init;
  double              tau;
  double              cfo;
} scenario_t;

/** Runs one scenario at the given per-sample SNR (dB); returns detections and fills det. */
static int run(srsran_nprach_t* q, const scenario_t* s, double snr_db, srsran_nprach_det_t* det, cf_t* buf)
{
  uint32_t n = srsran_nprach_nof_samples(&q->cfg) + 600;
  bzero(buf, sizeof(cf_t) * n);
  add_received(&s->cfg, s->n_init, s->tau, s->cfo, 1.0, buf, n);
  if (snr_db < 200) {
    add_noise(buf, n, 1.0 / pow_db_to_lin(snr_db));
  }
  return srsran_nprach_detect(q, buf, det);
}

/** Detection accuracy at a healthy SNR (tone SNR per FFT bin of about 21 dB). */
static void test_detection_accuracy(void)
{
  static const struct {
    uint32_t fmt, n_rep, offset, nsc;
    double   toa_tol, cfo_tol;
  } modes[] = {{0, 1, 0, 12, 3.0, 15.0}, {0, 2, 0, 48, 2.0, 10.0}, {0, 4, 0, 48, 1.0, 6.0}, {0, 8, 12, 24, 1.0, 6.0},
               {1, 2, 0, 12, 2.0, 10.0}, {1, 4, 36, 12, 1.0, 6.0}, {0, 4, 34, 12, 1.0, 6.0}, {0, 4, 2, 12, 1.0, 6.0}};
  for (unsigned mi = 0; mi < sizeof(modes) / sizeof(modes[0]); mi++) {
    srsran_nprach_cfg_t cfg = {.cell_id = 7 + mi * 61,
                               .format = modes[mi].fmt,
                               .n_rep = modes[mi].n_rep,
                               .n_sc_offset = modes[mi].offset,
                               .n_sc_nprach = modes[mi].nsc};
    srsran_nprach_t q;
    CHECK(srsran_nprach_init(&q, &cfg) == SRSRAN_SUCCESS, "init failed");
    cf_t* buf = srsran_vec_cf_malloc(srsran_nprach_nof_samples(&cfg) + 600);
    srsran_nprach_det_t det[SRSRAN_NPRACH_MAX_SC];

    uint32_t cp = srsran_nprach_cp_len(cfg.format);
    for (uint32_t n_init = 0; n_init < cfg.n_sc_nprach; n_init += (cfg.n_sc_nprach > 12 ? 5 : 1)) {
      double tau_max = cp < 400 ? cp - 2.0 : 480.0;
      scenario_t sc  = {cfg, n_init, 1.0 + (tau_max - 1.0) * ((n_init * 7) % 10) / 10.0 + 0.37,
                       -300.0 + 47.0 * (n_init % 13)};
      int nd = run(&q, &sc, -6.0, det, buf);
      CHECK(nd == 1, "fmt %d rep %d n_init %d: %d detections", cfg.format, cfg.n_rep, n_init, nd);
      CHECK(det[n_init].detected, "fmt %d rep %d n_init %d: not detected (metric %.1f)", cfg.format, cfg.n_rep, n_init,
            det[n_init].metric);
      CHECK(fabs(det[n_init].toa - sc.tau) < modes[mi].toa_tol, "fmt %d rep %d n_init %d: ToA %.2f expected %.2f",
            cfg.format, cfg.n_rep, n_init, det[n_init].toa, sc.tau);
      CHECK(fabs(det[n_init].cfo_hz - sc.cfo) < modes[mi].cfo_tol, "fmt %d rep %d n_init %d: CFO %.1f expected %.1f",
            cfg.format, cfg.n_rep, n_init, det[n_init].cfo_hz, sc.cfo);
    }
    free(buf);
    srsran_nprach_free(&q);
  }
}

/** Format 1 covers delays up to 256 us; check the whole range including its far end. */
static void test_format1_range(void)
{
  srsran_nprach_cfg_t cfg = {.cell_id = 20, .format = 1, .n_rep = 4, .n_sc_offset = 0, .n_sc_nprach = 12};
  srsran_nprach_t     q;
  CHECK(srsran_nprach_init(&q, &cfg) == SRSRAN_SUCCESS, "init");
  cf_t* buf = srsran_vec_cf_malloc(srsran_nprach_nof_samples(&cfg) + 600);
  srsran_nprach_det_t det[12];
  static const double taus[] = {0.0, 0.5, 60.0, 130.25, 255.5, 380.75, 450.0, 480.0, 494.0};
  for (unsigned i = 0; i < sizeof(taus) / sizeof(taus[0]); i++) {
    scenario_t sc = {cfg, (i * 5) % 12, taus[i], 120.0 - 30.0 * i};
    int        nd = run(&q, &sc, -15.0, det, buf);
    CHECK(nd == 1 && det[sc.n_init].detected, "tau %.2f: %d detections", taus[i], nd);
    CHECK(fabs(det[sc.n_init].toa - taus[i]) < 1.5, "tau %.2f estimated as %.2f", taus[i], det[sc.n_init].toa);
  }
  free(buf);
  srsran_nprach_free(&q);
}

/** The default threshold against the noise-only tail measured with -E (level exceeded with probability 1e-6, larger of
 *  the two formats): it must not be below it (false alarms) and not far above it (lost sensitivity). */
static void test_threshold_table(void)
{
  static const struct {
    uint32_t n_rep;
    float    tail;
  } table[] = {{1, 24.5f}, {2, 13.9f}, {4, 8.4f}, {8, 5.3f}, {16, 3.6f}, {32, 2.65f}};
  for (unsigned i = 0; i < sizeof(table) / sizeof(table[0]); i++) {
    float th = srsran_nprach_default_threshold(table[i].n_rep);
    CHECK(th >= table[i].tail, "n_rep %u: threshold %.2f below the measured 1e-6 tail %.2f", table[i].n_rep, th,
          table[i].tail);
    CHECK(th <= 1.3f * table[i].tail, "n_rep %u: threshold %.2f more than 30%% above the tail %.2f", table[i].n_rep,
          th, table[i].tail);
  }
  // Beyond 32 repetitions the tail was only measured to about 1e-3 (largest noise-only metric seen: 1.74 at 64, 1.51 at
  // 128 repetitions), so the threshold is held at 1.8 rather than following the formula further down.
  CHECK(srsran_nprach_default_threshold(64) >= 1.8f, "64 repetitions: %.2f", srsran_nprach_default_threshold(64));
  CHECK(srsran_nprach_default_threshold(128) >= 1.8f, "128 repetitions: %.2f", srsran_nprach_default_threshold(128));
  CHECK(srsran_nprach_default_threshold(0) == srsran_nprach_default_threshold(1), "0 repetitions not treated as 1");
  float prev = 1e9f;
  for (uint32_t n = 1; n <= 128; n++) {
    float th = srsran_nprach_default_threshold(n);
    CHECK(th <= prev, "threshold rises from %.2f to %.2f at %u repetitions", prev, th, n);
    prev = th;
  }
}

/** Noise only: no detections (the thresholds target 1e-6 per starting subcarrier). */
static void test_false_alarms(void)
{
  static const struct {
    uint32_t fmt, n_rep, trials;
  } cases[] = {{0, 1, 1000}, {0, 2, 1000}, {1, 2, 500}, {0, 4, 600}, {1, 4, 400}, {0, 8, 200}};
  for (unsigned c = 0; c < sizeof(cases) / sizeof(cases[0]); c++) {
    srsran_nprach_cfg_t cfg = {.cell_id = 33, .format = cases[c].fmt, .n_rep = cases[c].n_rep, .n_sc_offset = 0,
                               .n_sc_nprach = 12};
    srsran_nprach_t     q;
    CHECK(srsran_nprach_init(&q, &cfg) == SRSRAN_SUCCESS, "init");
    uint32_t n   = srsran_nprach_nof_samples(&cfg) + 600;
    cf_t*    buf = cf_zalloc(n);
    srsran_nprach_det_t det[12];
    int                 alarms = 0;
    for (uint32_t t = 0; t < cases[c].trials; t++) {
      bzero(buf, sizeof(cf_t) * n);
      add_noise(buf, n, 1.0);
      alarms += srsran_nprach_detect(&q, buf, det);
    }
    CHECK(alarms == 0, "fmt %d rep %d: %d false alarms in %d noise-only occasions", cfg.format, cfg.n_rep, alarms,
          cases[c].trials);
    free(buf);
    srsran_nprach_free(&q);
  }
}

/** Detection probability close to the sensitivity limit. Noise is over the whole 1.92 MHz, so -24 dB per sample is
 *  about -13.5 dB in the 180 kHz NB-IoT carrier. */
static void test_sensitivity(void)
{
  static const struct {
    uint32_t fmt, n_rep;
    double   snr;
    int      trials;
    double   pd_min;
  } cases[] = {{0, 1, -21.0, 40, 0.95}, {0, 4, -24.0, 40, 0.95}, {1, 4, -24.0, 40, 0.95}, {0, 16, -30.0, 20, 0.90}};
  for (unsigned c = 0; c < sizeof(cases) / sizeof(cases[0]); c++) {
    srsran_nprach_cfg_t cfg = {.cell_id = 44, .format = cases[c].fmt, .n_rep = cases[c].n_rep, .n_sc_offset = 0,
                               .n_sc_nprach = 12};
    seed(1000 + c);
    double pd, tr, cr, mo;
    stats(&cfg, cases[c].snr, cases[c].trials, &pd, &tr, &cr, &mo);
    CHECK(pd >= cases[c].pd_min, "fmt %d rep %d at %.0f dB: Pd %.2f < %.2f", cfg.format, cfg.n_rep, cases[c].snr, pd,
          cases[c].pd_min);
  }
}

static void test_wrong_cell(void)
{
  // The hopping sequence is seeded with the cell id, but the FIRST repetition is cell independent, so a
  // preamble of the wrong cell still matches 1/n_rep of its energy. At high SNR that is enough to be detected; near
  // the sensitivity limit (-30 dB per sample, 8 repetitions) it must score far below the same preamble seen by the
  // right cell and stay under the detection threshold.
  srsran_nprach_cfg_t tx = {.cell_id = 5, .format = 0, .n_rep = 8, .n_sc_offset = 0, .n_sc_nprach = 12};
  srsran_nprach_cfg_t bad = tx;
  bad.cell_id             = 6;
  srsran_nprach_t good_q, bad_q;
  CHECK(srsran_nprach_init(&good_q, &tx) == SRSRAN_SUCCESS, "init");
  CHECK(srsran_nprach_init(&bad_q, &bad) == SRSRAN_SUCCESS, "init");
  cf_t* buf = srsran_vec_cf_malloc(srsran_nprach_nof_samples(&tx) + 600);
  srsran_nprach_det_t dg[12], db[12];
  scenario_t          sc = {tx, 3, 40.0, 0.0};
  seed(777);
  int ng = run(&good_q, &sc, -30.0, dg, buf);
  int nb = srsran_nprach_detect(&bad_q, buf, db); // same samples
  CHECK(ng == 1 && dg[3].detected, "right cell did not detect (%d detections, metric %.1f)", ng, dg[3].metric);
  // (Other starting subcarriers can score by chance alignment with later repetitions; what matters is that this
  // preamble is not recognised by the wrong cell and scores far below what the right cell gives it.)
  (void)nb;
  CHECK(!db[3].detected, "preamble of another cell was detected as n_init 3 (metric %.1f, threshold %.2f)", db[3].metric,
        bad_q.threshold);
  CHECK(db[3].metric < 0.5 * dg[3].metric, "wrong cell metric %.1f vs right cell %.1f", db[3].metric, dg[3].metric);
  free(buf);
  srsran_nprach_free(&good_q);
  srsran_nprach_free(&bad_q);
}

static void test_two_users(void)
{
  // Two UEs on different starting subcarriers of the same 12-subcarrier block, different delays and CFOs
  srsran_nprach_cfg_t cfg = {.cell_id = 3, .format = 0, .n_rep = 4, .n_sc_offset = 0, .n_sc_nprach = 12};
  srsran_nprach_t     q;
  CHECK(srsran_nprach_init(&q, &cfg) == SRSRAN_SUCCESS, "init");
  uint32_t n   = srsran_nprach_nof_samples(&cfg) + 600;
  cf_t*    buf = cf_zalloc(n);
  add_received(&cfg, 2, 20.5, 100.0, 1.0, buf, n);
  add_received(&cfg, 9, 90.25, -150.0, 0.7, buf, n);
  srsran_nprach_det_t det[12];
  int                 nd = srsran_nprach_detect(&q, buf, det);
  CHECK(nd == 2, "expected 2 detections, got %d", nd);
  CHECK(det[2].detected && fabs(det[2].toa - 20.5) < 0.5 && fabs(det[2].cfo_hz - 100.0) < 15.0, "user 2: toa %.2f cfo %.1f",
        det[2].toa, det[2].cfo_hz);
  CHECK(det[9].detected && fabs(det[9].toa - 90.25) < 0.5 && fabs(det[9].cfo_hz + 150.0) < 15.0, "user 9: toa %.2f cfo %.1f",
        det[9].toa, det[9].cfo_hz);
  free(buf);
  srsran_nprach_free(&q);
}

static void test_input_validation(void)
{
  srsran_nprach_t     q;
  srsran_nprach_cfg_t bad = {.cell_id = 1, .format = 0, .n_rep = 5, .n_sc_offset = 0, .n_sc_nprach = 12};
  CHECK(srsran_nprach_init(&q, &bad) != SRSRAN_SUCCESS, "init with bad config succeeded");
  CHECK(srsran_nprach_gen(&bad, 0, NULL) != SRSRAN_SUCCESS, "gen with bad config succeeded");
  srsran_nprach_cfg_t ok = {.cell_id = 1, .format = 0, .n_rep = 1, .n_sc_offset = 0, .n_sc_nprach = 12};
  cf_t                buf[2688 * 4];
  CHECK(srsran_nprach_gen(&ok, 12, buf) != SRSRAN_SUCCESS, "n_init 12 of 12 subcarriers accepted");
  CHECK(srsran_nprach_gen(&ok, 11, buf) == SRSRAN_SUCCESS, "n_init 11 rejected");
}

/* --- statistics ------------------------------------------------------------------------------------------ */

/** Detection probability, ToA rmse and CFO rmse at one operating point (target n_init). */
static void stats(const srsran_nprach_cfg_t* cfg, double snr_db, int trials, double* pd, double* toa_rmse, double* cfo_rmse,
                  double* max_other)
{
  srsran_nprach_t q;
  srsran_nprach_init(&q, cfg);
  uint32_t n   = srsran_nprach_nof_samples(cfg) + 600;
  cf_t*    buf = srsran_vec_cf_malloc(n);
  srsran_nprach_det_t det[SRSRAN_NPRACH_MAX_SC];
  int    hit = 0;
  double st = 0, sc = 0, mo = 0;
  uint32_t cp = srsran_nprach_cp_len(cfg->format);
  for (int t = 0; t < trials; t++) {
    scenario_t s = {*cfg, (uint32_t)(rnd() * cfg->n_sc_nprach) % cfg->n_sc_nprach, rnd() * (cp < 400 ? cp - 4 : 480) + 2,
                    (rnd() - 0.5) * 600.0};
    run(&q, &s, snr_db, det, buf);
    for (uint32_t i = 0; i < cfg->n_sc_nprach; i++) {
      if (i != s.n_init && det[i].metric > mo) {
        mo = det[i].metric;
      }
    }
    if (det[s.n_init].detected) {
      hit++;
      st += (det[s.n_init].toa - s.tau) * (det[s.n_init].toa - s.tau);
      sc += (det[s.n_init].cfo_hz - s.cfo) * (det[s.n_init].cfo_hz - s.cfo);
    }
  }
  *pd       = (double)hit / trials;
  *toa_rmse = hit ? sqrt(st / hit) : NAN;
  *cfo_rmse = hit ? sqrt(sc / hit) : NAN;
  *max_other = mo;
  free(buf);
  srsran_nprach_free(&q);
}

static int cmp_double(const void* a, const void* b)
{
  double x = *(const double*)a, y = *(const double*)b;
  return x < y ? -1 : (x > y);
}

/** Noise-only peak metric for one configuration (used to check the thresholds for large repetition counts). */
static int noise_only(unsigned fmt, unsigned nrep, unsigned trials)
{
  srsran_nprach_cfg_t cfg = {.cell_id = 11, .format = fmt, .n_rep = nrep, .n_sc_offset = 0, .n_sc_nprach = 12};
  srsran_nprach_t     q;
  if (srsran_nprach_init(&q, &cfg)) {
    return 1;
  }
  uint32_t n   = srsran_nprach_nof_samples(&cfg) + 600;
  cf_t*    buf = cf_zalloc(n);
  srsran_nprach_det_t det[12];
  double  mx = 0, sum = 0;
  int     over = 0;
  for (unsigned t = 0; t < trials; t++) {
    bzero(buf, sizeof(cf_t) * n);
    add_noise(buf, n, 1.0);
    srsran_nprach_detect(&q, buf, det);
    for (int i = 0; i < 12; i++) {
      mx = det[i].metric > mx ? det[i].metric : mx;
      sum += det[i].metric;
      over += det[i].metric >= q.threshold;
    }
  }
  printf("fmt %u nrep %3u: %u hypothesis trials, mean %.2f max %.2f, threshold %.2f, %d above\n", fmt, nrep, trials * 12,
         sum / (trials * 12), mx, q.threshold, over);
  free(buf);
  srsran_nprach_free(&q);
  return 0;
}

static int explore(void)
{
  printf("# per-sample SNR is over the full 1.92 MHz sample rate\n");
  for (int fmt = 0; fmt <= 1; fmt++) {
    for (int nrep = 1; nrep <= 32; nrep *= 2) {
      srsran_nprach_cfg_t cfg = {.cell_id = 11, .format = fmt, .n_rep = nrep, .n_sc_offset = 0, .n_sc_nprach = 12};
      // noise only: distribution of the peak-to-mean metric
      srsran_nprach_t q;
      srsran_nprach_init(&q, &cfg);
      uint32_t n   = srsran_nprach_nof_samples(&cfg) + 600;
      cf_t*    buf = cf_zalloc(n);
      srsran_nprach_det_t det[12];
      int trials = nrep >= 16 ? 150 : (nrep >= 8 ? 500 : 1000);
      double* m = malloc(sizeof(double) * trials * 12);
      for (int t = 0; t < trials; t++) {
        bzero(buf, sizeof(cf_t) * n);
        add_noise(buf, n, 1.0);
        srsran_nprach_detect(&q, buf, det);
        for (int i = 0; i < 12; i++) {
          m[t * 12 + i] = det[i].metric;
        }
      }
      int cnt = trials * 12;
      qsort(m, cnt, sizeof(double), cmp_double);
      // exponential tail fit on the top 5 % and extrapolation to 1e-6 per hypothesis
      int    k  = cnt / 20;
      double u  = m[cnt - k];
      double ex = 0;
      for (int i = cnt - k; i < cnt; i++) {
        ex += m[i] - u;
      }
      ex /= k;
      printf("fmt %d nrep %2d noise-only metric: median %.2f  p99 %.2f  p99.9 %.2f  max %.2f  (%d samples)  tail fit -> "
             "1e-6 at %.2f  (1e-4 at %.2f)\n",
             fmt, nrep, m[cnt / 2], m[(int)(cnt * 0.99)], m[(int)(cnt * 0.999)], m[cnt - 1], cnt,
             u + ex * log(0.05 / 1e-6), u + ex * log(0.05 / 1e-4));
      free(m); free(buf);
      srsran_nprach_free(&q);

      for (double snr = -30; snr <= -6; snr += 3) {
        double pd, tr, cr, mo;
        stats(&cfg, snr, nrep >= 16 ? 30 : 60, &pd, &tr, &cr, &mo);
        printf("    snr %5.1f dB: Pd %.2f  toa rmse %.2f  cfo rmse %.1f Hz  max other metric %.1f\n", snr, pd, tr, cr, mo);
        fflush(stdout);
      }
    }
  }
  return 0;
}

int main(int argc, char** argv)
{
  if (argc >= 3 && !strcmp(argv[1], "-H")) {
    unsigned cell, n_init, groups;
    if (sscanf(argv[2], "%u,%u,%u", &cell, &n_init, &groups) != 3) {
      return 2;
    }
    uint8_t hop[SRSRAN_NPRACH_MAX_GROUPS];
    if (srsran_nprach_hops(cell, n_init, groups, hop)) {
      return 1;
    }
    for (unsigned i = 0; i < groups; i++) {
      printf("%d%c", hop[i], i + 1 < groups ? ' ' : '\n');
    }
    return 0;
  }
  if (argc >= 3 && !strcmp(argv[1], "-W")) {
    char     file[512];
    unsigned cell, fmt, nrep, off, nsc, n_init;
    if (sscanf(argv[2], "%511[^,],%u,%u,%u,%u,%u,%u", file, &cell, &fmt, &nrep, &off, &nsc, &n_init) != 7) {
      return 2;
    }
    srsran_nprach_cfg_t cfg = {cell, fmt, nrep, off, nsc};
    uint32_t            n   = srsran_nprach_nof_samples(&cfg);
    cf_t*               x   = srsran_vec_cf_malloc(n);
    if (srsran_nprach_gen(&cfg, n_init, x)) {
      return 1;
    }
    FILE* f = fopen(file, "wb");
    if (!f) {
      return 1;
    }
    fwrite(x, sizeof(cf_t), n, f);
    fclose(f);
    free(x);
    return 0;
  }
  if (argc >= 3 && !strcmp(argv[1], "-N")) {
    unsigned fmt, nrep, trials;
    if (sscanf(argv[2], "%u,%u,%u", &fmt, &nrep, &trials) != 3) {
      return 2;
    }
    return noise_only(fmt, nrep, trials);
  }
  if (argc >= 2 && !strcmp(argv[1], "-E")) {
    return explore();
  }

  seed(12345);
  test_config_validation();
  test_input_validation();
  test_timing();
  test_hop_structure();
  test_detection_accuracy();
  test_format1_range();
  test_threshold_table();
  test_false_alarms();
  test_sensitivity();
  test_wrong_cell();
  test_two_users();

  printf("nprach_test: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
