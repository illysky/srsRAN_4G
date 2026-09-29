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
 * End to end: LTE-rate uplink samples -> anchor front end -> NPRACH detector.
 *
 * The received signal is synthesised *directly at the LTE sample rate* from the tone definition of TS 36.211
 * 10.1.6.2 (each symbol group is a single tone at (k - 24 + 1/2) * 3.75 kHz from the NB-IoT carrier centre, the
 * carrier centre sitting at 'offset' from the radio centre), so neither an interpolation filter nor the front end's
 * own filter is involved in producing the expected answer. A preamble that arrives tau_in LTE-rate samples late must
 * be reported by the detector as tau_in / ratio samples at 1.92 MS/s: any filter-delay bias in the front end shows up
 * as a ToA error.
 */

#include "srsran/phy/phch/nprach.h"
#include "srsran/phy/resampling/anchor_fe.h"
#include "srsran/phy/utils/vector.h"
#include <complex.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_fail   = 0;
static int g_checks = 0;

#define CHECK(cond, ...)                                                                                               \
  do {                                                                                                                 \
    ++g_checks;                                                                                                        \
    if (!(cond)) {                                                                                                     \
      ++g_fail;                                                                                                        \
      printf("FAIL %s:%d: ", __FILE__, __LINE__);                                                                      \
      printf(__VA_ARGS__);                                                                                             \
      printf("\n");                                                                                                    \
    }                                                                                                                  \
  } while (0)

static uint64_t rng_state = 88172645463325252ull;
static double   rnd(void)
{
  rng_state ^= rng_state << 13;
  rng_state ^= rng_state >> 7;
  rng_state ^= rng_state << 17;
  return ((double)(rng_state >> 11) + 0.5) / 9007199254740992.0;
}

static void add_noise(cf_t* x, uint32_t n, double sigma2)
{
  double s = sqrt(sigma2 / 2.0);
  for (uint32_t i = 0; i < n; i++) {
    double u = rnd(), v = rnd();
    double r = sqrt(-2.0 * log(u));
    x[i] += (float)(s * r * cos(2 * M_PI * v)) + _Complex_I * (float)(s * r * sin(2 * M_PI * v));
  }
}

/// Preamble n_init as received at LTE rate. Nominal start = input sample s0; it arrives tau_in samples later.
static void synth(const srsran_nprach_cfg_t* cfg,
                  uint32_t                   n_init,
                  uint32_t                   fs,
                  int32_t                    offset,
                  uint32_t                   s0,
                  uint32_t                   tau_in,
                  double                     cfo_hz,
                  cf_t*                      x,
                  uint32_t                   x_len)
{
  const uint32_t ratio    = fs / SRSRAN_NPRACH_SRATE_HZ;
  const uint32_t n_groups = 4 * cfg->n_rep;
  uint8_t        hop[SRSRAN_NPRACH_MAX_GROUPS];
  srsran_nprach_hops(cfg->cell_id, n_init, n_groups, hop);
  const uint32_t cp   = srsran_nprach_cp_len(cfg->format);
  const uint32_t glen = srsran_nprach_group_len(cfg->format);
  const uint32_t base = cfg->n_sc_offset + (n_init / 12) * 12;
  for (uint32_t g = 0; g < n_groups; g++) {
    const double   f     = ((double)(base + hop[g]) - 24.0 + 0.5) * 3750.0;
    const uint64_t start = (uint64_t)s0 + (uint64_t)ratio * srsran_nprach_group_start(cfg, g) + tau_in;
    for (uint32_t m = 0; m < glen * ratio && start + m < x_len; m++) {
      const uint64_t idx    = start + m;
      const double   t_tone = ((double)m - (double)cp * ratio) / (double)fs;
      const double   t_abs  = (double)idx / (double)fs;
      // the carrier sits at 'offset' from the radio centre; reduce that phase exactly
      const int64_t mixnum = (int64_t)(((__int128)offset * (__int128)idx) % (__int128)fs);
      const double  ph     = 2.0 * M_PI * (f * t_tone + cfo_hz * t_abs + (double)mixnum / (double)fs);
      x[idx] += (float)cos(ph) + _Complex_I * (float)sin(ph);
    }
  }
}

typedef struct {
  uint32_t fs;
  int32_t  offset;
  const char* what;
} config_t;

static const config_t configs[] = {
    {5760000, 900000, "25 PRB @ 5.76 MS/s, anchor PRB 17 (+900 kHz)"},
    {7680000, -900000, "25 PRB @ 7.68 MS/s, anchor PRB 7 (-900 kHz)"},
    {15360000, -3510000, "50 PRB @ 15.36 MS/s, anchor PRB 5 (-3510 kHz)"},
};
#define NCONF (sizeof(configs) / sizeof(configs[0]))

/// The live cell's NPRACH configuration (SIB2-NB): cell 1, format 0, 16 repetitions, offset 18, 24 subcarriers.
static const srsran_nprach_cfg_t cell_cfg = {.cell_id = 1, .format = 0, .n_rep = 16, .n_sc_offset = 18, .n_sc_nprach = 24};

/// Runs front end + detector on one received buffer. 'align_shift' moves the front end's alignment by that many output
/// samples (0 = correct).
static int detect(srsran_nprach_t*     q,
                  srsran_anchor_fe_t*  fe,
                  const cf_t*          rx,
                  uint32_t             s0,
                  int                  align_shift,
                  cf_t*                y,
                  uint32_t             n_out,
                  srsran_nprach_det_t* det)
{
  const uint32_t D     = srsran_anchor_fe_delay(fe);
  const int64_t  start = (int64_t)s0 + (int64_t)align_shift * fe->ratio;
  if (srsran_anchor_fe_run(fe, rx + (start - D), (uint64_t)(start - D), y, n_out)) {
    return -1;
  }
  return srsran_nprach_detect(q, y, det);
}

static void test_config(const config_t* c)
{
  const uint32_t ratio = c->fs / SRSRAN_NPRACH_SRATE_HZ;
  srsran_nprach_t q;
  CHECK(srsran_nprach_init(&q, &cell_cfg) == SRSRAN_SUCCESS, "nprach init");
  srsran_anchor_fe_t fe;
  CHECK(srsran_anchor_fe_init(&fe, c->fs, c->offset) == 0, "%s: fe init", c->what);
  srsran_anchor_fe_t fe_wrong;
  CHECK(srsran_anchor_fe_init(&fe_wrong, c->fs, -c->offset) == 0, "%s: fe init (wrong sign)", c->what);

  const uint32_t n_pre = srsran_nprach_nof_samples(&cell_cfg);
  const uint32_t n_out = n_pre + 600;
  const uint32_t s0    = ratio * 2000;
  const uint32_t len   = s0 + ratio * (n_out + 2000);
  cf_t*          rx    = srsran_vec_cf_malloc(len);
  cf_t*          y     = srsran_vec_cf_malloc(n_out);
  srsran_nprach_det_t det[SRSRAN_NPRACH_MAX_SC];

  // ---- accuracy at a healthy SNR: ToA must be tau_in / ratio, with no filter-delay bias
  static const uint32_t n_inits[] = {0, 7, 11, 12, 23};
  double                bias_sum  = 0.0;
  int                   bias_n    = 0;
  double                worst_toa = 0.0;
  for (unsigned t = 0; t < 5; ++t) {
    const uint32_t n_init = n_inits[t];
    const uint32_t tau_in = (t == 0) ? 0 : (t == 1) ? 1 : (t == 2) ? ratio : (t == 3) ? 17 * ratio + 2 : 50 * ratio + 1;
    const double   cfo    = -250.0 + 130.0 * t;
    memset(rx, 0, sizeof(cf_t) * len);
    synth(&cell_cfg, n_init, c->fs, c->offset, s0, tau_in, cfo, rx, len);
    add_noise(rx, len, ratio * pow(10.0, -10.0 / 10.0)); // 10 dB per-sample SNR at 1.92 MS/s equivalent
    int nd = detect(&q, &fe, rx, s0, 0, y, n_out, det);
    CHECK(nd == 1 && det[n_init].detected, "%s: n_init %u tau %u: %d detections, wanted %u", c->what, n_init, tau_in, nd, n_init);
    if (det[n_init].detected) {
      const double want = (double)tau_in / (double)ratio;
      const double err  = det[n_init].toa - want;
      CHECK(fabs(err) < 1.0, "%s: n_init %u: ToA %.2f, wanted %.2f", c->what, n_init, det[n_init].toa, want);
      CHECK(fabs(det[n_init].cfo_hz - cfo) < 6.0, "%s: n_init %u: CFO %.1f, wanted %.1f", c->what, n_init, det[n_init].cfo_hz, cfo);
      bias_sum += err;
      ++bias_n;
      if (fabs(err) > worst_toa) {
        worst_toa = fabs(err);
      }
    }
  }
  CHECK(bias_n == 5 && fabs(bias_sum / bias_n) < 0.35, "%s: mean ToA error %.3f samples", c->what, bias_n ? bias_sum / bias_n : 99.0);
  printf("  %-52s ToA: mean error %+.3f, worst %.3f samples\n", c->what, bias_n ? bias_sum / bias_n : 99.0, worst_toa);

  // ---- calibration: shifting the front end's alignment by exactly one output sample must shift the ToA by one
  {
    const uint32_t n_init = 7, tau_in = 10 * ratio;
    memset(rx, 0, sizeof(cf_t) * len);
    synth(&cell_cfg, n_init, c->fs, c->offset, s0, tau_in, 0.0, rx, len);
    add_noise(rx, len, ratio * pow(10.0, -10.0 / 10.0));
    int nd = detect(&q, &fe, rx, s0, 1, y, n_out, det);
    CHECK(nd == 1 && det[n_init].detected, "%s: shifted alignment: %d detections", c->what, nd);
    if (det[n_init].detected) {
      CHECK(fabs(det[n_init].toa - 9.0) < 1.0, "%s: alignment shifted by one sample gave ToA %.2f, wanted 9", c->what, det[n_init].toa);
    }
  }

  // ---- weak signal: -18 dB per sample, n_rep 16, must still be found
  int found = 0;
  for (unsigned t = 0; t < 4; ++t) {
    const uint32_t n_init = 3 + 5 * t;
    memset(rx, 0, sizeof(cf_t) * len);
    synth(&cell_cfg, n_init, c->fs, c->offset, s0, 4 * ratio + t, 100.0, rx, len);
    add_noise(rx, len, ratio * pow(10.0, 18.0 / 10.0));
    int nd = detect(&q, &fe, rx, s0, 0, y, n_out, det);
    if (nd >= 1 && det[n_init].detected) {
      ++found;
    }
  }
  CHECK(found == 4, "%s: only %d of 4 weak (-18 dB) preambles detected", c->what, found);

  // ---- noise only: no false alarms
  int alarms = 0;
  for (unsigned t = 0; t < 12; ++t) {
    memset(rx, 0, sizeof(cf_t) * len);
    add_noise(rx, len, ratio);
    int nd = detect(&q, &fe, rx, s0, 0, y, n_out, det);
    alarms += nd > 0 ? nd : 0;
  }
  CHECK(alarms == 0, "%s: %d false alarms in 12 noise-only occasions", c->what, alarms);

  // ---- negative control: front end centred on the mirror image of the anchor sees nothing
  int wrong = 0;
  for (unsigned t = 0; t < 4; ++t) {
    memset(rx, 0, sizeof(cf_t) * len);
    synth(&cell_cfg, 7, c->fs, c->offset, s0, 3 * ratio, 0.0, rx, len);
    add_noise(rx, len, ratio * pow(10.0, -10.0 / 10.0));
    int nd = detect(&q, &fe_wrong, rx, s0, 0, y, n_out, det);
    wrong += nd > 0 ? 1 : 0;
  }
  CHECK(wrong == 0, "%s: a front end tuned to the wrong side of the LTE carrier still detected %d of 4", c->what, wrong);

  free(rx);
  free(y);
  srsran_anchor_fe_free(&fe);
  srsran_anchor_fe_free(&fe_wrong);
  srsran_nprach_free(&q);
}

int main(void)
{
  printf("nprach_frontend_test\n");
  for (unsigned i = 0; i < NCONF; ++i) {
    test_config(&configs[i]);
  }
  printf("%d checks, %d failed\n", g_checks, g_fail);
  return g_fail == 0 ? 0 : 1;
}
