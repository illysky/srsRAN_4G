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
 * Tests of the in-band NB-IoT uplink front end (anchor_fe). Every expectation is analytic: a complex tone at
 * (offset + f) must come out as a unit tone at f whose phase is that of the *aligned input sample*, so a wrong mixer
 * sign, a wrong alignment or a wrong absolute-phase reduction all show up as a phase error; tones outside the
 * passband must vanish; an impulse must land on the output sample it is aligned to.
 */

#include "srsran/phy/resampling/anchor_fe.h"
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

typedef struct {
  uint32_t fs;
  int32_t  offset;
  const char* what;
} config_t;

// The anchor offsets are (2 p + 1 - N_PRB) * 90 kHz of real carriers.
static const config_t configs[] = {
    {1920000, 0, "1.92 MS/s (no decimation), centred"},
    {5760000, 900000, "25 PRB, PRB 17"},
    {5760000, -900000, "25 PRB, PRB 7"},
    {7680000, 900000, "25 PRB at 7.68 MS/s, PRB 17"},
    {15360000, -3510000, "50 PRB, PRB 5"},
    {30720000, 5400000, "100 PRB at 30.72 MS/s, offset +5.4 MHz"},
    // Every real anchor offset is a multiple of 90 kHz, for which the mixer period (a power of two) divides the
    // internal chunk length, so a mixer that restarts its phase at a chunk boundary would go unnoticed. 5.76 MS/s and
    // 128 kHz give a period of 45, which does not divide the chunk (2048 * 3 samples).
    {5760000, 128000, "mixer period 45 (not a divisor of the chunk length)"},
};
#define NCONF (sizeof(configs) / sizeof(configs[0]))

/// exp(j 2 pi f n / fs) for integer f, n with the phase reduced exactly in integers.
static cf_t expj(int64_t f, uint64_t n, uint32_t fs)
{
  int64_t num = (int64_t)(((__int128)f * (__int128)n) % (__int128)fs);
  double  ph  = 2.0 * M_PI * (double)num / (double)fs;
  return (float)cos(ph) + _Complex_I * (float)sin(ph);
}

static void test_init_rejections(void)
{
  srsran_anchor_fe_t q;
  CHECK(srsran_anchor_fe_init(&q, 5000000, 0) != 0, "5 MS/s is not a multiple of 1.92 MS/s but was accepted");
  CHECK(srsran_anchor_fe_init(&q, 0, 0) != 0, "0 Hz accepted");
  CHECK(srsran_anchor_fe_init(&q, 32640000, 0) != 0, "decimation by 17 accepted");
  CHECK(srsran_anchor_fe_init(&q, 5760000, 2700000) != 0, "anchor 2.7 MHz off-centre leaves no room in a 5.76 MHz band");
  CHECK(srsran_anchor_fe_init(&q, 5760000, -2700000) != 0, "negative anchor 2.7 MHz off-centre accepted");
  CHECK(srsran_anchor_fe_init(NULL, 5760000, 0) != 0, "NULL accepted");
  CHECK(srsran_anchor_fe_init(&q, 5760000, 2580000) == 0, "largest legal offset refused");
  srsran_anchor_fe_free(&q);
}

/// The response to a tone inside the carrier: unit amplitude, and the phase of the aligned input sample.
static void test_passband_tone(const config_t* c)
{
  srsran_anchor_fe_t q;
  CHECK(srsran_anchor_fe_init(&q, c->fs, c->offset) == 0, "%s: init", c->what);
  const uint32_t D     = srsran_anchor_fe_delay(&q);
  const uint32_t n_out = 3000; // more than one internal chunk
  const uint32_t n_in  = srsran_anchor_fe_nof_input(&q, n_out);
  CHECK(n_in == 2 * D + (n_out - 1) * (c->fs / 1920000) + 1, "%s: nof_input %u", c->what, n_in);

  cf_t*    x  = malloc(sizeof(cf_t) * n_in);
  cf_t*    y  = malloc(sizeof(cf_t) * n_out);
  uint64_t n0 = 123456789ull; // large absolute index: the mixer phase must be reduced exactly
  for (int f = -100000; f <= 100000; f += 10000) {
    for (uint32_t i = 0; i < n_in; ++i) {
      x[i] = expj((int64_t)c->offset + f, n0 + i, c->fs);
    }
    CHECK(srsran_anchor_fe_run(&q, x, n0, y, n_out) == 0, "%s: run", c->what);
    double worst = 0.0;
    for (uint32_t j = 0; j < n_out; ++j) {
      // aligned to input sample D + j * ratio
      cf_t   expect = expj(f, n0 + D + (uint64_t)j * q.ratio, c->fs);
      double e      = cabsf(y[j] - expect);
      if (e > worst) {
        worst = e;
      }
    }
    CHECK(worst < 3e-3, "%s: tone at anchor%+d Hz deviates by %.4f from the aligned unit tone", c->what, f, worst);
  }
  free(x);
  free(y);
  srsran_anchor_fe_free(&q);
}

/// Anything beyond the stop edge must be gone, including the LTE carrier centre (DC of the input).
static void test_stopband(const config_t* c)
{
  srsran_anchor_fe_t q;
  CHECK(srsran_anchor_fe_init(&q, c->fs, c->offset) == 0, "%s: init", c->what);
  const uint32_t n_out = 1500;
  const uint32_t n_in  = srsran_anchor_fe_nof_input(&q, n_out);
  cf_t*          x     = malloc(sizeof(cf_t) * n_in);
  cf_t*          y     = malloc(sizeof(cf_t) * n_out);

  // relative to the anchor, in kHz: from the stop edge up to the far side of the input band, and the input DC
  int      rel[64];
  uint32_t nrel = 0;
  for (int f = 300; f <= 2000; f += (f < 1000 ? 100 : 250)) {
    rel[nrel++] = f * 1000;
    rel[nrel++] = -f * 1000;
  }
  rel[nrel++] = -c->offset; // the LTE carrier centre
  rel[nrel++] = -c->offset + 15000;

  uint32_t tested = 0;
  double   worst  = 0.0;
  for (uint32_t k = 0; k < nrel; ++k) {
    int64_t fin = (int64_t)c->offset + rel[k];
    if (rel[k] > -300000 && rel[k] < 300000) {
      continue; // inside the transition band (only reachable through -offset)
    }
    if (fin >= (int64_t)c->fs / 2 || fin <= -(int64_t)c->fs / 2) {
      continue; // outside the input band
    }
    for (uint32_t i = 0; i < n_in; ++i) {
      x[i] = expj(fin, i, c->fs);
    }
    srsran_anchor_fe_run(&q, x, 0, y, n_out);
    double p = 0.0;
    for (uint32_t j = 0; j < n_out; ++j) {
      p += crealf(y[j]) * crealf(y[j]) + cimagf(y[j]) * cimagf(y[j]);
    }
    double amp = sqrt(p / n_out);
    if (amp > worst) {
      worst = amp;
    }
    ++tested;
    CHECK(amp < 1e-3, "%s: tone at anchor%+d Hz leaks through at %.1f dB", c->what, rel[k], 20 * log10(amp + 1e-30));
  }
  CHECK(tested >= 4 || c->fs == 1920000, "%s: only %u stopband tones tested", c->what, tested);
  printf("  %-44s stopband: %2u tones, worst %.1f dB\n", c->what, tested, worst > 0 ? 20 * log10(worst) : -999.0);
  free(x);
  free(y);
  srsran_anchor_fe_free(&q);
}

/// An impulse aligned to input sample D + j0 * ratio must peak at output j0, symmetrically.
static void test_impulse_alignment(const config_t* c)
{
  srsran_anchor_fe_t q;
  CHECK(srsran_anchor_fe_init(&q, c->fs, c->offset) == 0, "%s: init", c->what);
  const uint32_t D = srsran_anchor_fe_delay(&q);
  if (D < q.ratio) {
    srsran_anchor_fe_free(&q); // ratio 1 / very short filters: nothing more to say
    return;
  }
  const uint32_t n_out = 200;
  const uint32_t n_in  = srsran_anchor_fe_nof_input(&q, n_out);
  cf_t*          x     = calloc(n_in, sizeof(cf_t));
  cf_t*          y     = malloc(sizeof(cf_t) * n_out);
  const uint32_t j0    = 100;
  x[D + j0 * q.ratio]  = 1.0f;
  srsran_anchor_fe_run(&q, x, 0, y, n_out);

  uint32_t peak = 0;
  for (uint32_t j = 1; j < n_out; ++j) {
    if (cabsf(y[j]) > cabsf(y[peak])) {
      peak = j;
    }
  }
  CHECK(peak == j0, "%s: impulse aligned to output %u peaked at %u", c->what, j0, peak);
  CHECK(fabsf(cabsf(y[j0]) - q.taps[0]) < 1e-6, "%s: peak %.6f != centre tap %.6f", c->what, cabsf(y[j0]), q.taps[0]);
  for (uint32_t k = 1; k * q.ratio <= D && j0 + k < n_out; ++k) {
    CHECK(fabsf(cabsf(y[j0 + k]) - cabsf(y[j0 - k])) < 1e-6,
          "%s: impulse response not symmetric at +-%u (%.6f vs %.6f)",
          c->what,
          k,
          cabsf(y[j0 + k]),
          cabsf(y[j0 - k]));
  }
  // unity gain at DC: sum of the impulse response over the whole filter
  double dc = q.taps[0];
  for (uint32_t m = 1; m <= D; ++m) {
    dc += 2.0 * q.taps[m];
  }
  CHECK(fabs(dc - 1.0) < 1e-5, "%s: DC gain %.6f", c->what, dc);
  free(x);
  free(y);
  srsran_anchor_fe_free(&q);
}

/// Splitting one long run into several calls (with the matching n0) must give bit-identical output, chunk seams and all.
static void test_seams(const config_t* c)
{
  srsran_anchor_fe_t q;
  CHECK(srsran_anchor_fe_init(&q, c->fs, c->offset) == 0, "%s: init", c->what);
  const uint32_t D     = srsran_anchor_fe_delay(&q);
  const uint32_t n_all = 5000;
  const uint32_t n_a   = 3000;
  const uint32_t n_in  = srsran_anchor_fe_nof_input(&q, n_all);
  cf_t*          x     = malloc(sizeof(cf_t) * n_in);
  uint32_t       s     = 12345;
  for (uint32_t i = 0; i < n_in; ++i) {
    s    = s * 1664525u + 1013904223u;
    float a = (float)(s >> 8) / (float)(1 << 24) - 0.5f;
    s    = s * 1664525u + 1013904223u;
    float b = (float)(s >> 8) / (float)(1 << 24) - 0.5f;
    x[i]    = a + _Complex_I * b;
  }
  cf_t* yall = malloc(sizeof(cf_t) * n_all);
  cf_t* ysp  = malloc(sizeof(cf_t) * n_all);
  const uint64_t n0 = 777777;
  srsran_anchor_fe_run(&q, x, n0, yall, n_all);
  srsran_anchor_fe_run(&q, x, n0, ysp, n_a);
  // second part: its first output is aligned to input index D + n_a * ratio, so it starts n_a * ratio samples further
  srsran_anchor_fe_run(&q, x + (uint64_t)n_a * q.ratio, n0 + (uint64_t)n_a * q.ratio, ysp + n_a, n_all - n_a);
  double worst = 0.0;
  for (uint32_t j = 0; j < n_all; ++j) {
    double e = cabsf(yall[j] - ysp[j]);
    if (e > worst) {
      worst = e;
    }
  }
  CHECK(worst == 0.0, "%s: split run differs from the single run by %g", c->what, worst);
  (void)D;
  free(x);
  free(yall);
  free(ysp);
  srsran_anchor_fe_free(&q);
}

/// The run must not read outside the samples srsran_anchor_fe_nof_input() promises.
static void test_read_extent(void)
{
  srsran_anchor_fe_t q;
  CHECK(srsran_anchor_fe_init(&q, 5760000, 900000) == 0, "init");
  const uint32_t n_out = 300;
  const uint32_t n_in  = srsran_anchor_fe_nof_input(&q, n_out);
  const uint32_t guard = 256;
  cf_t*          buf   = malloc(sizeof(cf_t) * (n_in + 2 * guard));
  for (uint32_t i = 0; i < n_in + 2 * guard; ++i) {
    buf[i] = (i < guard || i >= guard + n_in) ? NAN : 1.0f;
  }
  cf_t* y = malloc(sizeof(cf_t) * n_out);
  srsran_anchor_fe_run(&q, buf + guard, 0, y, n_out);
  bool ok = true;
  for (uint32_t j = 0; j < n_out; ++j) {
    if (!isfinite(crealf(y[j])) || !isfinite(cimagf(y[j]))) {
      ok = false;
    }
  }
  CHECK(ok, "output depends on samples outside the promised input range");
  free(buf);
  free(y);
  srsran_anchor_fe_free(&q);
}

int main(void)
{
  printf("anchor_fe_test\n");
  test_init_rejections();
  test_read_extent();
  for (unsigned i = 0; i < NCONF; ++i) {
    test_passband_tone(&configs[i]);
    test_stopband(&configs[i]);
    test_impulse_alignment(&configs[i]);
    test_seams(&configs[i]);
  }
  printf("%d checks, %d failed\n", g_checks, g_fail);
  return g_fail == 0 ? 0 : 1;
}
