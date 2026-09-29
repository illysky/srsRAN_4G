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

#include "srsran/phy/resampling/anchor_fe.h"
#include "srsran/phy/utils/debug.h"
#include <complex.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

static uint64_t gcd_u64(uint64_t a, uint64_t b)
{
  while (b != 0) {
    uint64_t t = a % b;
    a          = b;
    b          = t;
  }
  return a;
}

/// Modified Bessel function of the first kind, order 0 (power series).
static double bessel_i0(double x)
{
  double sum  = 1.0;
  double term = 1.0;
  double q    = x * x / 4.0;
  for (int k = 1; k < 200; ++k) {
    term *= q / ((double)k * (double)k);
    sum += term;
    if (term < 1e-16 * sum) {
      break;
    }
  }
  return sum;
}

int srsran_anchor_fe_init(srsran_anchor_fe_t* q, uint32_t fs_in_hz, int32_t offset_hz)
{
  if (q == NULL) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  memset(q, 0, sizeof(*q));

  if (fs_in_hz == 0 || fs_in_hz % SRSRAN_ANCHOR_FE_OUT_RATE_HZ != 0) {
    ERROR("Anchor FE: input rate %u Hz is not a multiple of %u Hz", fs_in_hz, SRSRAN_ANCHOR_FE_OUT_RATE_HZ);
    return SRSRAN_ERROR;
  }
  uint32_t ratio = fs_in_hz / SRSRAN_ANCHOR_FE_OUT_RATE_HZ;
  if (ratio > SRSRAN_ANCHOR_FE_MAX_RATIO) {
    ERROR("Anchor FE: decimation %u exceeds %u", ratio, SRSRAN_ANCHOR_FE_MAX_RATIO);
    return SRSRAN_ERROR;
  }
  int64_t max_off = (int64_t)fs_in_hz / 2 - (int64_t)SRSRAN_ANCHOR_FE_STOP_HZ;
  if ((int64_t)offset_hz > max_off || (int64_t)offset_hz < -max_off) {
    ERROR("Anchor FE: anchor offset %d Hz is too close to the edge of the %u Hz input band", offset_hz, fs_in_hz);
    return SRSRAN_ERROR;
  }
  q->fs_in_hz  = fs_in_hz;
  q->offset_hz = offset_hz;
  q->ratio     = ratio;

  // Kaiser windowed-sinc low-pass. Cutoff halfway between the pass and stop edges.
  const double a      = SRSRAN_ANCHOR_FE_STOP_ATTEN_DB;
  const double beta   = 0.1102 * (a - 8.7);
  const double dw     = 2.0 * M_PI * (double)(SRSRAN_ANCHOR_FE_STOP_HZ - SRSRAN_ANCHOR_FE_PASS_HZ) / (double)fs_in_hz;
  uint32_t     order  = (uint32_t)ceil((a - 7.95) / (2.285 * dw));
  q->half_len         = (order + 1) / 2;
  const double fc     = 0.5 * (double)(SRSRAN_ANCHOR_FE_PASS_HZ + SRSRAN_ANCHOR_FE_STOP_HZ) / (double)fs_in_hz;
  const double i0beta = bessel_i0(beta);

  q->taps = (float*)calloc(q->half_len + 1, sizeof(float));
  if (q->taps == NULL) {
    srsran_anchor_fe_free(q);
    return SRSRAN_ERROR;
  }
  double* h   = (double*)calloc(q->half_len + 1, sizeof(double));
  double  sum = 0.0;
  if (h == NULL) {
    srsran_anchor_fe_free(q);
    return SRSRAN_ERROR;
  }
  for (uint32_t m = 0; m <= q->half_len; ++m) {
    double r    = (double)m / (double)q->half_len; // Kaiser window argument, 0 at the centre, 1 at the last tap
    double win  = bessel_i0(beta * sqrt(1.0 - r * r)) / i0beta;
    double sinc = m == 0 ? 2.0 * fc : sin(2.0 * M_PI * fc * (double)m) / (M_PI * (double)m);
    h[m]        = sinc * win;
    sum += (m == 0 ? 1.0 : 2.0) * h[m];
  }
  for (uint32_t m = 0; m <= q->half_len; ++m) {
    q->taps[m] = (float)(h[m] / sum); // unity DC gain
  }
  free(h);

  // Mixer: exp(-j 2 pi offset n / fs). Period = fs / gcd(fs, |offset|).
  uint64_t g          = gcd_u64(fs_in_hz, (uint64_t)(offset_hz < 0 ? -(int64_t)offset_hz : offset_hz));
  q->osc_period       = (uint32_t)(fs_in_hz / g);
  q->osc              = (cf_t*)malloc(sizeof(cf_t) * q->osc_period);
  q->mixed            = (cf_t*)malloc(sizeof(cf_t) * (SRSRAN_ANCHOR_FE_CHUNK * ratio + 2 * q->half_len + 1));
  if (q->osc == NULL || q->mixed == NULL) {
    srsran_anchor_fe_free(q);
    return SRSRAN_ERROR;
  }
  for (uint32_t n = 0; n < q->osc_period; ++n) {
    // reduce the phase exactly in integers before converting: (offset * n) mod fs
    int64_t num = ((int64_t)offset_hz * (int64_t)n) % (int64_t)fs_in_hz;
    double  ph  = -2.0 * M_PI * (double)num / (double)fs_in_hz;
    q->osc[n]   = (float)cos(ph) + _Complex_I * (float)sin(ph);
  }
  return SRSRAN_SUCCESS;
}

void srsran_anchor_fe_free(srsran_anchor_fe_t* q)
{
  if (q == NULL) {
    return;
  }
  free(q->taps);
  free(q->osc);
  free(q->mixed);
  memset(q, 0, sizeof(*q));
}

uint32_t srsran_anchor_fe_delay(const srsran_anchor_fe_t* q)
{
  return q->half_len;
}

uint32_t srsran_anchor_fe_nof_input(const srsran_anchor_fe_t* q, uint32_t n_out)
{
  return n_out == 0 ? 0 : 2 * q->half_len + (n_out - 1) * q->ratio + 1;
}

int srsran_anchor_fe_run(srsran_anchor_fe_t* q, const cf_t* x, uint64_t n0, cf_t* y, uint32_t n_out)
{
  if (q == NULL || q->taps == NULL || x == NULL || y == NULL) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  const uint32_t D = q->half_len;

  uint32_t done = 0;
  while (done < n_out) {
    uint32_t n_chunk = n_out - done;
    if (n_chunk > SRSRAN_ANCHOR_FE_CHUNK) {
      n_chunk = SRSRAN_ANCHOR_FE_CHUNK;
    }
    // input range of this chunk: [done * ratio, done * ratio + 2 D + (n_chunk - 1) * ratio]
    const uint32_t first = done * q->ratio;
    const uint32_t n_in  = 2 * D + (n_chunk - 1) * q->ratio + 1;

    uint32_t ph = (uint32_t)((n0 + first) % q->osc_period);
    for (uint32_t i = 0; i < n_in; ++i) {
      q->mixed[i] = x[first + i] * q->osc[ph];
      if (++ph == q->osc_period) {
        ph = 0;
      }
    }
    for (uint32_t j = 0; j < n_chunk; ++j) {
      const cf_t* c   = &q->mixed[D + j * q->ratio];
      float       re  = q->taps[0] * crealf(c[0]);
      float       im  = q->taps[0] * cimagf(c[0]);
      for (uint32_t m = 1; m <= D; ++m) {
        const float t = q->taps[m];
        re += t * (crealf(c[-(int32_t)m]) + crealf(c[m]));
        im += t * (cimagf(c[-(int32_t)m]) + cimagf(c[m]));
      }
      y[done + j] = re + _Complex_I * im;
    }
    done += n_chunk;
  }
  return SRSRAN_SUCCESS;
}
