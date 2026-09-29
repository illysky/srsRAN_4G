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
 * NB-IoT NPRACH, TS 36.211 v14.2.0 clause 10.1.6.
 *
 * Detection follows the joint ToA / residual-CFO search described in
 *   X. Lin, A. Adhikary, Y.-P. E. Wang, "Random Access Preamble Design and
 *   Detection for 3GPP Narrowband IoT Systems", IEEE WCL 2016
 * (also implemented in the GPLv3 NPRACH_DETECTOR project, which is used here only as a source of
 * independent test data). For a hypothesised starting subcarrier the received symbol groups are
 * demodulated at the hopping subcarriers. Group i carries phase
 *      -2 pi k_i tau / 512  +  2 pi eps t_i  +  theta
 * where k_i is the hopping index, tau the arrival time (samples), eps the residual CFO and t_i the time
 * of the symbol, so tau and eps are the two frequencies of a 2D sinusoid and a matched search over a
 * (tau, eps) grid finds them. Repetitions are combined non-coherently.
 */

#include "srsran/phy/phch/nprach.h"
#include "srsran/phy/common/sequence.h"
#include "srsran/phy/utils/debug.h"
#include "srsran/phy/utils/vector.h"
#include <complex.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define NPRACH_WIN_LEN (SRSRAN_NPRACH_SYMS_PER_GROUP * SRSRAN_NPRACH_N_FFT)
#define NPRACH_GROUP_SYMS (SRSRAN_NPRACH_GROUPS_PER_REP * SRSRAN_NPRACH_SYMS_PER_GROUP)
#define NPRACH_TAU_MARGIN 16 // samples before the nominal start that are still searched
#define NPRACH_SYM_SECONDS ((double)SRSRAN_NPRACH_N_FFT / (double)SRSRAN_NPRACH_SRATE_HZ)

uint32_t srsran_nprach_cp_len(uint32_t format)
{
  // 2048 Ts and 8192 Ts, at 16 Ts per sample
  return format == 0 ? 128 : 512;
}

uint32_t srsran_nprach_group_len(uint32_t format)
{
  return srsran_nprach_cp_len(format) + NPRACH_WIN_LEN;
}

uint32_t srsran_nprach_group_start(const srsran_nprach_cfg_t* cfg, uint32_t group)
{
  // A 40 ms gap follows every 4 * 64 symbol groups (clause 10.1.6.1)
  return group * srsran_nprach_group_len(cfg->format) +
         (group / (SRSRAN_NPRACH_GROUPS_PER_REP * SRSRAN_NPRACH_REPS_PER_GAP)) * SRSRAN_NPRACH_GAP_LEN;
}

uint32_t srsran_nprach_nof_samples(const srsran_nprach_cfg_t* cfg)
{
  uint32_t last = SRSRAN_NPRACH_GROUPS_PER_REP * cfg->n_rep - 1;
  return srsran_nprach_group_start(cfg, last) + srsran_nprach_group_len(cfg->format);
}

int srsran_nprach_check_cfg(const srsran_nprach_cfg_t* cfg)
{
  if (cfg == NULL) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  if (cfg->format > 1) {
    ERROR("NPRACH: invalid preamble format %d", cfg->format);
    return SRSRAN_ERROR;
  }
  if (cfg->cell_id > 503) {
    ERROR("NPRACH: invalid cell id %d", cfg->cell_id);
    return SRSRAN_ERROR;
  }
  if (cfg->n_rep == 0 || cfg->n_rep > SRSRAN_NPRACH_MAX_REP || (cfg->n_rep & (cfg->n_rep - 1))) {
    ERROR("NPRACH: numRepetitions %d must be 1, 2, 4 ... 128", cfg->n_rep);
    return SRSRAN_ERROR;
  }
  if (cfg->n_sc_nprach == 0 || cfg->n_sc_nprach > SRSRAN_NPRACH_MAX_SC || cfg->n_sc_nprach % 12) {
    ERROR("NPRACH: nprach-NumSubcarriers %d must be 12, 24, 36 or 48", cfg->n_sc_nprach);
    return SRSRAN_ERROR;
  }
  static const uint32_t offsets[] = {0, 2, 12, 18, 24, 34, 36};
  bool                  ok        = false;
  for (uint32_t i = 0; i < sizeof(offsets) / sizeof(offsets[0]); i++) {
    ok |= (offsets[i] == cfg->n_sc_offset);
  }
  if (!ok) {
    ERROR("NPRACH: nprach-SubcarrierOffset %d is not one of 0, 2, 12, 18, 24, 34, 36", cfg->n_sc_offset);
    return SRSRAN_ERROR;
  }
  if (cfg->n_sc_offset + cfg->n_sc_nprach > SRSRAN_NPRACH_MAX_SC) {
    ERROR("NPRACH: subcarrier offset %d + %d subcarriers exceeds %d",
          cfg->n_sc_offset,
          cfg->n_sc_nprach,
          SRSRAN_NPRACH_MAX_SC);
    return SRSRAN_ERROR;
  }
  return SRSRAN_SUCCESS;
}

int srsran_nprach_hops(uint32_t cell_id, uint32_t n_init, uint32_t nof_groups, uint8_t* hop)
{
  if (hop == NULL || nof_groups == 0 || nof_groups > SRSRAN_NPRACH_MAX_GROUPS || cell_id > 503) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }

  // f(t) uses c(10t+1) .. c(10t+9)
  uint32_t n_t = (nof_groups - 1) / SRSRAN_NPRACH_GROUPS_PER_REP + 1;
  uint32_t len = 10 * n_t + 10;

  srsran_sequence_t seq = {};
  if (srsran_sequence_init(&seq, len) != SRSRAN_SUCCESS) {
    return SRSRAN_ERROR;
  }
  if (srsran_sequence_set_LTE_pr(&seq, len, cell_id) != SRSRAN_SUCCESS) {
    srsran_sequence_free(&seq);
    return SRSRAN_ERROR;
  }

  uint32_t f[SRSRAN_NPRACH_MAX_REP + 1];
  uint32_t f_prev = 0; // f(-1) = 0
  for (uint32_t t = 0; t < n_t; t++) {
    uint32_t sum = 0;
    for (uint32_t k = 0; k < 9; k++) {
      sum += (uint32_t)(seq.c[10 * t + 1 + k] & 1) << k;
    }
    f[t]   = (f_prev + (sum % (SRSRAN_NPRACH_N_SC_RA - 1)) + 1) % SRSRAN_NPRACH_N_SC_RA;
    f_prev = f[t];
  }
  srsran_sequence_free(&seq);

  uint32_t n0 = n_init % SRSRAN_NPRACH_N_SC_RA;
  for (uint32_t i = 0; i < nof_groups; i++) {
    uint32_t v;
    if (i == 0) {
      v = n0;
    } else if (i % 4 == 0) {
      v = (n0 + f[i / 4]) % SRSRAN_NPRACH_N_SC_RA;
    } else if (i % 4 == 2) {
      v = hop[i - 1] < 6 ? hop[i - 1] + 6 : hop[i - 1] - 6;
    } else { // i % 4 == 1 or 3
      v = (hop[i - 1] % 2 == 0) ? hop[i - 1] + 1 : hop[i - 1] - 1;
    }
    hop[i] = (uint8_t)v;
  }
  return SRSRAN_SUCCESS;
}

/** Starting subcarrier of the 12-subcarrier hopping block: n_start = N_scoffset + floor(n_init / 12) * 12 */
static uint32_t nprach_n_start(const srsran_nprach_cfg_t* cfg, uint32_t n_init)
{
  return cfg->n_sc_offset + (n_init / SRSRAN_NPRACH_N_SC_RA) * SRSRAN_NPRACH_N_SC_RA;
}

int srsran_nprach_gen(const srsran_nprach_cfg_t* cfg, uint32_t n_init, cf_t* out)
{
  if (out == NULL || srsran_nprach_check_cfg(cfg) != SRSRAN_SUCCESS || n_init >= cfg->n_sc_nprach) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }

  uint32_t n_groups = SRSRAN_NPRACH_GROUPS_PER_REP * cfg->n_rep;
  uint8_t* hop      = calloc(n_groups, 1);
  if (hop == NULL) {
    return SRSRAN_ERROR;
  }
  if (srsran_nprach_hops(cfg->cell_id, n_init, n_groups, hop) != SRSRAN_SUCCESS) {
    free(hop);
    return SRSRAN_ERROR;
  }

  memset(out, 0, sizeof(cf_t) * srsran_nprach_nof_samples(cfg));

  uint32_t cp   = srsran_nprach_cp_len(cfg->format);
  uint32_t glen = srsran_nprach_group_len(cfg->format);
  uint32_t base = nprach_n_start(cfg, n_init);
  for (uint32_t g = 0; g < n_groups; g++) {
    // s_i(t) = exp(j 2 pi (n_sc(i) + K k0 + 1/2) df (t - T_CP)),  K k0 = 4 * (-6) = -24
    double   f     = ((double)(base + hop[g]) - 24.0 + 0.5) * 3750.0;
    uint32_t start = srsran_nprach_group_start(cfg, g);
    for (uint32_t m = 0; m < glen; m++) {
      double t     = ((double)m - (double)cp) / (double)SRSRAN_NPRACH_SRATE_HZ;
      double phase = 2.0 * M_PI * f * t;
      out[start + m] = (float)cos(phase) + I * (float)sin(phase);
    }
  }
  free(hop);
  return SRSRAN_SUCCESS;
}

static void nprach_build_eps(srsran_nprach_t* q, double group_period_syms)
{
  for (uint32_t e = 0; e < q->n_eps; e++) {
    double eps = ((double)e - (double)(q->n_eps / 2)) * q->eps_step_hz;
    for (uint32_t j = 0; j < SRSRAN_NPRACH_GROUPS_PER_REP; j++) {
      for (uint32_t s = 0; s < SRSRAN_NPRACH_SYMS_PER_GROUP; s++) {
        double t = ((double)j * group_period_syms + (double)s) * NPRACH_SYM_SECONDS;
        double p = -2.0 * M_PI * eps * t;
        q->tw_eps[e * NPRACH_GROUP_SYMS + j * SRSRAN_NPRACH_SYMS_PER_GROUP + s] = (float)cos(p) + I * (float)sin(p);
      }
    }
  }
  q->tw_eps_period = group_period_syms;
}

float srsran_nprach_default_threshold(uint32_t n_rep)
{
  // Noise-only, per starting subcarrier, the peak of the correlation surface over its mean noise level has an
  // exponential upper tail. Measured with nprach_test -E (12000 hypothesis trials for 1..4 repetitions, fewer for
  // more) and extrapolated, the level exceeded with probability 1e-6 is, for formats 0 / 1:
  //   n_rep   1      2      4     8     16    32
  //          23.1   12.9   7.8   5.1   3.6   2.65   (format 0)
  //          24.5   13.9   8.4   5.3   3.3   2.54   (format 1)
  // 1 + 23.5 * n_rep^-0.75 lies above every one of them. With 48 starting subcarriers that is a false alarm rate of
  // about 5e-5 per NPRACH occasion.
  if (n_rep == 0) {
    n_rep = 1;
  }
  // Beyond 32 repetitions the tail was only measured to about 1e-3 (max 1.74 at 64, 1.51 at 128 repetitions), so the
  // formula is not allowed below 1.8 there.
  float th = 1.0f + 23.5f * powf((float)n_rep, -0.75f);
  return th < 1.8f ? 1.8f : th;
}

void srsran_nprach_free(srsran_nprach_t* q)
{
  if (q == NULL) {
    return;
  }
  srsran_dft_plan_free(&q->fft);
  free(q->shift);
  free(q->tw_tau);
  free(q->tw_eps);
  free(q->bins);
  free(q->win);
  free(q->spec);
  free(q->z);
  free(q->surf);
  free(q->sel);
  free(q->hop);
  free(q->y);
  memset(q, 0, sizeof(srsran_nprach_t));
}

int srsran_nprach_init(srsran_nprach_t* q, const srsran_nprach_cfg_t* cfg)
{
  if (q == NULL || srsran_nprach_check_cfg(cfg) != SRSRAN_SUCCESS) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  memset(q, 0, sizeof(srsran_nprach_t));
  q->cfg = *cfg;

  uint32_t n_groups = SRSRAN_NPRACH_GROUPS_PER_REP * cfg->n_rep;

  q->tau_min      = -NPRACH_TAU_MARGIN;
  // The hop phase repeats every 512 samples (1 / 3.75 kHz), so a ToA search span longer than that would alias
  // (format 1: CP = 512 plus the margin). Delays up to 512 - margin samples (38.7 km) are unambiguous.
  q->n_tau        = srsran_nprach_cp_len(cfg->format) + NPRACH_TAU_MARGIN + 1;
  if (q->n_tau > SRSRAN_NPRACH_N_FFT) {
    q->n_tau = SRSRAN_NPRACH_N_FFT;
  }
  q->n_eps        = 41;
  q->eps_step_hz  = 40.0f;
  q->threshold    = srsran_nprach_default_threshold(cfg->n_rep);
  q->ghost_ratio  = 0.1f;
  q->noise_var    = -1.0f;

  if (srsran_dft_plan_c(&q->fft, SRSRAN_NPRACH_N_FFT, SRSRAN_DFT_FORWARD)) {
    return SRSRAN_ERROR;
  }
  srsran_dft_plan_set_mirror(&q->fft, false);
  srsran_dft_plan_set_dc(&q->fft, false);
  srsran_dft_plan_set_norm(&q->fft, false);

  q->shift  = srsran_vec_cf_malloc(NPRACH_WIN_LEN);
  q->tw_tau = srsran_vec_cf_malloc(q->n_tau * SRSRAN_NPRACH_N_SC_RA);
  q->tw_eps = srsran_vec_cf_malloc(q->n_eps * NPRACH_GROUP_SYMS);
  q->bins   = srsran_vec_cf_malloc(n_groups * SRSRAN_NPRACH_SYMS_PER_GROUP * SRSRAN_NPRACH_MAX_SC);
  q->win    = srsran_vec_cf_malloc(SRSRAN_NPRACH_N_FFT);
  q->spec   = srsran_vec_cf_malloc(SRSRAN_NPRACH_N_FFT);
  q->z      = srsran_vec_cf_malloc(SRSRAN_NPRACH_GROUPS_PER_REP * q->n_eps);
  q->surf   = srsran_vec_f_malloc(q->n_tau * q->n_eps);
  q->sel    = srsran_vec_f_malloc(q->n_tau * q->n_eps);
  q->hop    = calloc(n_groups, 1);
  q->y      = srsran_vec_cf_malloc(n_groups * SRSRAN_NPRACH_SYMS_PER_GROUP);
  if (!q->shift || !q->tw_tau || !q->tw_eps || !q->bins || !q->win || !q->spec || !q->z || !q->surf || !q->sel ||
      !q->hop || !q->y) {
    srsran_nprach_free(q);
    return SRSRAN_ERROR;
  }

  // Shift by half a subcarrier so that the tones (n_sc - 24 + 1/2) * 3.75 kHz land on integer FFT bins. The
  // shift is continuous over the 5 symbols of a group, so all 5 symbols of a tone have the same phase.
  for (uint32_t i = 0; i < NPRACH_WIN_LEN; i++) {
    double p  = -M_PI * (double)i / (double)SRSRAN_NPRACH_N_FFT;
    q->shift[i] = (float)cos(p) + I * (float)sin(p);
  }
  for (uint32_t t = 0; t < q->n_tau; t++) {
    double tau = (double)q->tau_min + (double)t;
    for (uint32_t k = 0; k < SRSRAN_NPRACH_N_SC_RA; k++) {
      double p = 2.0 * M_PI * (double)k * tau / (double)SRSRAN_NPRACH_N_FFT;
      q->tw_tau[t * SRSRAN_NPRACH_N_SC_RA + k] = (float)cos(p) + I * (float)sin(p);
    }
  }
  nprach_build_eps(q, (double)srsran_nprach_group_len(cfg->format) / (double)SRSRAN_NPRACH_N_FFT);
  return SRSRAN_SUCCESS;
}

/** k-th smallest element (0-based) of a[0..n-1]; reorders a. */
static float nth_element(float* a, uint32_t n, uint32_t k)
{
  uint32_t lo = 0, hi = n - 1;
  while (lo < hi) {
    float    pivot = a[(lo + hi) / 2];
    uint32_t i = lo, j = hi;
    while (i <= j) {
      while (a[i] < pivot) {
        i++;
      }
      while (a[j] > pivot) {
        j--;
      }
      if (i <= j) {
        float t = a[i];
        a[i]    = a[j];
        a[j]    = t;
        i++;
        if (j == 0) {
          break;
        }
        j--;
      }
    }
    if (k <= j) {
      hi = j;
    } else if (k >= i) {
      lo = i;
    } else {
      return a[k];
    }
  }
  return a[k];
}

/** Exact value of the correlation at (tau, eps) for the given hypothesis (the surface the grid samples). */
static double nprach_power(const srsran_nprach_t* q,
                           const cf_t*            y,
                           const uint8_t*         hop,
                           uint32_t               n_rep,
                           double                 group_period_syms,
                           double                 tau,
                           double                 eps)
{
  double total = 0.0;
  (void)q;
  for (uint32_t r = 0; r < n_rep; r++) {
    double complex acc = 0;
    for (uint32_t j = 0; j < SRSRAN_NPRACH_GROUPS_PER_REP; j++) {
      uint32_t g   = r * SRSRAN_NPRACH_GROUPS_PER_REP + j;
      double   pk  = 2.0 * M_PI * (double)hop[g] * tau / (double)SRSRAN_NPRACH_N_FFT;
      for (uint32_t s = 0; s < SRSRAN_NPRACH_SYMS_PER_GROUP; s++) {
        double t  = ((double)j * group_period_syms + (double)s) * NPRACH_SYM_SECONDS;
        double ph = pk - 2.0 * M_PI * eps * t;
        acc += (double complex)y[g * SRSRAN_NPRACH_SYMS_PER_GROUP + s] * (cos(ph) + I * sin(ph));
      }
    }
    total += creal(acc) * creal(acc) + cimag(acc) * cimag(acc);
  }
  return total;
}

/** Pattern search around the grid maximum. The hop pattern correlates hopping index with time, so tau and eps
 *  form an elongated ridge and estimating them one axis at a time is biased. */
static void nprach_refine(const srsran_nprach_t* q,
                          const cf_t*            y,
                          const uint8_t*         hop,
                          uint32_t               n_rep,
                          double                 group_period_syms,
                          double*                tau,
                          double*                eps,
                          double*                power)
{
  double dt = 0.5, de = 0.25 * q->eps_step_hz;
  double best = nprach_power(q, y, hop, n_rep, group_period_syms, *tau, *eps);
  for (int it = 0; it < 60 && dt > 0.005; it++) {
    double bt = *tau, be = *eps, bp = best;
    for (int a = -1; a <= 1; a++) {
      for (int b = -1; b <= 1; b++) {
        if (a == 0 && b == 0) {
          continue;
        }
        double p = nprach_power(q, y, hop, n_rep, group_period_syms, *tau + a * dt, *eps + b * de);
        if (p > bp) {
          bp = p;
          bt = *tau + a * dt;
          be = *eps + b * de;
        }
      }
    }
    if (bp > best) {
      best = bp;
      *tau = bt;
      *eps = be;
    } else {
      dt *= 0.5;
      de *= 0.5;
    }
  }
  *power = best;
}

void srsran_nprach_search(srsran_nprach_t*     q,
                          const cf_t*          y,
                          const uint8_t*       hop,
                          uint32_t             nof_groups,
                          double               group_period_syms,
                          srsran_nprach_det_t* out)
{
  if (fabs(group_period_syms - q->tw_eps_period) > 1e-9) {
    nprach_build_eps(q, group_period_syms);
  }

  const uint32_t n_eps = q->n_eps;
  const uint32_t n_tau = q->n_tau;
  const uint32_t n_rep = nof_groups / SRSRAN_NPRACH_GROUPS_PER_REP;

  bzero(q->surf, sizeof(float) * n_tau * n_eps);

  for (uint32_t r = 0; r < n_rep; r++) {
    // Stage 1: coherent sum over the 5 symbols of each group for every CFO hypothesis
    for (uint32_t j = 0; j < SRSRAN_NPRACH_GROUPS_PER_REP; j++) {
      const cf_t* yg = &y[(r * SRSRAN_NPRACH_GROUPS_PER_REP + j) * SRSRAN_NPRACH_SYMS_PER_GROUP];
      cf_t*       zj = &q->z[j * n_eps];
      for (uint32_t e = 0; e < n_eps; e++) {
        const cf_t* tw  = &q->tw_eps[e * NPRACH_GROUP_SYMS + j * SRSRAN_NPRACH_SYMS_PER_GROUP];
        cf_t        acc = 0;
        for (uint32_t s = 0; s < SRSRAN_NPRACH_SYMS_PER_GROUP; s++) {
          acc += yg[s] * tw[s];
        }
        zj[e] = acc;
      }
    }

    // Stage 2: coherent sum over the 4 groups of a repetition for every ToA hypothesis, power-combined
    const uint8_t* h = &hop[r * SRSRAN_NPRACH_GROUPS_PER_REP];
    for (uint32_t t = 0; t < n_tau; t++) {
      const cf_t* c  = &q->tw_tau[t * SRSRAN_NPRACH_N_SC_RA];
      cf_t        c0 = c[h[0]], c1 = c[h[1]], c2 = c[h[2]], c3 = c[h[3]];
      const cf_t *z0 = &q->z[0], *z1 = &q->z[n_eps], *z2 = &q->z[2 * n_eps], *z3 = &q->z[3 * n_eps];
      float*      p  = &q->surf[t * n_eps];
      for (uint32_t e = 0; e < n_eps; e++) {
        cf_t w = z0[e] * c0 + z1[e] * c1 + z2[e] * c2 + z3[e] * c3;
        p[e] += crealf(w) * crealf(w) + cimagf(w) * cimagf(w);
      }
    }
  }

  // Peak of the surface, and its median as the noise floor: the signal only occupies a small part of the
  // (tau, eps) plane, so the median is insensitive to it, whereas the mean is not.
  uint32_t best = 0;
  for (uint32_t i = 0; i < n_tau * n_eps; i++) {
    if (q->surf[i] > q->surf[best]) {
      best = i;
    }
  }
  memcpy(q->sel, q->surf, sizeof(float) * n_tau * n_eps);
  float    median = nth_element(q->sel, n_tau * n_eps, (n_tau * n_eps) / 2);
  uint32_t bt     = best / n_eps;
  uint32_t be     = best % n_eps;

  double tau = (double)q->tau_min + (double)bt;
  double eps = ((double)be - (double)(n_eps / 2)) * (double)q->eps_step_hz;
  double peak;
  nprach_refine(q, y, hop, n_rep, group_period_syms, &tau, &eps, &peak);

  // Normalise by the noise level of the surface: every cell of a noise-only surface is a sum of n_rep terms of
  // 4 groups * 5 symbols with unit-modulus weights, i.e. has mean 20 * n_rep * noise_var.
  double floor_ = q->noise_var > 0.0f ? 20.0 * n_rep * (double)q->noise_var : (double)median;

  out->toa      = (float)tau;
  out->cfo_hz   = (float)eps;
  out->peak     = (float)peak;
  out->metric   = floor_ > 0.0 ? (float)(peak / floor_) : 1e9f;
  out->detected = out->metric >= q->threshold;
}

int srsran_nprach_detect(srsran_nprach_t* q, const cf_t* x, srsran_nprach_det_t* det)
{
  if (q == NULL || x == NULL || det == NULL) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }

  const srsran_nprach_cfg_t* cfg = &q->cfg;
  uint32_t n_groups = SRSRAN_NPRACH_GROUPS_PER_REP * cfg->n_rep;
  uint32_t cp       = srsran_nprach_cp_len(cfg->format);

  // Per-symbol FFT of every group; keep the 48 NPRACH subcarrier bins. Subcarrier n sits at (n - 24) after the
  // half-subcarrier shift.
  for (uint32_t g = 0; g < n_groups; g++) {
    const cf_t* w0 = &x[srsran_nprach_group_start(cfg, g) + cp];
    for (uint32_t s = 0; s < SRSRAN_NPRACH_SYMS_PER_GROUP; s++) {
      srsran_vec_prod_ccc(&w0[s * SRSRAN_NPRACH_N_FFT], &q->shift[s * SRSRAN_NPRACH_N_FFT], q->win, SRSRAN_NPRACH_N_FFT);
      srsran_dft_run_c(&q->fft, q->win, q->spec);
      cf_t* b = &q->bins[(g * SRSRAN_NPRACH_SYMS_PER_GROUP + s) * SRSRAN_NPRACH_MAX_SC];
      for (uint32_t n = 0; n < SRSRAN_NPRACH_MAX_SC; n++) {
        b[n] = q->spec[(n + SRSRAN_NPRACH_N_FFT - 24) % SRSRAN_NPRACH_N_FFT];
      }
    }
  }

  // Noise power per bin: the 20th percentile of |bin|^2 over the whole NPRACH region and all symbols. A tone
  // occupies one bin of a 12 subcarrier block per symbol, so the low percentile is unaffected by the signal, and
  // for exponentially distributed |bin|^2 it is 0.2231 * variance.
  {
    uint32_t n_pool = 0;
    float*   pool   = q->sel; // large enough: n_tau * n_eps >= 145 * 41; grown below if needed
    uint32_t cap    = q->n_tau * q->n_eps;
    uint32_t stride = 1;
    uint32_t total  = n_groups * SRSRAN_NPRACH_SYMS_PER_GROUP * cfg->n_sc_nprach;
    while (total / stride > cap) {
      stride++;
    }
    uint32_t idx = 0;
    for (uint32_t gs = 0; gs < n_groups * SRSRAN_NPRACH_SYMS_PER_GROUP; gs++) {
      for (uint32_t n = cfg->n_sc_offset; n < cfg->n_sc_offset + cfg->n_sc_nprach; n++, idx++) {
        if (idx % stride == 0 && n_pool < cap) {
          cf_t v       = q->bins[gs * SRSRAN_NPRACH_MAX_SC + n];
          pool[n_pool++] = crealf(v) * crealf(v) + cimagf(v) * cimagf(v);
        }
      }
    }
    float p20   = nth_element(pool, n_pool, n_pool / 5);
    q->noise_var = p20 / 0.2231f;
  }

  double group_period = (double)srsran_nprach_group_len(cfg->format) / (double)SRSRAN_NPRACH_N_FFT;
  int    nof_det      = 0;
  for (uint32_t n_init = 0; n_init < cfg->n_sc_nprach; n_init++) {
    srsran_nprach_hops(cfg->cell_id, n_init, n_groups, q->hop);
    uint32_t base = nprach_n_start(cfg, n_init);
    for (uint32_t g = 0; g < n_groups; g++) {
      const cf_t* b = &q->bins[(g * SRSRAN_NPRACH_SYMS_PER_GROUP) * SRSRAN_NPRACH_MAX_SC];
      for (uint32_t s = 0; s < SRSRAN_NPRACH_SYMS_PER_GROUP; s++) {
        q->y[g * SRSRAN_NPRACH_SYMS_PER_GROUP + s] = b[s * SRSRAN_NPRACH_MAX_SC + base + q->hop[g]];
      }
    }
    srsran_nprach_search(q, q->y, q->hop, n_groups, group_period, &det[n_init]);
    det[n_init].n_init = n_init;
  }

  // A stronger preamble leaks into hypotheses that share neighbouring subcarriers with it. Those partial matches
  // are far weaker than the real detection, so drop everything below ghost_ratio of the strongest.
  float strongest = 0.0f;
  for (uint32_t i = 0; i < cfg->n_sc_nprach; i++) {
    if (det[i].detected && det[i].peak > strongest) {
      strongest = det[i].peak;
    }
  }
  for (uint32_t i = 0; i < cfg->n_sc_nprach; i++) {
    if (det[i].detected && det[i].peak < q->ghost_ratio * strongest) {
      det[i].detected = false;
    }
    nof_det += det[i].detected ? 1 : 0;
  }
  return nof_det;
}
