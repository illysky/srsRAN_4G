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
 * Runs the NPRACH search on the test vectors of the third party NPRACH_DETECTOR project
 * (https://github.com/Pk8598/NPRACH_DETECTOR, GPLv3), which are used here only as independent data.
 *
 * Each vector is a 512 x 640 matrix (row = FFT bin, column = NPRACH symbol) of the demodulated signal of 12 users
 * that start on the 12 different subcarriers of one block, with a ToA and residual CFO per user in the matching
 * True_values file. The vectors use the project's own hopping table (freqHops.txt) rather than the pseudo random
 * sequence of TS 36.211, so it is read from the file. The columns are the symbols of consecutive groups with the
 * cyclic prefixes already removed, but the residual CFO in them advances with the real group period (5.25 symbols
 * for format 0, 6 for format 1).
 *
 *   nprach_vectors_test ROOT PRM FIRST LAST [need_toa_pct need_cfo_pct]
 *                       ROOT = NPRACH_DETECTOR checkout, PRM = 0 or 1
 *   NPRACH_VERBOSE=1    print every user;  NPRACH_KEEP_SYM0=1  keep symbol 0 of every group (see below)
 */

#include "srsran/phy/phch/nprach.h"
#include "srsran/phy/utils/vector.h"
#include <complex.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define N_ROWS 512
#define N_COLS 640
#define N_USERS 12
#define N_GROUPS 128

/** Reads all numbers of a comma separated text file. Returns the count, -1 on error. */
static long read_numbers(const char* path, double* out, long max)
{
  FILE* f = fopen(path, "r");
  if (!f) {
    fprintf(stderr, "cannot open %s\n", path);
    return -1;
  }
  long n = 0;
  int  c;
  char tok[64];
  int  len = 0;
  while ((c = fgetc(f)) != EOF) {
    if (c == ',' || c == '\n' || c == '\r' || c == ' ' || c == '\t') {
      if (len) {
        tok[len] = 0;
        if (n < max) {
          out[n] = atof(tok);
        }
        n++;
        len = 0;
      }
    } else if (len < 63) {
      tok[len++] = (char)c;
    }
  }
  if (len) {
    tok[len] = 0;
    if (n < max) {
      out[n] = atof(tok);
    }
    n++;
  }
  fclose(f);
  return n;
}

static int cmp(const void* a, const void* b)
{
  double x = *(const double*)a, y = *(const double*)b;
  return x < y ? -1 : (x > y);
}

int main(int argc, char** argv)
{
  if (argc < 5) {
    fprintf(stderr, "usage: %s ROOT PRM FIRST LAST\n", argv[0]);
    return 2;
  }
  const char* root  = argv[1];
  int         prm   = atoi(argv[2]);
  int         first = atoi(argv[3]);
  int         last  = atoi(argv[4]);
  if (prm < 0 || prm > 1) {
    return 2;
  }

  char path[1024];

  double* hops_raw = malloc(sizeof(double) * N_USERS * 128);
  snprintf(path, sizeof(path), "%s/NPRACH_C/freqHops.txt", root);
  if (read_numbers(path, hops_raw, N_USERS * 128) != N_USERS * 128) {
    fprintf(stderr, "freqHops.txt: unexpected size\n");
    return 1;
  }

  // The search runs with the matching preamble format (CP length decides the ToA range)
  srsran_nprach_cfg_t cfg = {.cell_id = 1, .format = prm, .n_rep = 32, .n_sc_offset = 0, .n_sc_nprach = 12};
  srsran_nprach_t     q;
  if (srsran_nprach_init(&q, &cfg)) {
    return 1;
  }
  q.noise_var = -1.0f; // no noise reference in these files: normalise by the surface median

  // The vectors were generated with the real timing, cyclic prefix included: a group starts (CP + 5 symbols) after
  // the previous one, i.e. 5.25 symbols (format 0) or 6 symbols (format 1). (Assuming 5 inflates every CFO estimate
  // by exactly that ratio, which is how this was found.)
  double group_period = (double)srsran_nprach_group_len(cfg.format) / (double)SRSRAN_NPRACH_N_FFT;

  double* raw = malloc(sizeof(double) * 2 * N_ROWS * N_COLS);
  cf_t*   y   = srsran_vec_cf_malloc(N_GROUPS * 5);
  uint8_t hop[N_GROUPS];

  int mask_sym0 = getenv("NPRACH_KEEP_SYM0") == NULL;

  // The ToA of a hopping preamble is only defined modulo 512 samples (the phase of tone k advances by 2 pi k tau / 512),
  // so the search covers exactly one period, [tau_min, tau_min + n_tau). For format 1 that is [-16, 496), i.e. cells up
  // to about 38.7 km. A true ToA outside it is indistinguishable from one 512 samples earlier, so such users cannot be
  // scored and are counted separately.
  const double span_lo = (double)q.tau_min;
  const double span_hi = (double)q.tau_min + (double)q.n_tau;
  int          n_out   = 0;

  double* toa_err = malloc(sizeof(double) * N_USERS * (last - first + 1));
  double* cfo_err = malloc(sizeof(double) * N_USERS * (last - first + 1));
  double* metric  = malloc(sizeof(double) * N_USERS * (last - first + 1));
  int     n       = 0;

  for (int v = first; v <= last; v++) {
    snprintf(path, sizeof(path), "%s/Test_Data/CVA_3/PRM_%d/rxDataMat%d_%d.txt", root, prm, prm, v);
    if (read_numbers(path, raw, 2 * N_ROWS * N_COLS) != 2 * N_ROWS * N_COLS) {
      fprintf(stderr, "%s: unexpected size\n", path);
      return 1;
    }
    double truth[2 * N_USERS];
    snprintf(path, sizeof(path), "%s/Test_Data/CVA_3/PRM_%d/True_values%d_%d.txt", root, prm, prm, v);
    if (read_numbers(path, truth, 2 * N_USERS) != 2 * N_USERS) {
      fprintf(stderr, "%s: unexpected size\n", path);
      return 1;
    }

    for (int u = 0; u < N_USERS; u++) {
      for (int g = 0; g < N_GROUPS; g++) {
        hop[g] = (uint8_t)hops_raw[u * 128 + g];
        for (int s = 0; s < 5; s++) {
          int col = g * 5 + s;
          int idx = 2 * (hop[g] * N_COLS + col);
          y[g * 5 + s] = (float)raw[idx] + I * (float)raw[idx + 1];
        }
        // The vectors' cyclic prefix is a copy of the end of the group, not the continuation of the tone that TS 36.211
        // 10.1.6.2 (s_i(t) with (t - T_CP)) defines. For a signal delayed by tau samples the FFT window of symbol 0
        // therefore holds tau samples of the wrong sign, and the coherent amplitude of symbol 0 is (512 - 2 tau) / 512:
        // measured over 40000 group observations per format this matches to within 0.03 for every 64-sample ToA band,
        // and it changes sign at tau = 256. Symbols 1..4 are clean. A conforming UE has no such effect, so instead of
        // bending the detector the contaminated symbol is left out here. NPRACH_KEEP_SYM0=1 keeps it.
        if (mask_sym0) {
          y[g * 5] = 0;
        }
      }
      // Every user is searched as a 32 repetition user. The vectors mix users of 2, 8 and 32 repetitions (the coverage
      // levels of NPRACH_Config_Fixed.c), which a real cell would put on separate NPRACH resources; running a separate
      // 2 / 8 / 32 repetition hypothesis per user and keeping the best was tried and picked 32 for all 1200 users of
      // both formats with identical estimates, so it is not done.
      srsran_nprach_det_t det;
      srsran_nprach_search(&q, y, hop, N_GROUPS, group_period, &det);
      if (getenv("NPRACH_VERBOSE")) {
        printf("vec %d user %2d: true ToA %6.1f est %7.2f  true CFO %7.1f est %7.1f  metric %.1f\n", v, u, truth[u], det.toa,
               truth[N_USERS + u] * 1.92e6, det.cfo_hz, det.metric);
      }
      // (within the 5 sample tolerance of an edge an estimate can wrap over it, so that band is not scored either)
      if (truth[u] < span_lo + 5.0 || truth[u] >= span_hi - 5.0) {
        n_out++;
        continue;
      }
      toa_err[n] = det.toa - truth[u];
      cfo_err[n] = det.cfo_hz - truth[N_USERS + u] * 1.92e6;
      metric[n]  = det.metric;
      n++;
    }
  }

  // Statistics of the absolute errors
  double* ta = malloc(sizeof(double) * n);
  double* ca = malloc(sizeof(double) * n);
  int     w2 = 0, w3 = 0, w5 = 0, wc30 = 0, wc60 = 0;
  for (int i = 0; i < n; i++) {
    ta[i] = fabs(toa_err[i]);
    ca[i] = fabs(cfo_err[i]);
    w2 += ta[i] <= 2.0;
    w3 += ta[i] <= 3.0;
    w5 += ta[i] <= 5.0;
    wc30 += ca[i] <= 30.0;
    wc60 += ca[i] <= 60.0;
  }
  qsort(ta, n, sizeof(double), cmp);
  qsort(ca, n, sizeof(double), cmp);
  double mean_sign = 0;
  for (int i = 0; i < n; i++) {
    mean_sign += toa_err[i];
  }
  printf("PRM %d, vectors %d..%d: %d users scored (+%d with a true ToA outside the %.0f..%.0f sample search span)%s\n", prm,
         first, last, n, n_out, span_lo, span_hi, mask_sym0 ? "" : ", symbol 0 kept");
  printf("  ToA error  : median %.2f  p90 %.2f  max %.1f samples; mean signed %.2f; within 2/3/5 samples: %.1f%% / %.1f%% / "
         "%.1f%%\n",
         ta[n / 2], ta[(int)(n * 0.9)], ta[n - 1], mean_sign / n, 100.0 * w2 / n, 100.0 * w3 / n, 100.0 * w5 / n);
  printf("  CFO error  : median %.1f  p90 %.1f  max %.1f Hz; within 30 / 60 Hz: %.1f%% / %.1f%%\n", ca[n / 2],
         ca[(int)(n * 0.9)], ca[n - 1], 100.0 * wc30 / n, 100.0 * wc60 / n);

  // Pass criteria: 99% of the scored users within 5 samples and within 60 Hz. Measured over all 100 vectors per format
  // with symbol 0 left out: format 0 99.9% / 100%, format 1 99.5% / 100%; with symbol 0 kept (its coherent amplitude
  // is (512 - 2 tau) / 512 in these vectors, see above) format 1 drops to 88% / 90%.
  int fail = 0;
  double need_toa = argc > 5 ? atof(argv[5]) : 99.0;
  double need_cfo = argc > 6 ? atof(argv[6]) : 99.0;
  if (100.0 * w5 / n < need_toa) {
    printf("FAIL: only %.1f%% of ToA estimates within 5 samples (need %.0f%%)\n", 100.0 * w5 / n, need_toa);
    fail = 1;
  }
  if (100.0 * wc60 / n < need_cfo) {
    printf("FAIL: only %.1f%% of CFO estimates within 60 Hz (need %.0f%%)\n", 100.0 * wc60 / n, need_cfo);
    fail = 1;
  }
  srsran_nprach_free(&q);
  return fail;
}
