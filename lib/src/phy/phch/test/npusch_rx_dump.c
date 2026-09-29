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
 * Runs the NPUSCH format 1 receiver on a file of 32 bit float I/Q samples (1.92 MS/s) written by the independent
 * Python transmitter, and prints the result:
 *
 *   npusch_rx_dump SAMPLES.cf32 key=value ...
 *
 * keys: n_sc spacing sc n_ru n_rep rv qm tbs rnti cell frame slot gh dss base cs win noise iters
 *
 * output: lines "crc_ok N", "snr_db X", "cfo_hz X", "noise X", "iters N" and, when the CRC is correct,
 * "tb 0101...".
 */

#include "srsran/phy/phch/npusch.h"
#include <complex.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char** argv)
{
  if (argc < 2) {
    fprintf(stderr, "usage: %s samples.cf32 key=value ...\n", argv[0]);
    return 2;
  }
  srsran_npusch_cfg_t cfg = {};
  cfg.base_seq            = -1;
  cfg.qm                  = 2;
  cfg.n_sc                = 1;
  cfg.spacing_hz          = 15000;
  cfg.n_ru                = 1;
  cfg.n_rep               = 1;
  for (int i = 2; i < argc; i++) {
    char* eq = strchr(argv[i], '=');
    if (!eq) {
      fprintf(stderr, "bad argument %s\n", argv[i]);
      return 2;
    }
    *eq          = 0;
    const char* k = argv[i];
    double      v = atof(eq + 1);
    if (!strcmp(k, "n_sc")) {
      cfg.n_sc = (uint32_t)v;
    } else if (!strcmp(k, "spacing")) {
      cfg.spacing_hz = (uint32_t)v;
    } else if (!strcmp(k, "sc")) {
      cfg.sc = (uint32_t)v;
    } else if (!strcmp(k, "n_ru")) {
      cfg.n_ru = (uint32_t)v;
    } else if (!strcmp(k, "n_rep")) {
      cfg.n_rep = (uint32_t)v;
    } else if (!strcmp(k, "rv")) {
      cfg.rv = (uint32_t)v;
    } else if (!strcmp(k, "qm")) {
      cfg.qm = (uint32_t)v;
    } else if (!strcmp(k, "tbs")) {
      cfg.tbs = (uint32_t)v;
    } else if (!strcmp(k, "rnti")) {
      cfg.rnti = (uint32_t)v;
    } else if (!strcmp(k, "cell")) {
      cfg.cell_id = (uint32_t)v;
    } else if (!strcmp(k, "frame")) {
      cfg.frame = (uint32_t)v;
    } else if (!strcmp(k, "slot")) {
      cfg.slot = (uint32_t)v;
    } else if (!strcmp(k, "gh")) {
      cfg.group_hopping = v != 0;
    } else if (!strcmp(k, "dss")) {
      cfg.delta_ss = (uint32_t)v;
    } else if (!strcmp(k, "base")) {
      cfg.base_seq = (int32_t)v;
    } else if (!strcmp(k, "cs")) {
      cfg.cyclic_shift = (uint32_t)v;
    } else if (!strcmp(k, "win")) {
      cfg.chest_window = (uint32_t)v;
    } else if (!strcmp(k, "noise")) {
      cfg.noise_var = (float)v;
    } else if (!strcmp(k, "iters")) {
      cfg.max_iterations = (uint32_t)v;
    } else {
      fprintf(stderr, "unknown key %s\n", k);
      return 2;
    }
  }
  if (srsran_npusch_check_cfg(&cfg) != SRSRAN_SUCCESS) {
    fprintf(stderr, "configuration rejected\n");
    return 3;
  }

  uint32_t n = srsran_npusch_nof_samples(&cfg);
  FILE*    f = fopen(argv[1], "rb");
  if (!f) {
    perror("open");
    return 2;
  }
  cf_t* samples = calloc(n, sizeof(cf_t));
  if (fread(samples, sizeof(cf_t), n, f) != n) {
    fprintf(stderr, "expected %u samples\n", n);
    return 2;
  }
  fclose(f);

  srsran_npusch_t rx;
  if (srsran_npusch_init(&rx)) {
    return 1;
  }
  uint8_t*            tb = calloc(cfg.tbs + 8, 1);
  srsran_npusch_res_t res;
  int                 ret = srsran_npusch_decode(&rx, &cfg, samples, tb, &res);
  if (ret != SRSRAN_SUCCESS) {
    fprintf(stderr, "decode failed: %d\n", ret);
    return 1;
  }
  printf("crc_ok %d\n", res.crc_ok ? 1 : 0);
  printf("snr_db %.2f\n", res.snr_db);
  printf("cfo_hz %.2f\n", res.cfo_hz);
  printf("noise %.6g\n", res.noise_var);
  printf("iters %u\n", res.nof_iterations);
  if (res.crc_ok) {
    printf("tb ");
    for (uint32_t i = 0; i < cfg.tbs; i++) {
      printf("%d", tb[i]);
    }
    printf("\n");
  }
  srsran_npusch_free(&rx);
  free(samples);
  free(tb);
  return 0;
}
