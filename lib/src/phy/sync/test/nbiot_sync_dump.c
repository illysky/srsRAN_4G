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
 * Dumps what the NPSS/NSSS generators and the subframe writers produce, for nbiot_sync_check.py:
 *
 *   nbiot_sync_dump <pci> <nof_prb> <anchor_prb> <nof_frames>
 *
 * Lines:
 *   G NPSS <n> <re> <im>                      srsran_npss_generate(), 121 values
 *   G NSSS <t> <n> <re> <im>                  srsran_nsss_generate(pci), 4 x 132 values
 *   N <nf> <idx> <re> <im>                    every non-zero resource element of subframe 5 (NPSS) put into an
 *                                             all-zero LTE grid; idx = symbol * (12 nof_prb) + subcarrier
 *   S <nf> <idx> <re> <im>                    the same for subframe 9 of radio frame nf (NSSS), nf = 0 .. nof_frames-1
 * The grid has the LTE layout (14 symbols x 12 nof_prb subcarriers, DC not skipped in the index).
 */

#include <complex.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "srsran/phy/sync/npss.h"
#include "srsran/phy/sync/nsss.h"
#include "srsran/phy/utils/vector.h"

static void dump_nonzero(char tag, uint32_t nf, const cf_t* grid, uint32_t n)
{
  for (uint32_t i = 0; i < n; i++) {
    if (crealf(grid[i]) != 0.0f || cimagf(grid[i]) != 0.0f) {
      printf("%c %u %u %.7f %.7f\n", tag, nf, i, crealf(grid[i]), cimagf(grid[i]));
    }
  }
}

int main(int argc, char** argv)
{
  if (argc < 5) {
    fprintf(stderr, "usage: %s pci nof_prb anchor_prb nof_frames\n", argv[0]);
    return 1;
  }
  uint32_t pci = (uint32_t)atoi(argv[1]), nof_prb = (uint32_t)atoi(argv[2]), prb = (uint32_t)atoi(argv[3]);
  uint32_t nf = (uint32_t)atoi(argv[4]);

  uint32_t fft_sz = srsran_symbol_sz(nof_prb);
  uint32_t sf_len = 2 * SRSRAN_SLOT_LEN(fft_sz);

  srsran_npss_synch_t npss;
  srsran_nsss_synch_t nsss;
  if (srsran_npss_synch_init(&npss, sf_len, fft_sz) || srsran_nsss_synch_init(&nsss, sf_len, fft_sz)) {
    fprintf(stderr, "init failed\n");
    return 1;
  }

  cf_t npss_sig[SRSRAN_NPSS_TOT_LEN];
  cf_t nsss_sig[SRSRAN_NSSS_TOT_LEN];
  srsran_npss_generate(npss_sig);
  srsran_nsss_generate(nsss_sig, pci);

  for (int i = 0; i < SRSRAN_NPSS_TOT_LEN; i++) {
    printf("G NPSS %d %.7f %.7f\n", i, crealf(npss_sig[i]), cimagf(npss_sig[i]));
  }
  for (int t = 0; t < SRSRAN_NSSS_NUM_SEQ; t++) {
    for (int n = 0; n < SRSRAN_NSSS_LEN; n++) {
      cf_t v = nsss_sig[t * SRSRAN_NSSS_LEN + n];
      printf("G NSSS %d %d %.7f %.7f\n", t, n, crealf(v), cimagf(v));
    }
  }

  uint32_t grid_len = 14 * 12 * nof_prb;
  cf_t*    grid     = srsran_vec_cf_malloc(grid_len);

  memset(grid, 0, grid_len * sizeof(cf_t));
  srsran_npss_put_subframe(&npss, npss_sig, grid, nof_prb, prb);
  dump_nonzero('N', 0, grid, grid_len);

  for (uint32_t f = 0; f < nf; f++) {
    memset(grid, 0, grid_len * sizeof(cf_t));
    srsran_nsss_put_subframe(&nsss, nsss_sig, grid, (int)f, nof_prb, prb);
    dump_nonzero('S', f, grid, grid_len);
  }

  free(grid);
  srsran_npss_synch_free(&npss);
  srsran_nsss_synch_free(&nsss);
  return 0;
}
