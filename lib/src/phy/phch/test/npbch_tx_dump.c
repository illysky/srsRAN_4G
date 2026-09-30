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
 * Dumps what the NPBCH transmitter puts on the air, frame by frame, for npbch_tx_check.py:
 *
 *   npbch_tx_dump <pci> <nof_frames>
 *
 * prints, for an in-band 25-PRB cell,
 *   S <f> <100 x (re im)>   the 100 NPBCH resource elements of frame f from the STREAMING encoder (frames in order)
 *   L <f> <100 x (re im)>   the same frame from the STATELESS encoder, called in a shuffled order
 *   P <f> <34 bits>         the MIB-NB payload used for the 64-frame period of frame f
 * The MIB payload changes from one 64-frame period to the next (SFN MSBs differ), so a stateless encoder that
 * reused a stale payload or block would be visible.
 */

#include <complex.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "srsran/phy/phch/npbch.h"
#include "srsran/phy/utils/vector.h"

#define GRID_RE (25 * 12 * 14)

static srsran_nbiot_cell_t make_cell(uint32_t pci)
{
  srsran_nbiot_cell_t cell;
  memset(&cell, 0, sizeof(cell));
  cell.base.nof_ports = 1;
  cell.base.nof_prb   = 25;
  cell.base.cp        = SRSRAN_CP_NORM;
  cell.base.id        = pci;
  cell.nbiot_prb      = 17;
  cell.n_id_ncell     = pci;
  cell.nof_ports      = 1;
  cell.mode           = SRSRAN_NBIOT_MODE_INBAND_SAME_PCI;
  cell.is_r14         = true; // the rotated NPBCH (TS 36.211 10.2.4.4, v13.6.0 and later)
  return cell;
}

static void payload_for(uint32_t frame, uint8_t* out)
{
  srsran_mib_nb_t mib;
  memset(&mib, 0, sizeof(mib));
  mib.sched_info_sib1    = 3;
  mib.sys_info_tag       = 5;
  mib.mode               = SRSRAN_NBIOT_MODE_INBAND_SAME_PCI;
  mib.eutra_crs_seq_info = 9;
  srsran_npbch_mib_pack(frame / 1024, frame & ~63u, mib, out);
}

static void print_res(const char* tag, uint32_t f, cf_t* re)
{
  printf("%s %u", tag, f);
  for (int i = 0; i < SRSRAN_NPBCH_NUM_RE; i++) {
    printf(" %.6f %.6f", crealf(re[i]), cimagf(re[i]));
  }
  printf("\n");
}

int main(int argc, char** argv)
{
  uint32_t pci = argc > 1 ? (uint32_t)atoi(argv[1]) : 1;
  uint32_t nf  = argc > 2 ? (uint32_t)atoi(argv[2]) : 192;

  srsran_nbiot_cell_t cell = make_cell(pci);
  srsran_npbch_t      stream, stateless;
  if (srsran_npbch_init(&stream) || srsran_npbch_set_cell(&stream, cell) || srsran_npbch_init(&stateless) ||
      srsran_npbch_set_cell(&stateless, cell)) {
    fprintf(stderr, "init failed\n");
    return 1;
  }

  cf_t* grid = srsran_vec_cf_malloc(GRID_RE);
  cf_t  re[SRSRAN_NPBCH_NUM_RE];
  cf_t* sf[SRSRAN_MAX_PORTS] = {grid};
  uint8_t payload[SRSRAN_MIB_NB_LEN];

  for (uint32_t f = 0; f < nf; f++) {
    payload_for(f, payload);
    memset(grid, 0, GRID_RE * sizeof(cf_t));
    if (srsran_npbch_encode(&stream, payload, sf, f) || srsran_npbch_cp(grid, re, cell, false) != SRSRAN_NPBCH_NUM_RE) {
      fprintf(stderr, "stream encode failed\n");
      return 1;
    }
    print_res("S", f, re);
    printf("P %u", f);
    for (int i = 0; i < SRSRAN_MIB_NB_LEN; i++) {
      printf(" %d", payload[i]);
    }
    printf("\n");
  }

  // stateless, in a deterministic pseudo-random order (a full permutation of 0..nf-1)
  uint32_t* order = malloc(nf * sizeof(uint32_t));
  for (uint32_t i = 0; i < nf; i++) {
    order[i] = i;
  }
  uint32_t lcg = 12345;
  for (uint32_t i = nf - 1; i > 0; i--) {
    lcg        = lcg * 1103515245u + 12345u;
    uint32_t j = (lcg >> 8) % (i + 1);
    uint32_t t = order[i];
    order[i]   = order[j];
    order[j]   = t;
  }
  for (uint32_t k = 0; k < nf; k++) {
    uint32_t f = order[k];
    payload_for(f, payload);
    memset(grid, 0, GRID_RE * sizeof(cf_t));
    if (srsran_npbch_encode_sf(&stateless, payload, sf, f) ||
        srsran_npbch_cp(grid, re, cell, false) != SRSRAN_NPBCH_NUM_RE) {
      fprintf(stderr, "stateless encode failed\n");
      return 1;
    }
    print_res("L", f, re);
  }

  free(order);
  free(grid);
  srsran_npbch_free(&stream);
  srsran_npbch_free(&stateless);
  return 0;
}
