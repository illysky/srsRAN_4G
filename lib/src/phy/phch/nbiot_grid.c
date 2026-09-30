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

#include "srsran/phy/phch/nbiot_grid.h"

#include <complex.h>
#include <math.h>
#include <string.h>

void srsran_nbiot_reserved_res(const srsran_nbiot_cell_t* cell, bool reserved[SRSRAN_CP_NORM_SF_NSYMB][SRSRAN_NRE])
{
  memset(reserved, 0, sizeof(bool) * SRSRAN_CP_NORM_SF_NSYMB * SRSRAN_NRE);

  // NRS, TS 36.211 10.2.6.2: symbols N_symb - 2 and N_symb - 1 of both slots, ports 2000 (v = 0, 3) and 2001 (v = 3, 0)
  uint32_t vshift_nrs = cell->n_id_ncell % 6;
  for (uint32_t p = 0; p < cell->nof_ports && p < 2; p++) {
    for (uint32_t slot = 0; slot < 2; slot++) {
      for (uint32_t s = 5; s <= 6; s++) {
        uint32_t v = (p == 0) ? (s == 5 ? 0 : 3) : (s == 5 ? 3 : 0);
        for (uint32_t m = 0; m < 2; m++) {
          reserved[slot * SRSRAN_CP_NORM_NSYMB + s][6 * m + (v + vshift_nrs) % 6] = true;
        }
      }
    }
  }

  // LTE CRS, TS 36.211 6.10.1.2, only where the LTE carrier is there (in-band)
  if (cell->mode == SRSRAN_NBIOT_MODE_INBAND_SAME_PCI || cell->mode == SRSRAN_NBIOT_MODE_INBAND_DIFFERENT_PCI) {
    uint32_t vshift_crs = cell->base.id % 6;
    for (uint32_t p = 0; p < cell->base.nof_ports && p < 4; p++) {
      for (uint32_t slot = 0; slot < 2; slot++) {
        uint32_t syms[2];
        uint32_t nsyms = 0;
        if (p < 2) {
          syms[nsyms++] = 0;
          syms[nsyms++] = SRSRAN_CP_NORM_NSYMB - 3; // 4
        } else {
          syms[nsyms++] = 1;
        }
        for (uint32_t i = 0; i < nsyms; i++) {
          uint32_t v;
          switch (p) {
            case 0:
              v = (syms[i] == 0) ? 0 : 3;
              break;
            case 1:
              v = (syms[i] == 0) ? 3 : 0;
              break;
            case 2:
              v = 3 * slot;
              break;
            default:
              v = 3 + 3 * slot;
              break;
          }
          for (uint32_t m = 0; m < 2; m++) {
            reserved[slot * SRSRAN_CP_NORM_NSYMB + syms[i]][6 * m + (v + vshift_crs) % 6] = true;
          }
        }
      }
    }
  }
}

uint32_t srsran_nbiot_grid_nof_data_re(const srsran_nbiot_cell_t* cell, uint32_t l_start)
{
  bool reserved[SRSRAN_CP_NORM_SF_NSYMB][SRSRAN_NRE];
  srsran_nbiot_reserved_res(cell, reserved);

  uint32_t n = 0;
  for (uint32_t l = l_start; l < SRSRAN_CP_NORM_SF_NSYMB; l++) {
    for (uint32_t k = 0; k < SRSRAN_NRE; k++) {
      if (!reserved[l][k]) {
        n++;
      }
    }
  }
  return n;
}

double srsran_nbiot_inband_freq_offset_sc(uint32_t nof_prb, uint32_t nbiot_prb)
{
  // TS 36.211 6.12: LTE subcarrier index i sits at (i - N_sc/2) below DC and (i - N_sc/2 + 1) above it (DC unused)
  const int half = (int)(nof_prb * SRSRAN_NRE) / 2;
  double    sum  = 0.0;
  for (int k = 0; k < SRSRAN_NRE; k++) {
    int i = (int)(nbiot_prb * SRSRAN_NRE) + k;
    sum += (i < half) ? (i - half) : (i - half + 1);
  }
  return sum / SRSRAN_NRE;
}

cf_t srsran_nbiot_inband_phase(uint32_t nof_prb, uint32_t nbiot_prb, uint32_t sf_idx, uint32_t l)
{
  // theta = 2 pi f_NB-IoT Ts (l' N + sum_{i=0..l'} N_CP,(i mod 7)), l' counted from the last even-numbered subframe,
  // N = 2048 and Ts = 1 / (2048 * 15 kHz): so f_NB-IoT Ts = offset_sc / 2048
  const double   f     = srsran_nbiot_inband_freq_offset_sc(nof_prb, nbiot_prb);
  const uint32_t l_abs = l + SRSRAN_CP_NORM_SF_NSYMB * (sf_idx % 2);
  double         cp    = 0.0;
  for (uint32_t i = 0; i <= l_abs; i++) {
    cp += (i % SRSRAN_CP_NORM_NSYMB == 0) ? 160.0 : 144.0;
  }
  double cycles = f * (double)l_abs + f * cp / 2048.0;
  cycles -= floor(cycles);
  return cexpf(I * (float)(2.0 * M_PI * cycles));
}

void srsran_nbiot_inband_rotate(const srsran_nbiot_cell_t* cell,
                                uint32_t                   sf_idx,
                                uint32_t                   l_start,
                                bool                       inverse,
                                cf_t*                      sf_symbols)
{
  if (cell->mode != SRSRAN_NBIOT_MODE_INBAND_SAME_PCI && cell->mode != SRSRAN_NBIOT_MODE_INBAND_DIFFERENT_PCI) {
    return;
  }
  const uint32_t w = cell->base.nof_prb * SRSRAN_NRE;
  for (uint32_t l = l_start; l < SRSRAN_CP_NORM_SF_NSYMB; l++) {
    cf_t ph = srsran_nbiot_inband_phase(cell->base.nof_prb, cell->nbiot_prb, sf_idx, l);
    if (inverse) {
      ph = conjf(ph);
    }
    cf_t* re = &sf_symbols[l * w + cell->nbiot_prb * SRSRAN_NRE];
    for (uint32_t k = 0; k < SRSRAN_NRE; k++) {
      re[k] *= ph;
    }
  }
}
