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

#include "srsran/phy/phch/nbiot_sched.h"

#include <stddef.h>

// H-SFN is 10 bits and SFN is 10 bits: the frame counter the SI-window formula works on wraps after 2^20 frames, a
// multiple of every si-Periodicity (64..4096)
#define FRAME_COUNTER_MASK 0xFFFFFu

// A repetition of at most 8 subframes needs at most two frames; one more is margin
#define MAX_FRAMES_BACK 2

uint32_t srsran_nbiot_si_nof_sf(uint32_t tb_bits)
{
  return (tb_bits == 56 || tb_bits == 120) ? 2 : 8;
}

bool srsran_nbiot_sib1_locate(uint32_t                 n_id_ncell,
                              const srsran_mib_nb_t*   mib,
                              uint32_t                 sfn,
                              uint32_t                 sf_idx,
                              srsran_nbiot_bcch_pos_t* pos)
{
  if (mib == NULL || pos == NULL || sf_idx != 4 || sfn >= 1024 || mib->sched_info_sib1 > 11) {
    return false;
  }

  srsran_mib_nb_t m = *mib; // the ra_nbiot helpers take a non-const pointer

  uint32_t n_rep = (uint32_t)srsran_ra_n_rep_sib1_nb(&m);
  uint32_t first = (uint32_t)srsran_ra_nbiot_get_starting_sib1_frame(n_id_ncell, &m);
  uint32_t space = SIB1_NB_TTI / n_rep; // repetitions are equally spaced within the 256 frame (2560 ms) period

  uint32_t period = sfn / SIB1_NB_TTI;
  uint32_t rel    = sfn % SIB1_NB_TTI;
  if (rel < first) {
    return false; // before the first repetition of this period
  }

  uint32_t d   = rel - first;
  uint32_t rep = d / space;
  uint32_t off = d % space; // frames since the start of this repetition
  if (rep >= n_rep || off >= 16 || (off % 2) != 0) {
    return false; // in the gap between repetitions, or an odd frame within one
  }

  pos->start_sfn = period * SIB1_NB_TTI + first + rep * space;
  pos->sf_idx    = off / 2;
  pos->nof_sf    = 8;
  return true;
}

bool srsran_nbiot_si_locate(uint32_t                        n_id_ncell,
                            const srsran_mib_nb_t*          mib,
                            const srsran_nbiot_si_params_t* p,
                            uint32_t                        hfn,
                            uint32_t                        sfn,
                            uint32_t                        sf_idx,
                            srsran_nbiot_bcch_pos_t*        pos)
{
  if (mib == NULL || p == NULL || pos == NULL || p->n == 0 || p->si_periodicity == 0 || p->si_window_length == 0 ||
      p->si_repetition_pattern == 0 || sfn >= 1024 || sf_idx >= 10) {
    return false;
  }

  // This subframe has to be able to carry an SI message at all
  srsran_nbiot_bcch_pos_t unused;
  if (!srsran_ra_nbiot_is_valid_dl_sf(sfn * 10 + sf_idx) ||
      srsran_nbiot_sib1_locate(n_id_ncell, mib, sfn, sf_idx, &unused)) {
    return false;
  }

  uint32_t nof_sf = srsran_nbiot_si_nof_sf(p->si_tb);
  uint32_t t      = p->si_periodicity;
  uint32_t x      = (p->n - 1) * p->si_window_length;
  uint32_t base   = (x / 10 + p->si_radio_frame_offset) % t; // window start: frame counter mod T == base

  uint32_t now = ((hfn & 1023u) * 1024u + sfn) & FRAME_COUNTER_MASK;

  for (uint32_t back = 0; back <= MAX_FRAMES_BACK; back++) {
    // Could a repetition have started `back` frames ago?
    uint32_t start = (now - back) & FRAME_COUNTER_MASK;

    // Frames since the most recent SI-window start, which must be a repetition boundary inside the window
    uint32_t since = ((start % t) + t - base) % t;
    if ((since % p->si_repetition_pattern) != 0 || since * 10 >= p->si_window_length) {
      continue;
    }

    // The repetition takes the next nof_sf valid subframes beginning at subframe 0 of its start frame; count the
    // ones before this subframe
    uint32_t start_sfn = start & 1023u;
    uint32_t idx       = 0;
    for (uint32_t j = 0; j <= back && idx < nof_sf; j++) {
      uint32_t f = (start_sfn + j) & 1023u;
      for (uint32_t s = 0; s < 10; s++) {
        if (j == back && s == sf_idx) {
          if (idx < nof_sf) {
            pos->start_sfn = start_sfn;
            pos->sf_idx    = idx;
            pos->nof_sf    = nof_sf;
            return true;
          }
          break;
        }
        if (srsran_ra_nbiot_is_valid_dl_sf(f * 10 + s) && !srsran_nbiot_sib1_locate(n_id_ncell, mib, f, s, &unused)) {
          idx++;
          if (idx >= nof_sf) {
            break;
          }
        }
      }
    }
  }
  return false;
}
