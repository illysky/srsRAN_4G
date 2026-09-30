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

#include "srsran/phy/phch/emtc.h"
#include <stddef.h>

uint32_t srsran_emtc_nof_nb(uint32_t nof_prb)
{
  return nof_prb / SRSRAN_EMTC_NB_NOF_PRB;
}

int srsran_emtc_nb_first_prb(uint32_t nof_prb, uint32_t nb)
{
  const uint32_t n_nb = srsran_emtc_nof_nb(nof_prb);
  if (nb >= n_nb) {
    return -1;
  }
  const uint32_t i0  = nof_prb / 2 - SRSRAN_EMTC_NB_NOF_PRB * n_nb / 2;
  uint32_t       prb = SRSRAN_EMTC_NB_NOF_PRB * nb + i0;
  // With an odd number of PRBs the centre PRB belongs to no narrowband
  if (nof_prb % 2 == 1 && 2 * nb >= n_nb) {
    prb++;
  }
  return (int)prb;
}

bool srsran_emtc_nb_in_centre(uint32_t nof_prb, uint32_t nb)
{
  const int first = srsran_emtc_nb_first_prb(nof_prb, nb);
  if (first < 0) {
    return false;
  }
  // PRBs holding any of the 72 centre subcarriers: nof_prb/2 - 3 ... nof_prb/2 + 2 (+1 if nof_prb is odd)
  const int lo = (int)(nof_prb / 2) - 3;
  const int hi = (int)(nof_prb / 2) + 3 + (int)(nof_prb % 2);
  return first < hi && first + SRSRAN_EMTC_NB_NOF_PRB > lo;
}

uint64_t srsran_emtc_nb_mask(uint32_t nof_prb, uint32_t nb)
{
  const int first = srsran_emtc_nb_first_prb(nof_prb, nb);
  if (first < 0) {
    return 0;
  }
  return ((1ULL << SRSRAN_EMTC_NB_NOF_PRB) - 1) << (uint32_t)first;
}

uint32_t srsran_emtc_sib1_br_repetitions(uint32_t s)
{
  if (s < 1 || s > 18) {
    return 0;
  }
  static const uint32_t rep[3] = {4, 8, 16};
  return rep[(s - 1) % 3];
}

uint32_t srsran_emtc_sib1_br_tbs(uint32_t s)
{
  static const uint32_t tbs[19] = {0, 208, 208, 208, 256, 256, 256, 328, 328, 328, 504, 504, 504, 712, 712, 712, 936, 936, 936};
  return s < 19 ? tbs[s] : 0;
}

uint32_t srsran_emtc_rv(uint32_t k)
{
  return ((3 * k + 1) / 2) % 4;
}

uint32_t srsran_emtc_scrambling_sf(uint32_t t, uint32_t n_acc)
{
  if (n_acc == 0) {
    n_acc = 1;
  }
  return ((t / n_acc) * n_acc) % 10;
}

bool srsran_emtc_sib1_br(const srsran_emtc_bcast_cfg_t* cfg, uint32_t sfn, uint32_t sf, uint32_t* nb, uint32_t* rv)
{
  const uint32_t r   = srsran_emtc_sib1_br_repetitions(cfg->sched_info_sib1_br);
  const bool     odd = cfg->pci % 2 == 1;
  const bool     big = cfg->nof_prb > 15;
  if (r == 0 || (!big && r != 4)) {
    return false;
  }

  // Tables 6.4.1-1 and 6.4.1-2, frame structure type 1; q counts the SIB1-BR subframes of the 8-frame period
  uint32_t q;
  switch (r) {
    case 4:
      if (sfn % 2 != (odd ? 1 : 0) || sf != 4) {
        return false;
      }
      q = (sfn % 8) / 2;
      break;
    case 8:
      if (sf != (odd ? 9 : 4)) {
        return false;
      }
      q = sfn % 8;
      break;
    default: {
      const uint32_t first = odd ? 0 : 4;
      if (sf != first && sf != 9) {
        return false;
      }
      q = 2 * (sfn % 8) + (sf == 9 ? 1 : 0);
      break;
    }
  }

  // The narrowbands it cycles through: all of them, less those in the centre above 15 PRBs
  uint32_t       s[16];
  uint32_t       n_s  = 0;
  const uint32_t n_nb = srsran_emtc_nof_nb(cfg->nof_prb);
  for (uint32_t n = 0; n < n_nb && n_s < 16; n++) {
    if (!big || !srsran_emtc_nb_in_centre(cfg->nof_prb, n)) {
      s[n_s++] = n;
    }
  }
  if (n_s == 0) {
    return false;
  }
  const uint32_t m = cfg->nof_prb < 12 ? 1 : (cfg->nof_prb <= 50 ? 2 : 4);
  const uint32_t i = q % m;
  const uint32_t j = (cfg->pci % n_s + (i * n_s) / m) % n_s;
  if (nb) {
    *nb = s[j];
  }

  // TS 36.321 5.3.1
  uint32_t k;
  switch (r) {
    case 4:
      k = (sfn / 2) % 4;
      break;
    case 8:
      k = sfn % 4;
      break;
    default:
      k = (sfn * 10 + sf) % 4;
      break;
  }
  if (rv) {
    *rv = srsran_emtc_rv(k);
  }
  return true;
}

int srsran_emtc_si(const srsran_emtc_bcast_cfg_t* cfg, uint32_t sfn, uint32_t sf, uint32_t* rv)
{
  const uint32_t w   = cfg->si_window_ms;
  const uint32_t rep = cfg->si_repetition_rf ? cfg->si_repetition_rf : 1;
  for (uint32_t n = 0; n < cfg->nof_si && n < SRSRAN_EMTC_MAX_SI; n++) {
    const uint32_t t = cfg->si[n].periodicity_rf;
    if (t == 0) {
      continue;
    }
    const uint32_t start = (n * w / 10) % t; // the window starts in subframe 0 of the frame with SFN mod T = start
    const uint32_t off   = (sfn % t + t - start) % t;
    const uint32_t i     = off * 10 + sf; // subframe within the window
    if (i >= w || off % rep != 0) {
      continue;
    }
    uint32_t sib1_nb = 0;
    if (srsran_emtc_sib1_br(cfg, sfn, sf, &sib1_nb, NULL) && sib1_nb == cfg->si[n].nb) {
      continue;
    }
    if (rv) {
      *rv = srsran_emtc_rv(i % 4);
    }
    return (int)n;
  }
  return -1;
}

uint64_t srsran_emtc_bcast_prbs(const srsran_emtc_bcast_cfg_t* cfg, uint32_t sfn, uint32_t sf)
{
  uint64_t mask = 0;
  uint32_t nb   = 0;
  if (srsran_emtc_sib1_br(cfg, sfn, sf, &nb, NULL)) {
    mask |= srsran_emtc_nb_mask(cfg->nof_prb, nb);
  }
  const int si = srsran_emtc_si(cfg, sfn, sf, NULL);
  if (si >= 0) {
    mask |= srsran_emtc_nb_mask(cfg->nof_prb, cfg->si[si].nb);
  }
  return mask;
}
