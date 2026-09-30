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

#include "srsran/phy/phch/mpdcch.h"
#include "srsran/phy/common/sequence.h"
#include "srsran/phy/fec/convolutional/convcoder.h"
#include "srsran/phy/fec/convolutional/rm_conv.h"
#include "srsran/phy/modem/mod.h"
#include "srsran/phy/utils/bit.h"
#include <complex.h>
#include <math.h>
#include <string.h>

#define MPDCCH_NOF_PRB 6
#define MAX_RB_DL 110 // N_RB^max,DL of the DM-RS sequence

int srsran_mpdcch_init(srsran_mpdcch_t* q, srsran_cell_t cell)
{
  if (q == NULL || !SRSRAN_CP_ISNORM(cell.cp) || cell.nof_prb < MPDCCH_NOF_PRB) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  memset(q, 0, sizeof(*q));
  q->cell = cell;
  if (srsran_crc_init(&q->crc, SRSRAN_LTE_CRC16, 16) || srsran_modem_table_lte(&q->qpsk, SRSRAN_MOD_QPSK)) {
    return SRSRAN_ERROR;
  }
  return SRSRAN_SUCCESS;
}

void srsran_mpdcch_free(srsran_mpdcch_t* q)
{
  if (q != NULL) {
    srsran_modem_table_free(&q->qpsk);
  }
}

static bool is_dmrs(uint32_t k, uint32_t l)
{
  // Ports 107-110 (normal subframe, normal CP): subcarriers {0,1,5,6,10,11} of symbols 5,6 of both slots
  if (l != 5 && l != 6 && l != 12 && l != 13) {
    return false;
  }
  uint32_t sc = k % SRSRAN_NRE;
  return sc == 0 || sc == 1 || sc == 5 || sc == 6 || sc == 10 || sc == 11;
}

static bool is_crs(const srsran_cell_t* cell, uint32_t k_abs, uint32_t l)
{
  uint32_t v_shift = cell->id % 6;
  uint32_t ls      = l % SRSRAN_CP_NORM_NSYMB;
  uint32_t slot    = l / SRSRAN_CP_NORM_NSYMB;
  uint32_t v[2];
  uint32_t n = 0;
  if (ls == 0) {
    v[n++] = 0;
    if (cell->nof_ports > 1) {
      v[n++] = 3;
    }
  } else if (ls == 4) {
    v[n++] = 3;
    if (cell->nof_ports > 1) {
      v[n++] = 0;
    }
  } else if (ls == 1 && cell->nof_ports == 4) {
    v[n++] = 3 * slot;
    v[n++] = 3 + 3 * slot;
  }
  for (uint32_t i = 0; i < n; i++) {
    if ((k_abs + 6 - (v[i] + v_shift) % 6) % 6 == 0) {
      return true;
    }
  }
  return false;
}

// PSS/SSS/PBCH REs: counted in the MPDCCH mapping but not transmitted (TS 36.211 6.8B.5)
static bool is_sync_or_pbch(const srsran_cell_t* cell, uint32_t sf_idx, uint32_t k_abs, uint32_t l)
{
  uint32_t lo = cell->nof_prb * SRSRAN_NRE / 2 - 36;
  if (k_abs < lo || k_abs >= lo + 72) {
    return false;
  }
  if ((sf_idx == 0 || sf_idx == 5) && (l == 5 || l == 6)) {
    return true;
  }
  return sf_idx == 0 && l >= 7 && l <= 10;
}

bool srsran_mpdcch_is_data_re(const srsran_mpdcch_t* q, const srsran_mpdcch_cfg_t* cfg, uint32_t k, uint32_t l)
{
  return l >= cfg->start_symbol && !is_dmrs(k, l) && !is_crs(&q->cell, cfg->first_prb * SRSRAN_NRE + k, l);
}

uint32_t srsran_mpdcch_nof_re(const srsran_mpdcch_t* q, const srsran_mpdcch_cfg_t* cfg)
{
  uint32_t n = 0;
  for (uint32_t l = 0; l < SRSRAN_CP_NORM_NSYMB * 2; l++) {
    for (uint32_t k = 0; k < MPDCCH_NOF_PRB * SRSRAN_NRE; k++) {
      n += srsran_mpdcch_is_data_re(q, cfg, k, l) ? 1 : 0;
    }
  }
  return n;
}

int srsran_mpdcch_dci_encode(srsran_mpdcch_t* q, const uint8_t* dci, uint32_t nof_bits, uint16_t rnti, uint8_t* e, uint32_t E)
{
  if (q == NULL || dci == NULL || e == NULL || nof_bits == 0 || nof_bits > SRSRAN_MPDCCH_MAX_BITS) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  uint8_t data[SRSRAN_MPDCCH_MAX_BITS + 16];
  uint8_t coded[3 * (SRSRAN_MPDCCH_MAX_BITS + 16)];
  memcpy(data, dci, nof_bits);
  srsran_crc_attach(&q->crc, data, nof_bits);
  uint8_t  mask[16];
  uint8_t* m = mask;
  srsran_bit_unpack(rnti, &m, 16);
  for (uint32_t i = 0; i < 16; i++) {
    data[nof_bits + i] ^= mask[i];
  }

  srsran_convcoder_t enc = {};
  int                poly[3] = {0x6D, 0x4F, 0x57};
  enc.K                      = 7;
  enc.R                      = 3;
  enc.tail_biting            = true;
  memcpy(enc.poly, poly, sizeof(poly));
  srsran_convcoder_encode(&enc, data, coded, nof_bits + 16);
  return srsran_rm_conv_tx(coded, 3 * (nof_bits + 16), e, E);
}

static void dmrs_seq(const srsran_mpdcch_t* q, const srsran_mpdcch_cfg_t* cfg, float* r, uint32_t len)
{
  uint32_t n_id  = cfg->common ? q->cell.id : cfg->n_id;
  uint32_t c_init = (((cfg->sf_idx % 10) + 1) * (2 * n_id + 1) << 16) + 2; // n_SCID^MPDCCH = 2
  srsran_sequence_state_t s;
  srsran_sequence_state_init(&s, c_init);
  srsran_sequence_state_gen_f(&s, (float)M_SQRT1_2, r, len);
}

cf_t srsran_mpdcch_dmrs(const srsran_mpdcch_t* q, const srsran_mpdcch_cfg_t* cfg, uint32_t n_prb, uint32_t lp, uint32_t mp)
{
  float    r[2 * 12 * MAX_RB_DL];
  dmrs_seq(q, cfg, r, 2 * 12 * MAX_RB_DL);
  uint32_t m = 3 * lp * MAX_RB_DL + 3 * n_prb + mp;
  return r[2 * m] + I * r[2 * m + 1];
}

int srsran_mpdcch_encode(srsran_mpdcch_t*           q,
                         const srsran_mpdcch_cfg_t* cfg,
                         const uint8_t*             dci,
                         uint32_t                   nof_bits,
                         uint16_t                   rnti,
                         cf_t*                      sf_symbols)
{
  if (q == NULL || cfg == NULL || sf_symbols == NULL || cfg->first_prb + MPDCCH_NOF_PRB > q->cell.nof_prb ||
      cfg->start_symbol < 1 || cfg->start_symbol > 4) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  const uint32_t nof_re = srsran_mpdcch_nof_re(q, cfg);
  const uint32_t E      = 2 * nof_re;
  if (srsran_mpdcch_dci_encode(q, dci, nof_bits, rnti, q->e, E) < 0) {
    return SRSRAN_ERROR;
  }

  uint32_t n_id = cfg->common ? q->cell.id : cfg->n_id;
  srsran_sequence_apply_bit(q->e, q->e, E, ((cfg->sf_idx % 10) << 9) + n_id);
  srsran_mod_modulate(&q->qpsk, q->e, q->d, E);

  const uint32_t nof_sc = q->cell.nof_prb * SRSRAN_NRE;
  uint32_t       i      = 0;
  for (uint32_t l = 0; l < SRSRAN_CP_NORM_NSYMB * 2; l++) {
    for (uint32_t k = 0; k < MPDCCH_NOF_PRB * SRSRAN_NRE; k++) {
      if (!srsran_mpdcch_is_data_re(q, cfg, k, l)) {
        continue;
      }
      uint32_t k_abs = cfg->first_prb * SRSRAN_NRE + k;
      if (!is_sync_or_pbch(&q->cell, cfg->sf_idx, k_abs, l)) {
        sf_symbols[l * nof_sc + k_abs] = q->d[i];
      }
      i++;
    }
  }

  // DM-RS, ports 107 (k' = 1) and 109 (k' = 0), both w = +1: r(3 l' N_RB^max + 3 n_PRB + m') at k = 5 m' + 12 n_PRB + k'
  float r[2 * 12 * MAX_RB_DL];
  dmrs_seq(q, cfg, r, 2 * 12 * MAX_RB_DL);
  static const uint32_t l_of[4] = {5, 6, 12, 13};
  for (uint32_t p = 0; p < MPDCCH_NOF_PRB; p++) {
    uint32_t n_prb = cfg->first_prb + p;
    for (uint32_t lp = 0; lp < 4; lp++) {
      uint32_t l = l_of[lp];
      for (uint32_t mp = 0; mp < 3; mp++) {
        uint32_t m   = 3 * lp * MAX_RB_DL + 3 * n_prb + mp;
        cf_t     val = r[2 * m] + I * r[2 * m + 1];
        for (uint32_t kp = 0; kp < 2; kp++) {
          uint32_t k_abs = 5 * mp + SRSRAN_NRE * n_prb + kp;
          if (!is_sync_or_pbch(&q->cell, cfg->sf_idx, k_abs, l)) {
            sf_symbols[l * nof_sc + k_abs] = val;
          }
        }
      }
    }
  }
  return SRSRAN_SUCCESS;
}
