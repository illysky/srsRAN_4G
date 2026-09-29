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

#include "srsran/phy/phch/nbiot_ra.h"
#include "srsran/phy/utils/debug.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>

// TS 36.213 Table 16.5.1-1 with the RAR change of 16.3.3 (k0 = 12 for I_delay = 0)
static const uint32_t k0_rar[4]  = {12, 16, 32, 64};
// TS 36.213 Table 16.5.1.1-3
static const uint32_t n_rep_tab[8] = {1, 2, 4, 8, 16, 32, 64, 128};

uint32_t srsran_nbiot_ra_rnti(uint32_t sfn_id, uint32_t carrier_id)
{
  return 1 + sfn_id / 4 + 256 * carrier_id;
}

uint32_t srsran_nbiot_ta_from_toa(float toa_samples)
{
  if (!(toa_samples > 0.0f)) { // also catches NaN
    return 0;
  }
  float r = roundf(toa_samples);
  if (r > (float)SRSRAN_NBIOT_RAR_MAX_TA) {
    return SRSRAN_NBIOT_RAR_MAX_TA;
  }
  return (uint32_t)r;
}

/// Tones, first tone and validity for a subcarrier indication (TS 36.213 16.5.1.1). Returns -1 for reserved values.
static int decode_isc(uint32_t sc_15khz, uint32_t i_sc, uint32_t* n_tones, uint32_t* first)
{
  if (!sc_15khz) {
    if (i_sc > 47) { // 48..63 reserved
      return -1;
    }
    *n_tones = 1;
    *first   = i_sc;
    return 0;
  }
  if (i_sc <= 11) {
    *n_tones = 1;
    *first   = i_sc;
  } else if (i_sc <= 15) {
    *n_tones = 3;
    *first   = 3 * (i_sc - 12);
  } else if (i_sc <= 17) {
    *n_tones = 6;
    *first   = 6 * (i_sc - 16);
  } else if (i_sc == 18) {
    *n_tones = 12;
    *first   = 0;
  } else {
    return -1; // 19..63 reserved
  }
  return 0;
}

int srsran_nbiot_msg3_grant_check(const srsran_nbiot_msg3_grant_t* g)
{
  uint32_t n, f;
  if (g == NULL || g->sc_15khz > 1 || g->i_delay > 3 || g->i_rep > 7 || g->i_mcs > 2) {
    return SRSRAN_ERROR;
  }
  return decode_isc(g->sc_15khz, g->i_sc, &n, &f) == 0 ? SRSRAN_SUCCESS : SRSRAN_ERROR;
}

int srsran_nbiot_msg3_grant_pack(const srsran_nbiot_msg3_grant_t* g, uint32_t* bits15)
{
  if (bits15 == NULL || srsran_nbiot_msg3_grant_check(g)) {
    return SRSRAN_ERROR;
  }
  // 1 | 6 | 2 | 3 | 3, most significant first
  *bits15 = (g->sc_15khz << 14) | (g->i_sc << 8) | (g->i_delay << 6) | (g->i_rep << 3) | g->i_mcs;
  return SRSRAN_SUCCESS;
}

void srsran_nbiot_msg3_grant_unpack(uint32_t bits15, srsran_nbiot_msg3_grant_t* g)
{
  g->sc_15khz = (bits15 >> 14) & 0x1;
  g->i_sc     = (bits15 >> 8) & 0x3f;
  g->i_delay  = (bits15 >> 6) & 0x3;
  g->i_rep    = (bits15 >> 3) & 0x7;
  g->i_mcs    = bits15 & 0x7;
}

int srsran_nbiot_msg3_grant_to_npusch(const srsran_nbiot_msg3_grant_t* g,
                                      uint32_t                         tc_rnti,
                                      uint32_t                         cell_id,
                                      srsran_npusch_cfg_t*             cfg,
                                      uint32_t*                        k0)
{
  uint32_t n_tones, first;
  if (cfg == NULL || k0 == NULL || srsran_nbiot_msg3_grant_check(g) || decode_isc(g->sc_15khz, g->i_sc, &n_tones, &first)) {
    return SRSRAN_ERROR;
  }
  memset(cfg, 0, sizeof(*cfg));
  cfg->n_sc       = n_tones;
  cfg->spacing_hz = g->sc_15khz ? 15000 : 3750;
  cfg->sc         = first;
  cfg->n_rep      = n_rep_tab[g->i_rep];
  cfg->rv         = 0;
  cfg->tbs        = SRSRAN_NBIOT_MSG3_TBS;
  cfg->rnti       = tc_rnti;
  cfg->cell_id    = cell_id;
  cfg->base_seq   = -1;

  // Table 16.3.3-1: I_MCS 0: pi/2 BPSK, 4 RU; 1: pi/4 QPSK, 3 RU; 2: pi/4 QPSK, 1 RU.
  // With more than one tone the modulation is QPSK for every entry.
  static const uint32_t n_ru_tab[3] = {4, 3, 1};
  cfg->n_ru = n_ru_tab[g->i_mcs];
  cfg->qm   = (n_tones == 1 && g->i_mcs == 0) ? 1 : 2;

  *k0 = k0_rar[g->i_delay];
  return SRSRAN_SUCCESS;
}

int srsran_nbiot_rar_pdu_pack(int backoff_ind, const srsran_nbiot_rar_t* rar, uint32_t nof_rar, uint8_t* out, uint32_t max_len)
{
  if (out == NULL || (nof_rar > 0 && rar == NULL) || backoff_ind > 15) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  if (backoff_ind < 0 && nof_rar == 0) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  if (nof_rar > SRSRAN_NBIOT_RAR_MAX_PER_PDU) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  const uint32_t n_sub = nof_rar + (backoff_ind >= 0 ? 1 : 0);
  const uint32_t total = n_sub + nof_rar * SRSRAN_NBIOT_RAR_LEN;
  if (total > max_len) {
    return SRSRAN_ERROR;
  }

  uint8_t* p = out;
  uint32_t sub = 0;
  if (backoff_ind >= 0) {
    // E/T/R/R/BI: T = 0
    *p++ = (uint8_t)(((++sub < n_sub) ? 0x80 : 0x00) | (backoff_ind & 0x0f));
  }
  for (uint32_t i = 0; i < nof_rar; ++i) {
    if (rar[i].rapid > 47 || rar[i].ta > SRSRAN_NBIOT_RAR_MAX_TA || rar[i].tc_rnti > 0xffff) {
      return SRSRAN_ERROR;
    }
    // E/T/RAPID: T = 1
    *p++ = (uint8_t)(((++sub < n_sub) ? 0x80 : 0x00) | 0x40 | (rar[i].rapid & 0x3f));
  }
  for (uint32_t i = 0; i < nof_rar; ++i) {
    uint32_t grant;
    if (srsran_nbiot_msg3_grant_pack(&rar[i].grant, &grant)) {
      return SRSRAN_ERROR;
    }
    // Figure 6.1.5-3b: R | TA[10:4] / TA[3:0] | grant[14:11] / grant[10:3] / grant[2:0] | R R R R R / TC-RNTI
    p[0] = (uint8_t)((rar[i].ta >> 4) & 0x7f);
    p[1] = (uint8_t)(((rar[i].ta & 0x0f) << 4) | ((grant >> 11) & 0x0f));
    p[2] = (uint8_t)((grant >> 3) & 0xff);
    p[3] = (uint8_t)((grant & 0x07) << 5);
    p[4] = (uint8_t)(rar[i].tc_rnti >> 8);
    p[5] = (uint8_t)(rar[i].tc_rnti & 0xff);
    p += SRSRAN_NBIOT_RAR_LEN;
  }
  return (int)total;
}

int srsran_nbiot_rar_pdu_unpack(const uint8_t* pdu, uint32_t len, int* backoff_ind, srsran_nbiot_rar_t* rar, uint32_t max_rar)
{
  if (pdu == NULL || backoff_ind == NULL || (max_rar > 0 && rar == NULL)) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  *backoff_ind = -1;

  uint32_t rapid[SRSRAN_NBIOT_RAR_MAX_PER_PDU];
  uint32_t nof_rar = 0;
  uint32_t pos     = 0;
  bool     ext     = true;
  bool     first   = true;
  while (ext) {
    if (pos >= len) {
      return SRSRAN_ERROR; // header runs off the end
    }
    uint8_t h = pdu[pos++];
    ext       = (h & 0x80) != 0;
    if (h & 0x40) { // T = 1: RAPID
      if (nof_rar >= SRSRAN_NBIOT_RAR_MAX_PER_PDU || nof_rar >= max_rar) {
        return SRSRAN_ERROR;
      }
      rapid[nof_rar++] = h & 0x3f;
    } else { // T = 0: Backoff Indicator, only allowed as the first subheader
      if (!first) {
        return SRSRAN_ERROR;
      }
      *backoff_ind = h & 0x0f;
    }
    first = false;
  }
  if (len - pos < nof_rar * SRSRAN_NBIOT_RAR_LEN) {
    return SRSRAN_ERROR; // truncated payload
  }
  for (uint32_t i = 0; i < nof_rar; ++i) {
    const uint8_t* p = &pdu[pos];
    uint32_t       g = ((uint32_t)(p[1] & 0x0f) << 11) | ((uint32_t)p[2] << 3) | (uint32_t)(p[3] >> 5);
    rar[i].rapid     = rapid[i];
    rar[i].ta        = ((uint32_t)(p[0] & 0x7f) << 4) | (uint32_t)(p[1] >> 4);
    srsran_nbiot_msg3_grant_unpack(g, &rar[i].grant);
    rar[i].tc_rnti = ((uint32_t)p[4] << 8) | p[5];
    pos += SRSRAN_NBIOT_RAR_LEN;
  }
  return (int)nof_rar;
}
