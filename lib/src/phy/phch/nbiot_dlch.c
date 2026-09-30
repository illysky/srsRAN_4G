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

#include "srsran/phy/phch/nbiot_dlch.h"

#include <stdio.h>
#include <string.h>

#include "srsran/phy/common/sequence.h"
#include "srsran/phy/fec/convolutional/rm_conv.h"
#include "srsran/phy/modem/mod.h"
#include "srsran/phy/phch/nbiot_grid.h"

#define MAX_E (2 * SRSRAN_NBIOT_DLCH_MAX_RE * SRSRAN_NBIOT_DLCH_MAX_NSF)

/* --------------------------------------------------------------------------------------------------- tables */

// TS 36.213 Table 16.4.1.3-1
static const uint32_t n_sf_tab[8] = {1, 2, 3, 4, 5, 6, 8, 10};

// TS 36.213 Table 16.4.1.3-2
static const uint32_t n_rep_tab[16] = {1, 2, 4, 8, 16, 32, 64, 128, 192, 256, 384, 512, 768, 1024, 1536, 2048};

// TS 36.213 Table 16.4.1-1: k0 for Rmax < 128 and Rmax >= 128
static const int k0_lo[8] = {0, 4, 8, 12, 16, 32, 64, 128};
static const int k0_hi[8] = {0, 16, 32, 64, 128, 256, 512, 1024};

// TS 36.213 Table 16.4.1.5.1-1: TBS[I_TBS][I_SF]
static const uint16_t tbs_tab[14][8] = {
    {16, 32, 56, 88, 120, 152, 208, 256},     {24, 56, 88, 144, 176, 208, 256, 344},
    {32, 72, 144, 176, 208, 256, 328, 424},   {40, 104, 176, 208, 256, 328, 440, 568},
    {56, 120, 208, 256, 328, 408, 552, 680},  {72, 144, 224, 328, 424, 504, 680, 872},
    {88, 176, 256, 392, 504, 600, 808, 1032}, {104, 224, 328, 472, 584, 680, 968, 1224},
    {120, 256, 392, 536, 680, 808, 1096, 1352}, {136, 296, 456, 616, 776, 936, 1256, 1544},
    {144, 328, 504, 680, 872, 1032, 1384, 1736}, {176, 376, 584, 776, 1000, 1192, 1608, 2024},
    {208, 440, 680, 904, 1128, 1352, 1800, 2280}, {224, 488, 744, 1128, 1256, 1544, 2024, 2536}};

uint32_t srsran_nbiot_npdsch_n_sf(uint32_t i_sf)
{
  return i_sf < 8 ? n_sf_tab[i_sf] : 0;
}

uint32_t srsran_nbiot_npdsch_n_rep(uint32_t i_rep)
{
  return i_rep < 16 ? n_rep_tab[i_rep] : 0;
}

int srsran_nbiot_npdsch_k0(uint32_t i_delay, uint32_t r_max)
{
  if (i_delay > 7) {
    return -1;
  }
  return r_max < 128 ? k0_lo[i_delay] : k0_hi[i_delay];
}

uint32_t srsran_nbiot_npdsch_tbs(uint32_t i_tbs, uint32_t i_sf)
{
  return (i_tbs < 14 && i_sf < 8) ? tbs_tab[i_tbs][i_sf] : 0;
}

/* --------------------------------------------------------------------------------------------------- setup */

int srsran_nbiot_dlch_init(srsran_nbiot_dlch_t* q, const srsran_nbiot_cell_t* cell, uint32_t l_start)
{
  if (q == NULL || cell == NULL || l_start >= SRSRAN_CP_NORM_SF_NSYMB) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  if (cell->nof_ports != 1) {
    fprintf(stderr, "nbiot_dlch: one NRS port only (%u given)\n", cell->nof_ports);
    return SRSRAN_ERROR;
  }
  memset(q, 0, sizeof(*q));

  bool reserved[SRSRAN_CP_NORM_SF_NSYMB][SRSRAN_NRE];
  srsran_nbiot_reserved_res(cell, reserved);

  q->cell_id    = cell->n_id_ncell;
  q->grid_width = cell->base.nof_prb * SRSRAN_NRE;
  q->anchor_col = cell->nbiot_prb * SRSRAN_NRE;
  q->l_start    = l_start;
  q->nof_re     = 0;
  for (uint32_t l = l_start; l < SRSRAN_CP_NORM_SF_NSYMB; l++) {
    for (uint32_t k = 0; k < SRSRAN_NRE; k++) {
      if (!reserved[l][k]) {
        q->re_l[q->nof_re] = l;
        q->re_k[q->nof_re] = k;
        q->nof_re++;
      }
    }
  }

  // TS 36.212 5.1.3.1: generator polynomials 133, 171, 165 (octal), constraint length 7, tail biting
  int poly[3]    = {0x6D, 0x4F, 0x57};
  q->enc.K       = 7;
  q->enc.R       = 3;
  q->enc.tail_biting = true;
  memcpy(q->enc.poly, poly, sizeof(poly));

  if (srsran_crc_init(&q->crc16, SRSRAN_LTE_CRC16, 16) || srsran_crc_init(&q->crc24, SRSRAN_LTE_CRC24A, 24) ||
      srsran_modem_table_lte(&q->mod, SRSRAN_MOD_QPSK)) {
    srsran_nbiot_dlch_free(q);
    return SRSRAN_ERROR;
  }
  q->initiated = true;
  return SRSRAN_SUCCESS;
}

void srsran_nbiot_dlch_free(srsran_nbiot_dlch_t* q)
{
  if (q == NULL) {
    return;
  }
  srsran_modem_table_free(&q->mod);
  memset(q, 0, sizeof(*q));
}

uint32_t srsran_nbiot_dlch_bits_per_sf(const srsran_nbiot_dlch_t* q)
{
  return q ? 2 * q->nof_re : 0;
}

/* ------------------------------------------------------------------------------------------------------- DCI */

static void put_bits(uint8_t* out, uint32_t* pos, uint32_t value, uint32_t width)
{
  for (uint32_t i = 0; i < width; i++) {
    out[(*pos)++] = (value >> (width - 1 - i)) & 1;
  }
}

static uint32_t get_bits(const uint8_t* in, uint32_t* pos, uint32_t width)
{
  uint32_t v = 0;
  for (uint32_t i = 0; i < width; i++) {
    v = (v << 1) | (in[(*pos)++] & 1);
  }
  return v;
}

// TS 36.212 6.4.3.2: flag N0/N1 (1 = N1), NPDCCH order indicator (0), delay, resource assignment, MCS, repetitions, NDI,
// HARQ-ACK resource, DCI subframe repetition number
int srsran_nbiot_dci_n1_pack(const srsran_nbiot_dci_n1_t* d, uint8_t bits[SRSRAN_NBIOT_DCI_LEN])
{
  if (d == NULL || bits == NULL || d->i_delay > 7 || d->i_sf > 7 || d->i_mcs > 15 || d->i_rep > 15 || d->ndi > 1 ||
      d->harq_ack_res > 15 || d->dci_rep > 3) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  uint32_t p = 0;
  put_bits(bits, &p, 1, 1);
  put_bits(bits, &p, 0, 1);
  put_bits(bits, &p, d->i_delay, 3);
  put_bits(bits, &p, d->i_sf, 3);
  put_bits(bits, &p, d->i_mcs, 4);
  put_bits(bits, &p, d->i_rep, 4);
  put_bits(bits, &p, d->ndi, 1);
  put_bits(bits, &p, d->harq_ack_res, 4);
  put_bits(bits, &p, d->dci_rep, 2);
  return p == SRSRAN_NBIOT_DCI_LEN ? SRSRAN_SUCCESS : SRSRAN_ERROR;
}

int srsran_nbiot_dci_n1_unpack(const uint8_t bits[SRSRAN_NBIOT_DCI_LEN], srsran_nbiot_dci_n1_t* d)
{
  if (bits == NULL || d == NULL) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  uint32_t p = 0;
  if (get_bits(bits, &p, 1) != 1 || get_bits(bits, &p, 1) != 0) {
    return SRSRAN_ERROR; // not format N1, or an NPDCCH order
  }
  d->i_delay      = get_bits(bits, &p, 3);
  d->i_sf         = get_bits(bits, &p, 3);
  d->i_mcs        = get_bits(bits, &p, 4);
  d->i_rep        = get_bits(bits, &p, 4);
  d->ndi          = get_bits(bits, &p, 1);
  d->harq_ack_res = get_bits(bits, &p, 4);
  d->dci_rep      = get_bits(bits, &p, 2);
  return SRSRAN_SUCCESS;
}

// TS 36.212 6.4.3.1: flag (0 = N0), subcarrier indication, resource assignment, scheduling delay, MCS, RV, repetitions,
// NDI, DCI subframe repetition number
int srsran_nbiot_dci_n0_pack(const srsran_nbiot_dci_n0_t* d, uint8_t bits[SRSRAN_NBIOT_DCI_LEN])
{
  if (d == NULL || bits == NULL || d->i_sc > 63 || d->i_ru > 7 || d->i_delay > 3 || d->i_mcs > 15 || d->rv > 1 ||
      d->i_rep > 7 || d->ndi > 1 || d->dci_rep > 3) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  uint32_t p = 0;
  put_bits(bits, &p, 0, 1);
  put_bits(bits, &p, d->i_sc, 6);
  put_bits(bits, &p, d->i_ru, 3);
  put_bits(bits, &p, d->i_delay, 2);
  put_bits(bits, &p, d->i_mcs, 4);
  put_bits(bits, &p, d->rv, 1);
  put_bits(bits, &p, d->i_rep, 3);
  put_bits(bits, &p, d->ndi, 1);
  put_bits(bits, &p, d->dci_rep, 2);
  return p == SRSRAN_NBIOT_DCI_LEN ? SRSRAN_SUCCESS : SRSRAN_ERROR;
}

int srsran_nbiot_dci_n0_unpack(const uint8_t bits[SRSRAN_NBIOT_DCI_LEN], srsran_nbiot_dci_n0_t* d)
{
  if (bits == NULL || d == NULL) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  uint32_t p = 0;
  if (get_bits(bits, &p, 1) != 0) {
    return SRSRAN_ERROR;
  }
  d->i_sc    = get_bits(bits, &p, 6);
  d->i_ru    = get_bits(bits, &p, 3);
  d->i_delay = get_bits(bits, &p, 2);
  d->i_mcs   = get_bits(bits, &p, 4);
  d->rv      = get_bits(bits, &p, 1);
  d->i_rep   = get_bits(bits, &p, 3);
  d->ndi     = get_bits(bits, &p, 1);
  d->dci_rep = get_bits(bits, &p, 2);
  return SRSRAN_SUCCESS;
}

/* ------------------------------------------------------------------------------------------------------ mapping */

// c(i) of TS 36.211 7.2 for i = offset .. offset + n - 1, XORed onto bits[0 .. n - 1]
static void scramble(uint8_t* bits, uint32_t n, uint32_t c_init, uint32_t offset)
{
  uint8_t zeros[MAX_E + 1024];
  uint8_t seq[MAX_E + 1024];
  memset(zeros, 0, offset + n);
  srsran_sequence_apply_bit(zeros, seq, offset + n, c_init);
  for (uint32_t i = 0; i < n; i++) {
    bits[i] ^= seq[offset + i];
  }
}

static void put_symbols(const srsran_nbiot_dlch_t* q, const cf_t* sym, cf_t* grid)
{
  for (uint32_t i = 0; i < q->nof_re; i++) {
    grid[q->re_l[i] * q->grid_width + q->anchor_col + q->re_k[i]] = sym[i];
  }
}

/* ------------------------------------------------------------------------------------------------------ NPDCCH */

int srsran_nbiot_npdcch_encode(const srsran_nbiot_dlch_t* q, const uint8_t* dci, uint32_t nbits, uint16_t rnti, uint8_t* e)
{
  if (q == NULL || !q->initiated || dci == NULL || e == NULL || nbits == 0 || nbits > 64) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  uint8_t c[64 + 16];
  uint8_t d[3 * (64 + 16)];
  memcpy(c, dci, nbits);

  srsran_crc_t crc = q->crc16;
  srsran_crc_attach(&crc, c, nbits);
  // 5.3.3.2: the parity bits are scrambled with the RNTI, x_rnti,0 being its most significant bit
  for (uint32_t k = 0; k < 16; k++) {
    c[nbits + k] ^= (rnti >> (15 - k)) & 1;
  }

  srsran_convcoder_t enc = q->enc;
  srsran_convcoder_encode(&enc, c, d, nbits + 16);
  srsran_rm_conv_tx(d, 3 * (nbits + 16), e, srsran_nbiot_dlch_bits_per_sf(q));
  return SRSRAN_SUCCESS;
}

int srsran_nbiot_npdcch_put_sf(const srsran_nbiot_dlch_t* q,
                               const uint8_t*             e,
                               uint32_t                   reinit_sf,
                               uint32_t                   pos_in_group,
                               cf_t*                      grid)
{
  if (q == NULL || !q->initiated || e == NULL || grid == NULL || reinit_sf > 9 || pos_in_group > 3) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  const uint32_t n = srsran_nbiot_dlch_bits_per_sf(q);
  uint8_t        b[2 * SRSRAN_NBIOT_DLCH_MAX_RE];
  cf_t           sym[SRSRAN_NBIOT_DLCH_MAX_RE];

  memcpy(b, e, n);
  // 10.2.5.2: c_init = floor(n_s / 2) 2^9 + N_ID; the sequence runs on over the (up to) four subframes of a group
  scramble(b, n, (reinit_sf << 9) + q->cell_id, pos_in_group * n);

  srsran_mod_modulate(&q->mod, b, sym, n);
  put_symbols(q, sym, grid);
  return SRSRAN_SUCCESS;
}

/* ------------------------------------------------------------------------------------------------------ NPDSCH */

int srsran_nbiot_npdsch_encode(const srsran_nbiot_dlch_t* q, const uint8_t* tb, uint32_t tb_bits, uint32_t nof_sf, uint8_t* e)
{
  if (q == NULL || !q->initiated || tb == NULL || e == NULL || tb_bits == 0 || tb_bits > SRSRAN_NBIOT_DLCH_MAX_TB_BITS ||
      nof_sf == 0 || nof_sf > SRSRAN_NBIOT_DLCH_MAX_NSF) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  uint8_t c[SRSRAN_NBIOT_DLCH_MAX_TB_BITS + 24];
  uint8_t d[3 * (SRSRAN_NBIOT_DLCH_MAX_TB_BITS + 24)];

  for (uint32_t i = 0; i < tb_bits; i++) {
    c[i] = (tb[i / 8] >> (7 - (i % 8))) & 1;
  }
  srsran_crc_t crc = q->crc24;
  srsran_crc_attach(&crc, c, tb_bits);

  srsran_convcoder_t enc = q->enc;
  srsran_convcoder_encode(&enc, c, d, tb_bits + 24);
  srsran_rm_conv_tx(d, 3 * (tb_bits + 24), e, nof_sf * srsran_nbiot_dlch_bits_per_sf(q));
  return SRSRAN_SUCCESS;
}

int srsran_nbiot_npdsch_put_sf(const srsran_nbiot_dlch_t* q,
                               const uint8_t*             e,
                               uint32_t                   nof_sf,
                               uint32_t                   sf_in_cw,
                               uint16_t                   rnti,
                               uint32_t                   pass_sfn,
                               uint32_t                   pass_sf,
                               cf_t*                      grid)
{
  if (q == NULL || !q->initiated || e == NULL || grid == NULL || nof_sf == 0 || nof_sf > SRSRAN_NBIOT_DLCH_MAX_NSF ||
      sf_in_cw >= nof_sf || pass_sf > 9) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  const uint32_t n = srsran_nbiot_dlch_bits_per_sf(q);
  uint8_t        b[MAX_E];
  cf_t           sym[SRSRAN_NBIOT_DLCH_MAX_RE];

  // the codeword is scrambled as one block of nof_sf * n bits; this subframe carries its part sf_in_cw
  memcpy(b, e, nof_sf * n);
  scramble(b, nof_sf * n, ((uint32_t)rnti << 14) + ((pass_sfn & 1) << 13) + (pass_sf << 9) + q->cell_id, 0);

  srsran_mod_modulate(&q->mod, &b[sf_in_cw * n], sym, n);
  put_symbols(q, sym, grid);
  return SRSRAN_SUCCESS;
}
