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

// MPDCCH format 5 round trip: encode into a subframe, then take the REs back in mapping order, descramble, decode and
// check the RNTI-masked CRC; plus the DCI 6-x sizes, the RIV and the CE mode A RAR grant.

#include "srsran/phy/common/sequence.h"
#include "srsran/phy/fec/convolutional/rm_conv.h"
#include "srsran/phy/fec/convolutional/viterbi.h"
#include "srsran/phy/modem/demod_soft.h"
#include "srsran/phy/phch/emtc.h"
#include "srsran/phy/phch/mpdcch.h"
#include "srsran/phy/utils/bit.h"
#include <complex.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(c)                                                                                                       \
  do {                                                                                                                 \
    if (!(c)) {                                                                                                        \
      fprintf(stderr, "%s:%d: check failed: %s\n", __FILE__, __LINE__, #c);                                            \
      exit(1);                                                                                                         \
    }                                                                                                                  \
  } while (0)

static int decode(srsran_mpdcch_t* q, const srsran_mpdcch_cfg_t* cfg, const cf_t* sf, uint32_t nof_bits, uint16_t* rnti)
{
  static cf_t  d[SRSRAN_MPDCCH_MAX_RE];
  static float llr[2 * SRSRAN_MPDCCH_MAX_RE];
  uint32_t     n      = 0;
  uint32_t     nof_sc = q->cell.nof_prb * SRSRAN_NRE;
  for (uint32_t l = 0; l < 14; l++) {
    for (uint32_t k = 0; k < 72; k++) {
      if (srsran_mpdcch_is_data_re(q, cfg, k, l)) {
        d[n++] = sf[l * nof_sc + cfg->first_prb * SRSRAN_NRE + k];
      }
    }
  }
  uint32_t E = 2 * n;
  srsran_demod_soft_demodulate(SRSRAN_MOD_QPSK, d, llr, n);
  uint32_t n_id = cfg->common ? q->cell.id : cfg->n_id;
  srsran_sequence_apply_f(llr, llr, E, ((cfg->sf_idx % 10) << 9) + n_id);

  float rm[3 * (SRSRAN_MPDCCH_MAX_BITS + 16)] = {};
  srsran_rm_conv_rx(llr, E, rm, 3 * (nof_bits + 16));
  srsran_viterbi_t v;
  int              poly[3] = {0x6D, 0x4F, 0x57};
  CHECK(srsran_viterbi_init(&v, SRSRAN_VITERBI_37, poly, SRSRAN_MPDCCH_MAX_BITS + 16, true) == 0);
  uint8_t data[SRSRAN_MPDCCH_MAX_BITS + 16];
  srsran_viterbi_decode_f(&v, rm, data, nof_bits + 16);
  srsran_viterbi_free(&v);
  uint8_t* x   = &data[nof_bits];
  uint16_t p   = (uint16_t)srsran_bit_pack(&x, 16);
  uint16_t crc = (uint16_t)(srsran_crc_checksum(&q->crc, data, nof_bits) & 0xffff);
  *rnti        = p ^ crc;
  return (int)n;
}

int main(void)
{
  srsran_cell_t cell = {};
  cell.nof_prb       = 25;
  cell.nof_ports     = 1;
  cell.id            = 1;
  cell.cp            = SRSRAN_CP_NORM;

  // DCI sizes for 25 PRB: 6-0A = 27 bits; 6-1A = 27 without the SRS bit, 28 with it
  CHECK(srsran_emtc_nb_bits(25) == 2);
  CHECK(srsran_emtc_dci_size(25, false) == 27);
  CHECK(srsran_emtc_dci_size(25, true) == 28);

  // RIV within a narrowband (N = 6)
  CHECK(srsran_emtc_riv(0, 1) == 0);
  CHECK(srsran_emtc_riv(0, 2) == 6);
  CHECK(srsran_emtc_riv(0, 6) == 11);
  CHECK(srsran_emtc_riv(5, 1) == 5);

  // RAR grant: NB 2, RIV 6, rep 00, MCS 2, TPC 3, no CSI/delay, MPDCCH NB offset 0, 2 bits of padding
  uint32_t g = srsran_emtc_rar_grant_ce_a(25, 2, 6, 0, 2, 3, false, false, 0);
  CHECK(g == ((2U << 18) | (6U << 14) | (0U << 12) | (2U << 9) | (3U << 6)));

  CHECK(srsran_emtc_ra_rnti(0, 1, 0) == 2);
  CHECK(srsran_emtc_ra_rnti(41, 1, 0) == 62);

  srsran_mpdcch_t q;
  CHECK(srsran_mpdcch_init(&q, cell) == 0);

  srsran_mpdcch_cfg_t cfg = {};
  cfg.first_prb           = srsran_emtc_nb_first_prb(25, 3);
  cfg.start_symbol        = 3;
  cfg.common              = true;

  // Symbols 3..13 x 72 subcarriers, minus 24 DM-RS REs per PRB, minus CRS (1 port) on symbols 4, 7 and 11
  CHECK(srsran_mpdcch_nof_re(&q, &cfg) == 6 * (11 * 12 - 24 - 3 * 2));

  static cf_t sf[14 * 25 * 12];
  for (uint32_t sf_idx = 0; sf_idx < 10; sf_idx++) {
    for (uint32_t t = 0; t < 2; t++) {
      memset(sf, 0, sizeof(sf));
      cfg.sf_idx = sf_idx;
      cfg.common = t == 0;
      cfg.n_id   = 77;

      srsran_emtc_dci_t dci = {};
      dci.nb                = 3;
      dci.riv               = srsran_emtc_riv(0, 6);
      dci.mcs               = sf_idx;
      dci.tpc               = 1;
      uint8_t  bits[SRSRAN_MPDCCH_MAX_BITS];
      uint32_t nof_bits = srsran_emtc_dci_6_1a_pack(25, true, &dci, bits);
      CHECK(nof_bits == 28);
      uint16_t rnti = (uint16_t)(0x100 + sf_idx);
      CHECK(srsran_mpdcch_encode(&q, &cfg, bits, nof_bits, rnti, sf) == 0);

      // the DM-RS of port 107 at PRB 19, l' = 0, m' = 0 sits at k = 19 * 12 + 1, l = 5
      cf_t dmrs = srsran_mpdcch_dmrs(&q, &cfg, 19, 0, 0);
      CHECK(cabsf(sf[5 * 300 + 19 * 12 + 1] - dmrs) < 1e-6);
      CHECK(cabsf(sf[5 * 300 + 19 * 12 + 0] - dmrs) < 1e-6); // port 109, same value (w = +1)

      // nothing outside the narrowband
      for (uint32_t l = 0; l < 14; l++) {
        for (uint32_t k = 0; k < 300; k++) {
          if (k < cfg.first_prb * 12 || k >= cfg.first_prb * 12 + 72) {
            CHECK(sf[l * 300 + k] == 0);
          }
        }
      }

      uint16_t got = 0;
      decode(&q, &cfg, sf, nof_bits, &got);
      CHECK(got == rnti);
    }
  }

  srsran_mpdcch_free(&q);
  printf("OK\n");
  return 0;
}
