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

/******************************************************************************
 *  File:         mpdcch.h
 *
 *  Description:  MTC physical downlink control channel (TS 36.211 6.8B, 6.10.3A), transmit side.
 *
 *                Only MPDCCH format 5 is implemented: aggregation level 24 over the 2+4 PRB set, i.e. every EREG of
 *                the six PRBs of a narrowband. That is the Type2 common search space candidate at CE level 0/1
 *                (TS 36.213 9.1.5, Table 9.1.5-1b) and the UE-specific candidate when the MPDCCH-PRB-set is 6 PRBs.
 *                Distributed transmission: DM-RS on ports 107 and 109. CE mode A only (N_acc = 1).
 *****************************************************************************/

#ifndef SRSRAN_MPDCCH_H
#define SRSRAN_MPDCCH_H

#include "srsran/config.h"
#include "srsran/phy/common/phy_common.h"
#include "srsran/phy/fec/crc.h"
#include "srsran/phy/modem/modem_table.h"
#include <stdbool.h>
#include <stdint.h>

#define SRSRAN_MPDCCH_MAX_RE (6 * SRSRAN_NRE * 2 * SRSRAN_CP_NORM_NSYMB)
#define SRSRAN_MPDCCH_MAX_BITS 64 // DCI payload

typedef struct SRSRAN_API {
  srsran_cell_t        cell;
  srsran_crc_t         crc;
  srsran_modem_table_t qpsk;
  uint8_t              e[2 * SRSRAN_MPDCCH_MAX_RE];
  cf_t                 d[SRSRAN_MPDCCH_MAX_RE];
} srsran_mpdcch_t;

typedef struct SRSRAN_API {
  uint32_t first_prb;    ///< lowest PRB of the narrowband
  uint32_t start_symbol; ///< l_MPDCCHStart (startSymbolBR)
  uint32_t sf_idx;       ///< subframe number (RE mapping; scrambling and DM-RS too unless scrambling_sf_set)
  bool     scrambling_sf_set;
  uint32_t scrambling_sf; ///< (j0 N_acc) mod 10 of TS 36.211 6.8B.2 / 6.10.3A.1 when N_acc > 1 (P-RNTI, SC-RNTI)
  bool     common;       ///< Type1/Type2 common search space: scrambled with the PCI instead of n_ID
  uint32_t n_id;         ///< n_ID^MPDCCH (UE-specific search space)
} srsran_mpdcch_cfg_t;

SRSRAN_API int srsran_mpdcch_init(srsran_mpdcch_t* q, srsran_cell_t cell);

SRSRAN_API void srsran_mpdcch_free(srsran_mpdcch_t* q);

/// Number of REs that carry MPDCCH format 5 with this configuration (E = 2 * this)
SRSRAN_API uint32_t srsran_mpdcch_nof_re(const srsran_mpdcch_t* q, const srsran_mpdcch_cfg_t* cfg);

/// True if (k, l) of the subframe is an MPDCCH data RE of the narrowband starting at cfg->first_prb (k relative to it)
SRSRAN_API bool srsran_mpdcch_is_data_re(const srsran_mpdcch_t* q, const srsran_mpdcch_cfg_t* cfg, uint32_t k, uint32_t l);

/// DCI + RNTI-masked CRC, tail-biting convolutional code, rate matching to E bits (TS 36.212 5.3.3.2-5.3.3.4)
SRSRAN_API int
srsran_mpdcch_dci_encode(srsran_mpdcch_t* q, const uint8_t* dci, uint32_t nof_bits, uint16_t rnti, uint8_t* e, uint32_t E);

/// Encodes one DCI (unpacked bits) and maps it, with its DM-RS, into sf_symbols (one antenna port)
SRSRAN_API int srsran_mpdcch_encode(srsran_mpdcch_t*           q,
                                    const srsran_mpdcch_cfg_t* cfg,
                                    const uint8_t*             dci,
                                    uint32_t                   nof_bits,
                                    uint16_t                   rnti,
                                    cf_t*                      sf_symbols);

/// DM-RS value for PRB n_prb (absolute), l' (0..3) and m' (0..2); same for ports 107 and 109 (w = +1)
SRSRAN_API cf_t srsran_mpdcch_dmrs(const srsran_mpdcch_t* q, const srsran_mpdcch_cfg_t* cfg, uint32_t n_prb, uint32_t lp, uint32_t mp);

#endif // SRSRAN_MPDCCH_H
