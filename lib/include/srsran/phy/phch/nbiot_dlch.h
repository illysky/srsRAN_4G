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
 *  File:         nbiot_dlch.h
 *
 *  Description:  NB-IoT downlink channels of the eNodeB that carry addressed traffic: NPDCCH (DCI formats N0 and N1)
 *                and NPDSCH scrambled by an RNTI (random access response, Msg4, user data). One subframe at a time and
 *                without state, so that a composer running on any thread can produce any subframe of a transmission
 *                from the transmission's description alone.
 *
 *                Scope: in-band anchor PRB of an LTE grid, one antenna port, normal CP. The resource elements taken
 *                by NRS and by the LTE CRS follow from srsran_nbiot_reserved_res().
 *
 *  Reference:    3GPP TS 36.212 v14.2.0 5.1.1, 5.1.3.1, 5.1.4.2, 5.3.3, 6.4.2, 6.4.3
 *                3GPP TS 36.211 v14.2.0 6.8.2 (via 10.2.5.2), 6.3, 10.2.3, 10.2.5
 *                3GPP TS 36.213 v14.2.0 16.4.1, 16.5.1, 16.6
 *****************************************************************************/

#ifndef SRSRAN_NBIOT_DLCH_H
#define SRSRAN_NBIOT_DLCH_H

#include <stdbool.h>
#include <stdint.h>

#include "srsran/config.h"
#include "srsran/phy/common/phy_common.h"
#include "srsran/phy/fec/convolutional/convcoder.h"
#include "srsran/phy/fec/crc.h"
#include "srsran/phy/modem/modem_table.h"

/// DCI formats N0 and N1 both have 23 bits (TS 36.212 6.4.3.1, 6.4.3.2, one HARQ process)
#define SRSRAN_NBIOT_DCI_LEN 23

/// Resource elements of one anchor PRB after l_start; more than any real configuration leaves
#define SRSRAN_NBIOT_DLCH_MAX_RE (SRSRAN_CP_NORM_SF_NSYMB * SRSRAN_NRE)

/// Largest transport block srsran_nbiot_npdsch_encode() accepts: the convolutional rate matcher of the library holds a
/// 32 x 32 sub-block interleaver, i.e. 1024 bits per stream = 1000 bits plus the CRC. Rel-13 NB-IoT UEs (Cat-NB1, nRF91)
/// receive at most 680 bits, which is far below. Table 16.4.1.5.1-1 goes up to 2536 for Cat-NB2.
#define SRSRAN_NBIOT_DLCH_MAX_TB_BITS 1000

/// Subframes of one NPDSCH codeword: N_SF <= 10 (Table 16.4.1.3-1)
#define SRSRAN_NBIOT_DLCH_MAX_NSF 10

typedef struct SRSRAN_API {
  uint32_t cell_id;    ///< N_ID^Ncell
  uint32_t grid_width; ///< subcarriers of the host grid (12 * PRBs)
  uint32_t anchor_col; ///< first subcarrier of the anchor PRB in the host grid
  uint32_t l_start;    ///< l_DataStart = l_NPDCCHStart (eutraControlRegionSize)
  uint32_t nof_re;     ///< resource elements per subframe available to NPDSCH and NPDCCH format 1
  uint8_t  re_l[SRSRAN_NBIOT_DLCH_MAX_RE]; ///< symbol of each of them, in mapping order (k first, then l)
  uint8_t  re_k[SRSRAN_NBIOT_DLCH_MAX_RE]; ///< subcarrier

  srsran_crc_t         crc16;
  srsran_crc_t         crc24;
  srsran_convcoder_t   enc;
  srsran_modem_table_t mod;
  bool                 initiated;
} srsran_nbiot_dlch_t;

/**
 * cell->nof_ports must be 1. Fails (message printed) for anything outside the scope above.
 * grid_width is the width of the LTE grid the anchor sits in, in subcarriers.
 */
SRSRAN_API int srsran_nbiot_dlch_init(srsran_nbiot_dlch_t* q, const srsran_nbiot_cell_t* cell, uint32_t l_start);

SRSRAN_API void srsran_nbiot_dlch_free(srsran_nbiot_dlch_t* q);

/// Coded bits of one subframe of NPDSCH or of an NPDCCH format 1 (both NCCEs): two per resource element
SRSRAN_API uint32_t srsran_nbiot_dlch_bits_per_sf(const srsran_nbiot_dlch_t* q);

/* ---------------------------------------------------------------------------------------------------- DCI fields */

/** DCI format N1 with NPDCCH order indicator 0 (TS 36.212 6.4.3.2). Field widths in brackets. */
typedef struct SRSRAN_API {
  uint32_t i_delay;      ///< scheduling delay (3), Table 16.4.1-1
  uint32_t i_sf;         ///< resource assignment: N_SF (3), Table 16.4.1.3-1
  uint32_t i_mcs;        ///< modulation and coding scheme = I_TBS (4)
  uint32_t i_rep;        ///< repetition number: N_Rep (4), Table 16.4.1.3-2
  uint32_t ndi;          ///< new data indicator (1); reserved (sent as given) for RA-RNTI
  uint32_t harq_ack_res; ///< HARQ-ACK resource (4); reserved (sent as given) for RA-RNTI
  uint32_t dci_rep;      ///< DCI subframe repetition number (2), Table 16.6-1 / 16.6-3
} srsran_nbiot_dci_n1_t;

/** DCI format N0 (TS 36.212 6.4.3.1). */
typedef struct SRSRAN_API {
  uint32_t i_sc;    ///< subcarrier indication (6), 16.5.1.1
  uint32_t i_ru;    ///< resource assignment: resource units N_RU (3), Table 16.5.1.1-2
  uint32_t i_delay; ///< scheduling delay (2), Table 16.5.1-1
  uint32_t i_mcs;   ///< modulation and coding scheme (4)
  uint32_t rv;      ///< redundancy version (1)
  uint32_t i_rep;   ///< repetition number (3), Table 16.5.1.1-3
  uint32_t ndi;     ///< new data indicator (1)
  uint32_t dci_rep; ///< DCI subframe repetition number (2)
} srsran_nbiot_dci_n0_t;

/// Field values are range-checked; returns SRSRAN_ERROR_INVALID_INPUTS for one that does not fit. bits: one per byte.
SRSRAN_API int srsran_nbiot_dci_n1_pack(const srsran_nbiot_dci_n1_t* d, uint8_t bits[SRSRAN_NBIOT_DCI_LEN]);
SRSRAN_API int srsran_nbiot_dci_n1_unpack(const uint8_t bits[SRSRAN_NBIOT_DCI_LEN], srsran_nbiot_dci_n1_t* d);
SRSRAN_API int srsran_nbiot_dci_n0_pack(const srsran_nbiot_dci_n0_t* d, uint8_t bits[SRSRAN_NBIOT_DCI_LEN]);
SRSRAN_API int srsran_nbiot_dci_n0_unpack(const uint8_t bits[SRSRAN_NBIOT_DCI_LEN], srsran_nbiot_dci_n0_t* d);

/* -------------------------------------------------------------------------------------------------- transport blocks */

/// N_SF of Table 16.4.1.3-1, or 0 for an index that does not exist
SRSRAN_API uint32_t srsran_nbiot_npdsch_n_sf(uint32_t i_sf);

/// N_Rep of Table 16.4.1.3-2, or 0
SRSRAN_API uint32_t srsran_nbiot_npdsch_n_rep(uint32_t i_rep);

/// k0 of Table 16.4.1-1: the number of NB-IoT DL subframes between subframe n+5 and the NPDSCH, or -1
SRSRAN_API int srsran_nbiot_npdsch_k0(uint32_t i_delay, uint32_t r_max);

/// TBS of Table 16.4.1.5.1-1 for (I_TBS, I_SF), or 0 for an entry the table does not have
SRSRAN_API uint32_t srsran_nbiot_npdsch_tbs(uint32_t i_tbs, uint32_t i_sf);

/* -------------------------------------------------------------------------------------------------------- NPDCCH */

/**
 * Codes one DCI for an NPDCCH of aggregation level 2 (format 1: NCCE 0 and 1): CRC-16, scrambled with the RNTI (36.212
 * 5.3.3.2), tail-biting convolutional code and rate matching to E = srsran_nbiot_dlch_bits_per_sf() bits, one per byte.
 */
SRSRAN_API int
srsran_nbiot_npdcch_encode(const srsran_nbiot_dlch_t* q, const uint8_t* dci, uint32_t nbits, uint16_t rnti, uint8_t* e);

/**
 * Maps one subframe of an NPDCCH transmission of the coded bits 'e' into the anchor PRB of the host grid (port 0).
 *
 * The scrambling sequence is initialised at the start of the search space and after every 4th NPDCCH subframe
 * (36.211 10.2.5.2) with c_init = floor(n_s / 2) 2^9 + N_ID, where n_s is the first slot of the subframe at which that
 * happened; pos_in_group is the position of this subframe within its group of four (0..3) and reinit_sf the subframe
 * number (0..9) of the subframe that started the group. Every subframe of the transmission carries the same coded bits.
 */
SRSRAN_API int srsran_nbiot_npdcch_put_sf(const srsran_nbiot_dlch_t* q,
                                          const uint8_t*             e,
                                          uint32_t                   reinit_sf,
                                          uint32_t                   pos_in_group,
                                          cf_t*                      grid);

/* -------------------------------------------------------------------------------------------------------- NPDSCH */

/**
 * CRC-24A, tail-biting convolutional code and rate matching of a transport block (msb first) to nof_sf subframes
 * (36.212 6.4.2). e receives nof_sf * srsran_nbiot_dlch_bits_per_sf() bits, one per byte.
 */
SRSRAN_API int srsran_nbiot_npdsch_encode(const srsran_nbiot_dlch_t* q,
                                          const uint8_t*             tb,
                                          uint32_t                   tb_bits,
                                          uint32_t                   nof_sf,
                                          uint8_t*                   e);

/**
 * One subframe of an NPDSCH not carrying the BCCH: the part 'sf_in_cw' (0 .. nof_sf - 1) of the codeword 'e', scrambled
 * as the codeword and mapped to the anchor PRB. The scrambling sequence (TS 36.211 10.2.3.1)
 *
 *   c_init = n_RNTI 2^14 + (n_f mod 2) 2^13 + floor(n_s / 2) 2^9 + N_ID
 *
 * is initialised for every pass through the codeword (min(N_Rep, 4) transmissions of it, each subframe repeated
 * min(N_Rep, 4) times in a row, 10.2.3.4), with n_f, n_s those of the first subframe of the pass: pass_sfn and pass_sf.
 */
SRSRAN_API int srsran_nbiot_npdsch_put_sf(const srsran_nbiot_dlch_t* q,
                                          const uint8_t*             e,
                                          uint32_t                   nof_sf,
                                          uint32_t                   sf_in_cw,
                                          uint16_t                   rnti,
                                          uint32_t                   pass_sfn,
                                          uint32_t                   pass_sf,
                                          cf_t*                      grid);

#endif // SRSRAN_NBIOT_DLCH_H
