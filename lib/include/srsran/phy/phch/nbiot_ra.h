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
 *  File:         nbiot_ra.h
 *
 *  Description:  NB-IoT random access, eNodeB side: what to answer to a detected preamble.
 *                RA-RNTI, timing advance from the measured ToA, the Msg3 grant carried in the random access
 *                response, its translation into NPUSCH parameters, and the Msg2 MAC PDU itself.
 *
 *  Reference:    3GPP TS 36.321 v14.2.0 clauses 5.1.4, 6.1.5, 6.2.2, 6.2.3 (Figure 6.1.5-3b)
 *                3GPP TS 36.213 v14.2.0 clauses 4.2.3, 16.3.3 (Table 16.3.3-1), 16.5.1, 16.5.1.1
 *****************************************************************************/

#ifndef SRSRAN_NBIOT_RA_H
#define SRSRAN_NBIOT_RA_H

#include "srsran/config.h"
#include "srsran/phy/phch/npusch.h"
#include <stdint.h>

#define SRSRAN_NBIOT_RAR_LEN 6           // one MAC RAR, octets (Figure 6.1.5-3b)
#define SRSRAN_NBIOT_RAR_MAX_TA 1282     // TS 36.213 4.2.3
#define SRSRAN_NBIOT_MSG3_TBS 88         // every Msg3 grant of Table 16.3.3-1
#define SRSRAN_NBIOT_RAR_MAX_PER_PDU 8

/** The 15-bit narrowband random access response grant (TS 36.213 16.3.3), fields as signalled. */
typedef struct SRSRAN_API {
  uint32_t sc_15khz; // uplink subcarrier spacing: 0 = 3.75 kHz, 1 = 15 kHz (1 bit)
  uint32_t i_sc;     // subcarrier indication (6 bits)
  uint32_t i_delay;  // scheduling delay: k0 = 12, 16, 32, 64 subframes (2 bits)
  uint32_t i_rep;    // Msg3 repetitions: 1, 2, 4 ... 128 (3 bits)
  uint32_t i_mcs;    // Table 16.3.3-1 (3 bits): 0..2 valid
} srsran_nbiot_msg3_grant_t;

/** One MAC RAR together with the RAPID of its subheader. */
typedef struct SRSRAN_API {
  uint32_t                  rapid;    // preamble identifier: the start subcarrier of the preamble (6 bits)
  uint32_t                  ta;       // timing advance command index, 0..1282
  srsran_nbiot_msg3_grant_t grant;
  uint32_t                  tc_rnti;  // temporary C-RNTI (16 bits)
} srsran_nbiot_rar_t;

/** RA-RNTI of an NPRACH occasion whose first radio frame is sfn_id (TS 36.321 5.1.4). carrier_id 0 = anchor. */
SRSRAN_API uint32_t srsran_nbiot_ra_rnti(uint32_t sfn_id, uint32_t carrier_id);

/**
 * Timing advance command index for a preamble that arrived toa_samples (1.92 MS/s samples) after its nominal start:
 * N_TA = TA * 16 Ts and one sample is 16 Ts, so TA is the ToA rounded to a sample, clipped to 0 .. 1282.
 */
SRSRAN_API uint32_t srsran_nbiot_ta_from_toa(float toa_samples);

/** SRSRAN_SUCCESS if every field is a value the specification defines for a Msg3 grant. */
SRSRAN_API int srsran_nbiot_msg3_grant_check(const srsran_nbiot_msg3_grant_t* g);

/** Packs to the 15-bit field, first field in the most significant bit. Fails on reserved values. */
SRSRAN_API int srsran_nbiot_msg3_grant_pack(const srsran_nbiot_msg3_grant_t* g, uint32_t* bits15);

/** Splits a 15-bit field; performs no validation. */
SRSRAN_API void srsran_nbiot_msg3_grant_unpack(uint32_t bits15, srsran_nbiot_msg3_grant_t* g);

/**
 * NPUSCH parameters of the Msg3 transmission described by a valid grant: tones, spacing, resource units, repetitions,
 * modulation, transport block size 88, scrambling by the temporary C-RNTI, redundancy version 0. Fields that depend
 * on when Msg3 is sent (frame, slot) and on system information (group hopping, delta_ss, cyclic shift) are left at
 * zero for the caller. k0 receives the scheduling delay in subframes, counted from the last subframe of the RAR
 * NPDSCH: Msg3 starts at the first uplink slot after the end of subframe n + k0.
 */
SRSRAN_API int srsran_nbiot_msg3_grant_to_npusch(const srsran_nbiot_msg3_grant_t* g,
                                                 uint32_t                         tc_rnti,
                                                 uint32_t                         cell_id,
                                                 srsran_npusch_cfg_t*             cfg,
                                                 uint32_t*                        k0);

/**
 * Builds a random access response MAC PDU: an optional Backoff Indicator subheader (backoff_ind < 0 for none), one
 * E/T/RAPID subheader per RAR, then the RARs in the same order. Returns the length in octets, or a negative value.
 */
SRSRAN_API int srsran_nbiot_rar_pdu_pack(int                       backoff_ind,
                                         const srsran_nbiot_rar_t* rar,
                                         uint32_t                  nof_rar,
                                         uint8_t*                  out,
                                         uint32_t                  max_len);

/**
 * Parses a random access response MAC PDU. Returns the number of RARs stored in 'rar' (at most max_rar), or a negative
 * value if the PDU is malformed (truncated, a Backoff Indicator anywhere but first, more RARs than max_rar).
 * *backoff_ind is -1 if the PDU has no Backoff Indicator. Trailing octets after the last RAR are padding.
 */
SRSRAN_API int srsran_nbiot_rar_pdu_unpack(const uint8_t*      pdu,
                                           uint32_t            len,
                                           int*                backoff_ind,
                                           srsran_nbiot_rar_t* rar,
                                           uint32_t            max_rar);

#endif // SRSRAN_NBIOT_RA_H
