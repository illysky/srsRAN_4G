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
 *  File:         emtc.h
 *
 *  Description:  LTE-M (eMTC, BL/CE UEs): narrowbands and where the BR system information goes.
 *
 *                Pure functions of the cell and (SFN, subframe): which narrowband, redundancy version and scrambling
 *                subframe SIB1-BR and the BR SI messages use, and which PRBs they take from the LTE carrier. The DL
 *                composer and the LTE scheduler both call them, so they agree on every subframe without sharing state.
 *
 *  Reference:    3GPP TS 36.211 v14.2.0 6.2.7 (narrowbands), 6.3.1 (BL/CE scrambling), 6.4.1 (SIB1-BR)
 *                3GPP TS 36.213 v14.2.0 7.1.6 (Table 7.1.6-1), 7.1.6.4A, 7.1.7.2.7 (Table 7.1.7.2.7-1)
 *                3GPP TS 36.321 v14.2.0 5.3.1 (RV of SIB1-BR and SI messages)
 *                3GPP TS 36.331 v14.2.2 5.2.3a (SI window of BL UEs)
 *****************************************************************************/

#ifndef SRSRAN_EMTC_H
#define SRSRAN_EMTC_H

#include "srsran/config.h"
#include <stdbool.h>
#include <stdint.h>

#define SRSRAN_EMTC_NB_NOF_PRB 6
#define SRSRAN_EMTC_MAX_SI 8

/// Number of downlink (or uplink) narrowbands of a carrier with nof_prb PRBs
SRSRAN_API uint32_t srsran_emtc_nof_nb(uint32_t nof_prb);

/// Lowest PRB of narrowband nb; the narrowband is that PRB and the next five. -1 if nb does not exist.
SRSRAN_API int srsran_emtc_nb_first_prb(uint32_t nof_prb, uint32_t nb);

/// Whether narrowband nb overlaps the 72 centre subcarriers (PSS, SSS, PBCH)
SRSRAN_API bool srsran_emtc_nb_in_centre(uint32_t nof_prb, uint32_t nb);

/// PRB bitmask (bit n = PRB n) of narrowband nb
SRSRAN_API uint64_t srsran_emtc_nb_mask(uint32_t nof_prb, uint32_t nb);

/// Number of PDSCH repetitions of SIB1-BR per 80 ms for schedulingInfoSIB1-BR (4, 8 or 16); 0 if not transmitted
SRSRAN_API uint32_t srsran_emtc_sib1_br_repetitions(uint32_t sched_info_sib1_br);

/// Transport block size of SIB1-BR, in bits, for schedulingInfoSIB1-BR; 0 if not transmitted
SRSRAN_API uint32_t srsran_emtc_sib1_br_tbs(uint32_t sched_info_sib1_br);

/// RV_K = ceil(3k/2) mod 4
SRSRAN_API uint32_t srsran_emtc_rv(uint32_t k);

/// The subframe index (0..9) whose slot pair seeds the PDSCH scrambling of absolute subframe t when the same
/// scrambling covers blocks of n_acc subframes (TS 36.211 6.3.1, FDD): ((t / n_acc) * n_acc) mod 10
SRSRAN_API uint32_t srsran_emtc_scrambling_sf(uint32_t t, uint32_t n_acc);

/// One BR SI message (entry n of schedulingInfoList / schedulingInfoList-BR)
typedef struct SRSRAN_API {
  uint32_t periodicity_rf; ///< si-Periodicity, radio frames (8..512)
  uint32_t nb;             ///< si-Narrowband - 1
  uint32_t tbs;            ///< si-TBS, bits
} srsran_emtc_si_t;

/// What the cell broadcasts for BL/CE UEs
typedef struct SRSRAN_API {
  uint32_t         nof_prb;
  uint32_t         pci;
  uint32_t         sched_info_sib1_br; ///< MIB schedulingInfoSIB1-BR-r13, 1..18
  uint32_t         si_window_ms;       ///< si-WindowLength-BR
  uint32_t         si_repetition_rf;   ///< si-RepetitionPattern: every 1, 2, 4 or 8 radio frames
  uint32_t         nof_si;
  srsran_emtc_si_t si[SRSRAN_EMTC_MAX_SI];
} srsran_emtc_bcast_cfg_t;

/// Whether SIB1-BR is transmitted in (sfn, sf); if so its narrowband and redundancy version
SRSRAN_API bool
srsran_emtc_sib1_br(const srsran_emtc_bcast_cfg_t* cfg, uint32_t sfn, uint32_t sf, uint32_t* nb, uint32_t* rv);

/// The BR SI message (index into cfg->si) transmitted in (sfn, sf), and its redundancy version; -1 if none. An SI
/// message is sent in every subframe of the radio frames of its window given by si-RepetitionPattern, except where
/// SIB1-BR takes the same narrowband (TS 36.213 7.1.11: the UE assumes it dropped there).
SRSRAN_API int srsran_emtc_si(const srsran_emtc_bcast_cfg_t* cfg, uint32_t sfn, uint32_t sf, uint32_t* rv);

/// PRBs of the LTE carrier (bit n = PRB n) that SIB1-BR and the BR SI messages take in (sfn, sf)
SRSRAN_API uint64_t srsran_emtc_bcast_prbs(const srsran_emtc_bcast_cfg_t* cfg, uint32_t sfn, uint32_t sf);

/// Narrowband that carries (a repetition of) a BR broadcast in (sfn, sf), as a bitmask of narrowbands
SRSRAN_API uint32_t srsran_emtc_bcast_nbs(const srsran_emtc_bcast_cfg_t* cfg, uint32_t sfn, uint32_t sf);

/// Bits of a narrowband index: ceil(log2(floor(N_RB / 6)))
SRSRAN_API uint32_t srsran_emtc_nb_bits(uint32_t nof_prb);

/// Resource indication value of L contiguous PRBs starting at PRB s of a narrowband (N = 6, TS 36.213 7.1.6.3 / 8.1.1)
SRSRAN_API uint32_t srsran_emtc_riv(uint32_t start, uint32_t len);

/// Fields of DCI format 6-0A / 6-1A (TS 36.212 5.3.3.1.10, 5.3.3.1.12) for FDD, CE mode A, TM1/TM2
typedef struct SRSRAN_API {
  bool     hopping;
  uint32_t nb;              ///< narrowband index
  uint32_t riv;             ///< 5-bit allocation within the narrowband (DL type 2, UL type 0)
  uint32_t mcs;             ///< 4 bits
  uint32_t rep;             ///< PDSCH/PUSCH repetition number, 2 bits
  uint32_t harq_pid;        ///< 3 bits
  bool     ndi;
  uint32_t rv;              ///< 2 bits
  uint32_t tpc;             ///< 2 bits (6-1A with RA-RNTI: LSB selects N_PRB^1A = 3 when set)
  uint32_t harq_ack_offset; ///< 6-1A, 2 bits
  bool     csi_request;     ///< 6-0A
  bool     srs_request;
  uint32_t dci_rep;         ///< DCI subframe repetition number, 2 bits
} srsran_emtc_dci_t;

/// Payload size of 6-0A/6-1A on the same search space (the smaller one is zero padded to the larger).
/// srs_6_1a: whether 6-1A carries the 1-bit SRS request.
SRSRAN_API uint32_t srsran_emtc_dci_size(uint32_t nof_prb, bool srs_6_1a);

/// Packs DCI 6-1A (unpacked bits, MSB first) padded to srsran_emtc_dci_size(); returns the size
SRSRAN_API uint32_t srsran_emtc_dci_6_1a_pack(uint32_t nof_prb, bool srs_6_1a, const srsran_emtc_dci_t* dci, uint8_t* bits);

/// Packs DCI 6-0A (unpacked bits, MSB first) padded to srsran_emtc_dci_size(); returns the size
SRSRAN_API uint32_t srsran_emtc_dci_6_0a_pack(uint32_t nof_prb, bool srs_6_1a, const srsran_emtc_dci_t* dci, uint8_t* bits);

/// 20-bit CE mode A random access response grant (TS 36.213 6.2, Table 6-2). riv4 is the 4-bit UL type 0 allocation
/// within the Msg3 narrowband (a leading 0 is implied); rep is the 2-bit Msg3 repetition code (00 = pusch-max/8),
/// mcs the 3-bit truncated MCS, tpc the 3-bit TPC (3 = 0 dB), mpdcch_nb the 2-bit Msg3/4 MPDCCH narrowband offset.
SRSRAN_API uint32_t srsran_emtc_rar_grant_ce_a(uint32_t nof_prb_ul,
                                               uint32_t msg3_nb,
                                               uint32_t riv4,
                                               uint32_t rep,
                                               uint32_t mcs,
                                               uint32_t tpc,
                                               bool     csi,
                                               bool     ul_delay,
                                               uint32_t mpdcch_nb);

/// BR RA-RNTI = 1 + t_id + 10 f_id + 60 (SFN_id mod (Wmax / 10)), Wmax = 400 (TS 36.321 5.1.4)
SRSRAN_API uint16_t srsran_emtc_ra_rnti(uint32_t prach_sfn, uint32_t prach_sf, uint32_t f_id);

#endif // SRSRAN_EMTC_H
