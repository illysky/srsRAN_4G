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

#ifndef SRSENB_EMTC_CONFIG_H
#define SRSENB_EMTC_CONFIG_H

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

extern "C" {
#include "srsran/phy/phch/emtc.h"
}

namespace srsenb {

struct rrc_cfg_t;

namespace emtc {

/**
 * LTE-M (eMTC) on the LTE carrier, as described by enb_emtc.conf: what is broadcast to BL/CE UEs in SIB1-BR and in
 * the BR SIB2 on top of the LTE cell's own SIB1/SIB2, and the eNB's resources for them. One coverage level (CE level 0,
 * CE mode A). Values are the numbers of the spec (0-based narrowbands, subframes, repetitions), not ASN.1 enums.
 */
struct config {
  // SIB1-BR
  uint32_t sched_info_sib1_br = 0; ///< MIB schedulingInfoSIB1-BR: TBS and repetitions of SIB1-BR (1..18)
  uint32_t start_symbol       = 3; ///< startSymbolBR: first OFDM symbol of MPDCCH/PDSCH (not SIB1-BR)
  uint32_t si_window_ms       = 40;
  uint32_t si_repetition_rf   = 1; ///< si-RepetitionPattern: SI message in every 1, 2, 4 or 8 radio frames
  struct si_msg {
    uint32_t periodicity_rf = 16;
    uint32_t nb             = 0;
    uint32_t tbs            = 504;
  };
  std::vector<si_msg> si; ///< the first one carries SIB2; no other SIBs are broadcast to BL UEs

  // RACH (CE level 0): the LTE PRACH resource, the preambles above the LTE ones
  uint32_t first_preamble        = 52;
  uint32_t last_preamble         = 63;
  uint32_t ra_response_window_sf = 20;  ///< 20, 50, 80, 120, 180, 240, 320, 400
  uint32_t contention_timer_sf   = 80;  ///< 80, 100, 120, 160, 200, 240, 480, 960
  uint32_t preamble_trans_max_ce = 10;
  uint32_t prach_config_index    = 3;
  uint32_t prach_freq_offset     = 4;
  uint32_t prach_repetitions     = 1;
  uint32_t max_preamble_attempts = 3;
  std::vector<uint32_t> mpdcch_nb_ra;   ///< narrowbands of the Type2-CSS (RAR, Msg4)
  uint32_t mpdcch_rep_ra        = 1;   ///< Rmax of the Type2-CSS
  double   mpdcch_start_sf_ra   = 2.0; ///< G of the Type2-CSS

  // PUCCH, paging, maximum repetitions
  uint32_t n1_pucch_an       = 30; ///< n1PUCCH-AN of CE level 0 (HARQ-ACK of Msg4)
  uint32_t pucch_rep_msg4    = 1;
  uint32_t paging_nbs        = 1; ///< paging-narrowBands-r13: narrowbands 0 .. paging_nbs - 1
  uint32_t mpdcch_rep_paging = 1;
  uint32_t pdsch_max_rep     = 16; ///< pdsch-maxNumRepetitionCEmodeA
  uint32_t pusch_max_rep     = 8;  ///< pusch-maxNumRepetitionCEmodeA
  uint32_t ul_hop_interval   = 1;  ///< interval-ULHoppingConfigCommonModeA (FDD: 1, 2, 4, 8 subframes)

  // Msg3 (eNB side, sent in the RAR grant): UL narrowband, PRBs within it, MCS 0..7
  uint32_t msg3_nb        = 2;
  uint32_t msg3_rb_start  = 0;
  uint32_t msg3_nof_rb    = 3;
  uint32_t msg3_mcs       = 1;
  bool     dci_srs_6_1a   = true; ///< DCI 6-1A carries the 1-bit SRS request (TS 36.212 V14.2 5.3.3.1.12)
};

/// What the DL composer transmits: the packed messages and when/where they go
struct bcast {
  srsran_emtc_bcast_cfg_t           sched      = {};
  uint32_t                          start_symbol = 3;
  uint32_t                          ul_hop_interval = 1; ///< N_NB^ch,UL of CE mode A
  std::vector<uint8_t>              sib1;       ///< BCCH-DL-SCH-BR SIB1-BR, zero padded to its TBS
  std::vector<std::vector<uint8_t>> si;         ///< BCCH-DL-SCH-BR SystemInformation-BR, zero padded to each si-TBS
  size_t                            sib1_len = 0; ///< unpadded bytes
  std::vector<size_t>               si_len;
};

/// Read and validate enb_emtc.conf
bool load(const std::string& path, config& out, std::string& err);

/// SIB1-BR and the SI messages of the first cell of rrc_cfg (whose PCI/bandwidth are given) with the BR fields of cfg
bool build_bcast(const rrc_cfg_t& rrc_cfg,
                 uint32_t         nof_prb,
                 uint32_t         pci,
                 const config&    cfg,
                 bcast&           out,
                 std::string&     err);

} // namespace emtc
} // namespace srsenb

#endif // SRSENB_EMTC_CONFIG_H
