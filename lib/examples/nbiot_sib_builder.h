/*
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

/**
 * NB-IoT system information builder.
 *
 * Reads the NB-IoT carrier description (enb_nbiot.conf, libconfig format), validates it against what the PHY
 * actually implements, and packs the resulting MIB-NB and SIB1-NB with the ASN.1 library. The config file is the
 * single source of truth: the PHY generator and (later) the eNB stack must take their cell parameters from the same
 * cell_config so that what is broadcast is what is transmitted.
 */

#ifndef SRSRAN_NBIOT_SIB_BUILDER_H
#define SRSRAN_NBIOT_SIB_BUILDER_H

#include <cstdint>
#include <libconfig.h++>
#include <string>
#include <vector>

extern "C" {
#include "srsran/phy/common/phy_common.h"
#include "srsran/phy/phch/npbch.h"
#include "srsran/phy/phch/ra_nbiot.h"
}

namespace nbiot {

/// One entry of SIB1-NB schedulingInfoList (TS 36.331 SchedulingInfo-NB-r13)
struct si_sched_entry {
  uint32_t              periodicity_rf     = 0; ///< 64..4096 radio frames
  uint32_t              repetition_pattern = 0; ///< 2, 4, 8 or 16 (every Nth radio frame)
  uint32_t              tb_bits            = 0; ///< 56..680
  std::vector<uint32_t> sib_mapping;            ///< SIB numbers OTHER than SIB2-NB (which is implicit)
};

/// SIB2-NB content (TS 36.331 SystemInformationBlockType2-NB). Everything the UE needs for random access and paging.
struct sib2_config {
  // RACH-ConfigCommon-NB
  uint32_t preamble_trans_max_ce         = 0; ///< preambleTransMax-CE, attempts
  int      preamble_init_rx_target_power = 0; ///< dBm, even values -120..-90
  uint32_t power_ramping_step_db         = 0; ///< 0, 2, 4 or 6
  uint32_t ra_response_window            = 0; ///< in NPDCCH search-space periods
  uint32_t mac_contention_timer          = 0; ///< in NPDCCH search-space periods

  // NPRACH-ConfigSIB-NB (single coverage-enhancement level)
  double   nprach_cp_length_us          = 0; ///< 66.7 (format 0) or 266.7 (format 1)
  uint32_t nprach_periodicity_ms        = 0;
  uint32_t nprach_start_time_ms         = 0;
  uint32_t nprach_subcarrier_offset     = 0;
  uint32_t nprach_num_subcarriers       = 0;
  std::string msg3_subcarrier_range_start;   ///< "zero", "oneThird", "twoThird" or "one"
  uint32_t max_preamble_attempts        = 0; ///< maxNumPreambleAttemptCE
  uint32_t num_repetitions_per_preamble = 0;
  uint32_t npdcch_num_repetitions_ra    = 0;
  double   npdcch_start_sf_css_ra       = 0; ///< multiple of Rmax: 1.5, 2, 4, ... 64
  std::string npdcch_offset_ra;              ///< "zero", "oneEighth", "oneFourth" or "threeEighth"

  // PCCH-Config-NB
  uint32_t    default_paging_cycle_rf = 0; ///< radio frames: 128, 256, 512 or 1024
  std::string nb;                          ///< "oneT", "halfT", ...
  uint32_t    npdcch_num_repetitions_paging = 0;

  // NPDSCH / NPUSCH common
  int      nrs_power_dbm                = 0;
  uint32_t ack_nack_num_repetitions_msg4 = 0;
  bool     group_hopping_enabled        = false;
  uint32_t group_assignment_npusch      = 0;

  // UL power control
  int    p0_nominal_npusch   = 0;
  double alpha               = 0;
  int    delta_preamble_msg3 = 0;

  // UE timers and constants (ms / counts)
  uint32_t t300 = 0, t301 = 0, t310 = 0, n310 = 0, t311 = 0, n311 = 0;

  std::string time_alignment_timer; ///< "infinity" or "sf500" ... "sf10240"
};

/// Everything needed to describe one in-band NB-IoT cell. Fields marked "derived" are computed, not configured.
struct cell_config {
  // LTE host carrier
  uint32_t lte_nof_prb   = 0;
  uint32_t lte_pci       = 0;
  uint32_t lte_nof_ports = 0;
  uint32_t lte_cfi       = 0;

  // NB-IoT carrier
  srsran_nbiot_mode_t mode       = SRSRAN_NBIOT_MODE_INBAND_SAME_PCI;
  uint32_t            nbiot_prb  = 0;
  uint32_t            n_id_ncell = 0;
  uint32_t            nof_ports  = 0;

  // derived from lte_nof_prb + nbiot_prb (TS 36.213 Table 16.8-1)
  uint8_t                      crs_seq_info  = 0;
  srsran_nbiot_raster_offset_t raster_offset = SRSRAN_NBIOT_RASTER_OFFSET_M7DOT5_KHZ;

  // cell identity
  std::string mcc;
  std::string mnc;
  uint32_t    tac              = 0;
  uint32_t    cell_id          = 0;
  uint32_t    band             = 0;
  bool        cell_barred      = false;
  bool        intra_freq_resel = true;
  int         q_rx_lev_min     = -70;

  // MIB-NB
  uint32_t sched_info_sib1 = 0;
  uint32_t sys_info_tag    = 0;
  bool     ac_barring      = false;

  // SIB1-NB
  uint32_t                    si_window_ms          = 0;
  uint32_t                    si_radio_frame_offset = 0;
  double                      nrs_crs_pwr_offset_db = 0.0;
  uint32_t                    eutra_ctrl_region     = 0;
  std::vector<si_sched_entry> si_sched;

  // SIB2-NB
  sib2_config sib2;
};

/// Parse and validate. On failure returns false and err says which key is wrong and why.
/// (cfg is non-const because libconfig's integer/float auto-conversion is a per-Config setting.)
bool load_config(libconfig::Config& cfg, cell_config& out, std::string& err);

/// Convenience: read a file, then load_config().
bool load_config_file(const std::string& path, cell_config& out, std::string& err);

/// SIB1-NB transport block size implied by schedulingInfoSIB1 (TS 36.213 Table 16.4.1.3-3), in bits.
uint32_t sib1_tbs_bits(const cell_config& c);

/// Number of NPDSCH repetitions of SIB1-NB implied by schedulingInfoSIB1.
uint32_t sib1_repetitions(const cell_config& c);

/// srsran_mib_nb_t equivalent of the config (what the C packer takes). Used for cross-checking the two encoders.
srsran_mib_nb_t to_c_mib(const cell_config& c);

/// Pack MIB-NB with the ASN.1 library. Output: 34 bits, one bit per byte (the layout the NPBCH encoder takes),
/// so it can be compared directly with srsran_npbch_mib_pack().
bool pack_mib_bits(const cell_config& c, uint32_t hfn, uint32_t sfn, std::vector<uint8_t>& bits, std::string& err);

/// Pack a BCCH-DL-SCH message carrying SIB1-NB. The result is zero-padded to sib1_tbs_bits()/8 bytes, ready to be
/// handed to the NPDSCH encoder. Fails if the message does not fit the transport block.
///
/// hyper_sfn_msb: 8 MSBs of the 10-bit hyper-SFN (the 2 LSBs are in MIB-NB).
bool pack_sib1(const cell_config& c, uint8_t hyper_sfn_msb, std::vector<uint8_t>& out, size_t& unpadded_len, std::string& err);

/// SIB2-NB transport block size implied by the first SI message's si_tb, in bits.
uint32_t sib2_tbs_bits(const cell_config& c);

/// Pack the first SI message: a BCCH-DL-SCH SystemInformation-NB carrying SIB2-NB (plus nothing else; other SIBs are
/// not implemented). Zero-padded to the si_tb of the first schedulingInfoList entry, and refused if it does not fit --
/// in which case the error names the smallest si_tb that would.
bool pack_sib2(const cell_config& c, std::vector<uint8_t>& out, size_t& unpadded_len, std::string& err);

} // namespace nbiot

#endif // SRSRAN_NBIOT_SIB_BUILDER_H
