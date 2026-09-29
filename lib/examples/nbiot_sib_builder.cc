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

#include "nbiot_sib_builder.h"

#include <cmath>
#include <cstring>

#include "srsran/asn1/rrc_nbiot.h"

namespace nbiot {

// The PHY (ra_nbiot.c EUTRA_CONTROL_REGION_SIZE) starts in-band NPDSCH/NPDCCH at this symbol. SIB1-NB must never
// advertise anything else. Keep in sync with that constant until the PHY reads it from the SIB.
static const uint32_t PHY_EUTRA_CONTROL_REGION_SIZE = 3;

/*************************************************************************************************
 * Config parsing
 *************************************************************************************************/

namespace {

template <typename T>
bool get(const libconfig::Config& c, const std::string& path, T& v, std::string& err)
{
  if (!c.exists(path)) {
    err = "missing required key '" + path + "'";
    return false;
  }
  try {
    if (!c.lookupValue(path, v)) {
      err = "key '" + path + "' has the wrong type";
      return false;
    }
  } catch (const libconfig::SettingException& e) {
    err = "key '" + path + "': " + e.what();
    return false;
  }
  return true;
}

bool all_digits(const std::string& s)
{
  if (s.empty()) {
    return false;
  }
  for (char ch : s) {
    if (ch < '0' || ch > '9') {
      return false;
    }
  }
  return true;
}

bool valid_lte_prb(uint32_t n)
{
  return n == 6 || n == 15 || n == 25 || n == 50 || n == 75 || n == 100;
}

} // namespace

bool load_config(libconfig::Config& cfg, cell_config& o, std::string& err)
{
  cfg.setAutoConvert(true);
  o = cell_config();

  // ---- LTE host
  if (!get(cfg, "nbiot.lte.n_prb", o.lte_nof_prb, err) || !get(cfg, "nbiot.lte.pci", o.lte_pci, err) ||
      !get(cfg, "nbiot.lte.nof_ports", o.lte_nof_ports, err) || !get(cfg, "nbiot.lte.cfi", o.lte_cfi, err)) {
    return false;
  }
  if (!valid_lte_prb(o.lte_nof_prb)) {
    err = "nbiot.lte.n_prb must be one of 6,15,25,50,75,100";
    return false;
  }
  if (o.lte_pci > 503) {
    err = "nbiot.lte.pci must be 0..503";
    return false;
  }
  if (o.lte_nof_ports != 1 && o.lte_nof_ports != 2 && o.lte_nof_ports != 4) {
    err = "nbiot.lte.nof_ports must be 1, 2 or 4";
    return false;
  }

  // ---- NB-IoT carrier
  std::string mode;
  if (!get(cfg, "nbiot.carrier.operation_mode", mode, err) || !get(cfg, "nbiot.carrier.nbiot_prb", o.nbiot_prb, err) ||
      !get(cfg, "nbiot.carrier.n_id_ncell", o.n_id_ncell, err) ||
      !get(cfg, "nbiot.carrier.nof_ports", o.nof_ports, err)) {
    return false;
  }
  if (mode != "inband_same_pci") {
    err = "nbiot.carrier.operation_mode '" + mode +
          "' is not supported yet; only 'inband_same_pci' is implemented end-to-end";
    return false;
  }
  o.mode = SRSRAN_NBIOT_MODE_INBAND_SAME_PCI;
  if (o.n_id_ncell != o.lte_pci) {
    err = "inband_same_pci requires nbiot.carrier.n_id_ncell == nbiot.lte.pci (" + std::to_string(o.n_id_ncell) +
          " != " + std::to_string(o.lte_pci) + ")";
    return false;
  }
  if (o.nof_ports != 1 && o.nof_ports != 2) {
    err = "nbiot.carrier.nof_ports (NRS) must be 1 or 2";
    return false;
  }
  if (srsran_nbiot_prb_to_crs_seq_info(o.lte_nof_prb, o.nbiot_prb, &o.crs_seq_info, &o.raster_offset) !=
      SRSRAN_SUCCESS) {
    err = "nbiot.carrier.nbiot_prb=" + std::to_string(o.nbiot_prb) + " is not a legal anchor PRB for a " +
          std::to_string(o.lte_nof_prb) + "-PRB LTE carrier (TS 36.101 5.7.3 / TS 36.213 Table 16.8-1)";
    return false;
  }

  // ---- identity
  if (!get(cfg, "nbiot.cell.mcc", o.mcc, err) || !get(cfg, "nbiot.cell.mnc", o.mnc, err) ||
      !get(cfg, "nbiot.cell.tac", o.tac, err) || !get(cfg, "nbiot.cell.cell_id", o.cell_id, err) ||
      !get(cfg, "nbiot.cell.band", o.band, err) || !get(cfg, "nbiot.cell.cell_barred", o.cell_barred, err) ||
      !get(cfg, "nbiot.cell.intra_freq_reselection", o.intra_freq_resel, err) ||
      !get(cfg, "nbiot.cell.q_rx_lev_min", o.q_rx_lev_min, err)) {
    return false;
  }
  if (o.mcc.size() != 3 || !all_digits(o.mcc)) {
    err = "nbiot.cell.mcc must be exactly 3 digits";
    return false;
  }
  if ((o.mnc.size() != 2 && o.mnc.size() != 3) || !all_digits(o.mnc)) {
    err = "nbiot.cell.mnc must be 2 or 3 digits";
    return false;
  }
  if (o.tac > 0xFFFF) {
    err = "nbiot.cell.tac must fit 16 bits";
    return false;
  }
  if (o.cell_id > 0x0FFFFFFF) {
    err = "nbiot.cell.cell_id must fit 28 bits";
    return false;
  }
  if (o.band < 1 || o.band > 256) {
    err = "nbiot.cell.band must be 1..256";
    return false;
  }
  if (o.q_rx_lev_min < -70 || o.q_rx_lev_min > -22) {
    err = "nbiot.cell.q_rx_lev_min must be -70..-22 dBm";
    return false;
  }

  // ---- MIB
  if (!get(cfg, "nbiot.mib.sched_info_sib1", o.sched_info_sib1, err) ||
      !get(cfg, "nbiot.mib.sys_info_tag", o.sys_info_tag, err) ||
      !get(cfg, "nbiot.mib.ac_barring", o.ac_barring, err)) {
    return false;
  }
  if (o.sched_info_sib1 > 11) {
    err = "nbiot.mib.sched_info_sib1 must be 0..11 (12..15 are reserved)";
    return false;
  }
  if (o.sys_info_tag > 31) {
    err = "nbiot.mib.sys_info_tag must be 0..31";
    return false;
  }

  // ---- SIB1
  if (!get(cfg, "nbiot.sib1.si_window_length", o.si_window_ms, err) ||
      !get(cfg, "nbiot.sib1.si_radio_frame_offset", o.si_radio_frame_offset, err) ||
      !get(cfg, "nbiot.sib1.nrs_crs_power_offset", o.nrs_crs_pwr_offset_db, err) ||
      !get(cfg, "nbiot.sib1.eutra_control_region_size", o.eutra_ctrl_region, err)) {
    return false;
  }
  if (o.si_radio_frame_offset < 1 || o.si_radio_frame_offset > 15) {
    err = "nbiot.sib1.si_radio_frame_offset must be 1..15";
    return false;
  }
  if (o.eutra_ctrl_region != o.lte_cfi) {
    err = "nbiot.sib1.eutra_control_region_size (" + std::to_string(o.eutra_ctrl_region) +
          ") must equal nbiot.lte.cfi (" + std::to_string(o.lte_cfi) + ")";
    return false;
  }
  if (o.eutra_ctrl_region != PHY_EUTRA_CONTROL_REGION_SIZE) {
    err = "the PHY currently starts in-band NB-IoT at symbol " + std::to_string(PHY_EUTRA_CONTROL_REGION_SIZE) +
          "; a control region size of " + std::to_string(o.eutra_ctrl_region) +
          " would be advertised in SIB1-NB but not honoured on the air";
    return false;
  }

  const std::string sched_path = "nbiot.sib1.sched_info";
  if (!cfg.exists(sched_path)) {
    err = "missing required key '" + sched_path + "'";
    return false;
  }
  try {
    const libconfig::Setting& list = cfg.lookup(sched_path);
    if (!list.isList() || list.getLength() < 1 || list.getLength() > 32) {
      err = sched_path + " must be a list of 1..32 groups";
      return false;
    }
    for (int i = 0; i < list.getLength(); ++i) {
      const libconfig::Setting& g = list[i];
      si_sched_entry            e;
      if (!g.lookupValue("si_periodicity", e.periodicity_rf) ||
          !g.lookupValue("si_repetition_pattern", e.repetition_pattern) || !g.lookupValue("si_tb", e.tb_bits)) {
        err = sched_path + "[" + std::to_string(i) + "] needs si_periodicity, si_repetition_pattern and si_tb";
        return false;
      }
      if (g.exists("sib_mapping")) {
        const libconfig::Setting& m = g["sib_mapping"];
        for (int j = 0; j < m.getLength(); ++j) {
          uint32_t sib = (unsigned int)m[j];
          if (sib == 2) {
            err = sched_path + "[" + std::to_string(i) +
                  "].sib_mapping lists SIB2, which is implicit in the first SI message of NB-IoT and cannot be "
                  "encoded (TS 36.331 SIB-Type-NB-r13 starts at SIB3)";
            return false;
          }
          e.sib_mapping.push_back(sib);
        }
      }
      o.si_sched.push_back(e);
    }
  } catch (const libconfig::SettingException& e) {
    err = sched_path + ": " + e.what();
    return false;
  }

  // ---- SIB2
  sib2_config& z = o.sib2;
  uint32_t     nof_ce_levels = 0;
  if (!get(cfg, "nbiot.sib2.rach.preamble_trans_max_ce", z.preamble_trans_max_ce, err) ||
      !get(cfg, "nbiot.sib2.rach.preamble_init_rx_target_power", z.preamble_init_rx_target_power, err) ||
      !get(cfg, "nbiot.sib2.rach.power_ramping_step", z.power_ramping_step_db, err) ||
      !get(cfg, "nbiot.sib2.rach.ra_response_window", z.ra_response_window, err) ||
      !get(cfg, "nbiot.sib2.rach.mac_contention_timer", z.mac_contention_timer, err) ||
      !get(cfg, "nbiot.sib2.nprach.cp_length_us", z.nprach_cp_length_us, err) ||
      !get(cfg, "nbiot.sib2.nprach.periodicity_ms", z.nprach_periodicity_ms, err) ||
      !get(cfg, "nbiot.sib2.nprach.start_time_ms", z.nprach_start_time_ms, err) ||
      !get(cfg, "nbiot.sib2.nprach.subcarrier_offset", z.nprach_subcarrier_offset, err) ||
      !get(cfg, "nbiot.sib2.nprach.nof_subcarriers", z.nprach_num_subcarriers, err) ||
      !get(cfg, "nbiot.sib2.nprach.nof_ce_levels", nof_ce_levels, err) ||
      !get(cfg, "nbiot.sib2.nprach.msg3_subcarrier_range_start", z.msg3_subcarrier_range_start, err) ||
      !get(cfg, "nbiot.sib2.nprach.max_preamble_attempts", z.max_preamble_attempts, err) ||
      !get(cfg, "nbiot.sib2.nprach.num_repetitions_per_preamble", z.num_repetitions_per_preamble, err) ||
      !get(cfg, "nbiot.sib2.nprach.npdcch_num_repetitions_ra", z.npdcch_num_repetitions_ra, err) ||
      !get(cfg, "nbiot.sib2.nprach.npdcch_start_sf_css_ra", z.npdcch_start_sf_css_ra, err) ||
      !get(cfg, "nbiot.sib2.nprach.npdcch_offset_ra", z.npdcch_offset_ra, err) ||
      !get(cfg, "nbiot.sib2.pcch.default_paging_cycle_rf", z.default_paging_cycle_rf, err) ||
      !get(cfg, "nbiot.sib2.pcch.nB", z.nb, err) ||
      !get(cfg, "nbiot.sib2.pcch.npdcch_num_repetitions_paging", z.npdcch_num_repetitions_paging, err) ||
      !get(cfg, "nbiot.sib2.npdsch.nrs_power_dbm", z.nrs_power_dbm, err) ||
      !get(cfg, "nbiot.sib2.npusch.ack_nack_num_repetitions_msg4", z.ack_nack_num_repetitions_msg4, err) ||
      !get(cfg, "nbiot.sib2.npusch.group_hopping_enabled", z.group_hopping_enabled, err) ||
      !get(cfg, "nbiot.sib2.npusch.group_assignment_npusch", z.group_assignment_npusch, err) ||
      !get(cfg, "nbiot.sib2.ul_power_control.p0_nominal_npusch", z.p0_nominal_npusch, err) ||
      !get(cfg, "nbiot.sib2.ul_power_control.alpha", z.alpha, err) ||
      !get(cfg, "nbiot.sib2.ul_power_control.delta_preamble_msg3", z.delta_preamble_msg3, err) ||
      !get(cfg, "nbiot.sib2.ue_timers.t300", z.t300, err) || !get(cfg, "nbiot.sib2.ue_timers.t301", z.t301, err) ||
      !get(cfg, "nbiot.sib2.ue_timers.t310", z.t310, err) || !get(cfg, "nbiot.sib2.ue_timers.n310", z.n310, err) ||
      !get(cfg, "nbiot.sib2.ue_timers.t311", z.t311, err) || !get(cfg, "nbiot.sib2.ue_timers.n311", z.n311, err) ||
      !get(cfg, "nbiot.sib2.time_alignment_timer", z.time_alignment_timer, err)) {
    return false;
  }
  if (nof_ce_levels != 1) {
    err = "nbiot.sib2.nprach.nof_ce_levels must be 1: multiple coverage-enhancement levels are not implemented";
    return false;
  }
  if (z.nrs_power_dbm < -60 || z.nrs_power_dbm > 50) {
    err = "nbiot.sib2.npdsch.nrs_power_dbm must be -60..50";
    return false;
  }
  if (z.p0_nominal_npusch < -126 || z.p0_nominal_npusch > 24) {
    err = "nbiot.sib2.ul_power_control.p0_nominal_npusch must be -126..24";
    return false;
  }
  if (z.delta_preamble_msg3 < -1 || z.delta_preamble_msg3 > 6) {
    err = "nbiot.sib2.ul_power_control.delta_preamble_msg3 must be -1..6";
    return false;
  }
  if (z.group_assignment_npusch > 29) {
    err = "nbiot.sib2.npusch.group_assignment_npusch must be 0..29";
    return false;
  }
  // NPRACH has to fit in the UL carrier's 48 3.75 kHz subcarriers
  if (z.nprach_subcarrier_offset + z.nprach_num_subcarriers > 48) {
    err = "nbiot.sib2.nprach: subcarrier_offset + nof_subcarriers must not exceed 48";
    return false;
  }

  return true;
}

bool load_config_file(const std::string& path, cell_config& out, std::string& err)
{
  libconfig::Config cfg;
  try {
    cfg.readFile(path.c_str());
  } catch (const libconfig::FileIOException&) {
    err = "cannot read '" + path + "'";
    return false;
  } catch (const libconfig::ParseException& e) {
    err = "parse error in '" + path + "' line " + std::to_string(e.getLine()) + ": " + e.getError();
    return false;
  }
  return load_config(cfg, out, err);
}

/*************************************************************************************************
 * Derived values
 *************************************************************************************************/

srsran_mib_nb_t to_c_mib(const cell_config& c)
{
  srsran_mib_nb_t mib;
  memset(&mib, 0, sizeof(mib));
  mib.sched_info_sib1    = (uint8_t)c.sched_info_sib1;
  mib.sys_info_tag       = (uint8_t)c.sys_info_tag;
  mib.ac_barring         = c.ac_barring;
  mib.mode               = c.mode;
  mib.eutra_crs_seq_info = c.crs_seq_info;
  mib.raster_offset      = c.raster_offset;
  return mib;
}

uint32_t sib1_tbs_bits(const cell_config& c)
{
  srsran_mib_nb_t mib = to_c_mib(c);
  int             tbs = srsran_ra_nbiot_get_sib1_tbs(&mib);
  return tbs > 0 ? (uint32_t)tbs : 0;
}

uint32_t sib1_repetitions(const cell_config& c)
{
  srsran_mib_nb_t mib = to_c_mib(c);
  int             n   = srsran_ra_n_rep_sib1_nb(&mib);
  return n > 0 ? (uint32_t)n : 0;
}

/*************************************************************************************************
 * ASN.1 packing
 *************************************************************************************************/

namespace {

std::vector<uint8_t> bytes_to_bits(const uint8_t* p, size_t nbits)
{
  std::vector<uint8_t> bits(nbits);
  for (size_t i = 0; i < nbits; ++i) {
    bits[i] = (p[i / 8] >> (7 - (i % 8))) & 1;
  }
  return bits;
}

template <typename E, typename N>
bool set_by_number(E& e, N n, const char* what, std::string& err)
{
  if (!asn1::number_to_enum(e, n)) {
    err = std::string("value ") + std::to_string(n) + " is not encodable for " + what;
    return false;
  }
  return true;
}

} // namespace

bool pack_mib_bits(const cell_config& c, uint32_t hfn, uint32_t sfn, std::vector<uint8_t>& bits, std::string& err)
{
  asn1::rrc::bcch_bch_msg_nb_s bch;
  asn1::rrc::mib_nb_s&         m = bch.msg;

  m.sys_frame_num_msb_r13.from_number((sfn >> 6) & 0xF);
  m.hyper_sfn_lsb_r13.from_number(hfn & 0x3);
  m.sched_info_sib1_r13    = (uint8_t)c.sched_info_sib1;
  m.sys_info_value_tag_r13 = (uint8_t)c.sys_info_tag;
  m.ab_enabled_r13         = c.ac_barring;

  if (c.mode != SRSRAN_NBIOT_MODE_INBAND_SAME_PCI) {
    err = "only inband_same_pci MIB-NB packing is implemented";
    return false;
  }
  m.operation_mode_info_r13.set_inband_same_pci_r13().eutra_crs_seq_info_r13 = c.crs_seq_info;

  // Later-release extension fields that follow operationModeInfo in the 34-bit MIB-NB
  m.add_tx_sib1_r15          = false;
  m.ab_enabled_minus5_gc_r16 = false;
  m.part_earfcn_minus17.set(asn1::rrc::mib_nb_s::part_earfcn_minus17_c_::types::spare);
  m.part_earfcn_minus17.spare().from_number(0);
  m.spare.from_number(0);

  // Pack the MIB-NB body itself: BCCH-BCH-Message-NB::pack() additionally aligns to a byte boundary (34 -> 40 bits),
  // and the NPBCH transport block is exactly the 34 information bits.
  uint8_t           buf[16] = {};
  asn1::bit_ref     bref(buf, sizeof(buf));
  asn1::SRSASN_CODE ret = m.pack(bref);
  if (ret != asn1::SRSASN_SUCCESS) {
    err = "MIB-NB ASN.1 pack failed";
    return false;
  }
  if (bref.distance() != SRSRAN_MIB_NB_LEN) {
    err = "MIB-NB packed to " + std::to_string(bref.distance()) + " bits, expected " +
          std::to_string(SRSRAN_MIB_NB_LEN);
    return false;
  }
  bits = bytes_to_bits(buf, SRSRAN_MIB_NB_LEN);
  return true;
}

bool pack_sib1(const cell_config&   c,
               uint8_t              hyper_sfn_msb,
               std::vector<uint8_t>& out,
               size_t&              unpadded_len,
               std::string&         err)
{
  using namespace asn1::rrc;

  bcch_dl_sch_msg_nb_s msg;
  sib_type1_nb_s&      s = msg.msg.set_c1().set_sib_type1_r13();

  s.hyper_sfn_msb_r13.from_number(hyper_sfn_msb);

  // ---- cellAccessRelatedInfo
  auto& car = s.cell_access_related_info_r13;
  car.plmn_id_list_r13.resize(1);
  plmn_id_info_nb_r13_s& p = car.plmn_id_list_r13[0];
  p.plmn_id_r13.mcc_present = true;
  for (int i = 0; i < 3; ++i) {
    p.plmn_id_r13.mcc[i] = c.mcc[i] - '0';
  }
  p.plmn_id_r13.mnc.resize(c.mnc.size());
  for (size_t i = 0; i < c.mnc.size(); ++i) {
    p.plmn_id_r13.mnc[i] = c.mnc[i] - '0';
  }
  p.cell_reserved_for_oper_r13 = plmn_id_info_nb_r13_s::cell_reserved_for_oper_r13_opts::not_reserved;
  car.tac_r13.from_number(c.tac);
  car.cell_id_r13.from_number(c.cell_id);
  car.cell_barred_r13 = c.cell_barred ? sib_type1_nb_s::cell_access_related_info_r13_s_::cell_barred_r13_opts::barred
                                      : sib_type1_nb_s::cell_access_related_info_r13_s_::cell_barred_r13_opts::not_barred;
  car.intra_freq_resel_r13 = c.intra_freq_resel
                                 ? sib_type1_nb_s::cell_access_related_info_r13_s_::intra_freq_resel_r13_opts::allowed
                                 : sib_type1_nb_s::cell_access_related_info_r13_s_::intra_freq_resel_r13_opts::not_allowed;

  // ---- cellSelectionInfo
  s.cell_sel_info_r13.q_rx_lev_min_r13 = (int8_t)c.q_rx_lev_min;
  s.cell_sel_info_r13.q_qual_min_r13   = -34; // spec default; not yet configurable

  s.freq_band_ind_r13 = (uint16_t)c.band;

  // ---- in-band only
  s.eutra_ctrl_region_size_r13_present = true;
  switch (c.eutra_ctrl_region) {
    case 1:
      s.eutra_ctrl_region_size_r13 = sib_type1_nb_s::eutra_ctrl_region_size_r13_opts::n1;
      break;
    case 2:
      s.eutra_ctrl_region_size_r13 = sib_type1_nb_s::eutra_ctrl_region_size_r13_opts::n2;
      break;
    case 3:
      s.eutra_ctrl_region_size_r13 = sib_type1_nb_s::eutra_ctrl_region_size_r13_opts::n3;
      break;
    default:
      err = "eutra control region size must be 1..3";
      return false;
  }

  s.nrs_crs_pwr_offset_r13_present = true;
  bool found                       = false;
  for (uint32_t i = 0; i < sib_type1_nb_s::nrs_crs_pwr_offset_r13_e_::nof_types; ++i) {
    sib_type1_nb_s::nrs_crs_pwr_offset_r13_e_ e = (sib_type1_nb_s::nrs_crs_pwr_offset_r13_opts::options)i;
    if (std::fabs(e.to_number() - c.nrs_crs_pwr_offset_db) < 0.01) {
      s.nrs_crs_pwr_offset_r13 = e;
      found                    = true;
      break;
    }
  }
  if (!found) {
    err = "nrs_crs_power_offset " + std::to_string(c.nrs_crs_pwr_offset_db) + " dB is not an encodable value";
    return false;
  }

  // ---- SI scheduling
  s.sched_info_list_r13.resize(c.si_sched.size());
  for (size_t i = 0; i < c.si_sched.size(); ++i) {
    const si_sched_entry& in  = c.si_sched[i];
    sched_info_nb_r13_s&  out_ = s.sched_info_list_r13[i];
    if (!set_by_number(out_.si_periodicity_r13, (uint16_t)in.periodicity_rf, "si_periodicity", err) ||
        !set_by_number(out_.si_repeat_pattern_r13, (uint8_t)in.repetition_pattern, "si_repetition_pattern", err) ||
        !set_by_number(out_.si_tb_r13, (uint16_t)in.tb_bits, "si_tb", err)) {
      return false;
    }
    for (uint32_t sib : in.sib_mapping) {
      sib_type_nb_r13_e t;
      if (!asn1::number_to_enum(t, (uint8_t)sib)) {
        err = "SIB" + std::to_string(sib) + " cannot be mapped into an NB-IoT SI message";
        return false;
      }
      out_.sib_map_info_r13.push_back(t);
    }
  }
  if (!set_by_number(s.si_win_len_r13, (uint16_t)c.si_window_ms, "si_window_length", err)) {
    return false;
  }
  s.si_radio_frame_offset_r13_present = true;
  s.si_radio_frame_offset_r13         = (uint8_t)c.si_radio_frame_offset;

  // ---- pack
  uint8_t           buf[128] = {};
  asn1::bit_ref     bref(buf, sizeof(buf));
  asn1::SRSASN_CODE ret = msg.pack(bref);
  if (ret != asn1::SRSASN_SUCCESS) {
    err = "SIB1-NB ASN.1 pack failed";
    return false;
  }
  bref.align_bytes_zero();
  unpadded_len = bref.distance_bytes();

  const uint32_t tbs_bits = sib1_tbs_bits(c);
  if (tbs_bits == 0) {
    err = "schedulingInfoSIB1 " + std::to_string(c.sched_info_sib1) + " has no transport block size";
    return false;
  }
  if (unpadded_len * 8 > tbs_bits) {
    err = "SIB1-NB is " + std::to_string(unpadded_len * 8) + " bits but schedulingInfoSIB1=" +
          std::to_string(c.sched_info_sib1) + " only carries " + std::to_string(tbs_bits) +
          " bits; choose a schedulingInfoSIB1 with a larger TBS (3..5 = 328, 6..8 = 440, 9..11 = 680)";
    return false;
  }

  out.assign(tbs_bits / 8, 0);
  memcpy(out.data(), buf, unpadded_len);
  return true;
}

/*************************************************************************************************
 * SIB2-NB
 *************************************************************************************************/

namespace {

std::string fmt_num(double v)
{
  char b[32];
  snprintf(b, sizeof(b), "%g", v);
  return b;
}

/// Set an ASN.1 enumeration from a number by matching to_number() (with a small tolerance for the float ones).
template <typename E>
bool set_num(E& e, double v, const std::string& key, std::string& err)
{
  for (uint32_t i = 0; i < E::nof_types; ++i) {
    e = (typename E::options)i;
    double n = (double)e.to_number();
    if (std::fabs(n - v) <= 1e-3 * std::max(1.0, std::fabs(v))) {
      return true;
    }
  }
  err = "value " + fmt_num(v) + " of " + key + " is not encodable";
  return false;
}

/// Set an ASN.1 enumeration from its ASN.1 name (as printed by to_string()).
template <typename E>
bool set_str(E& e, const std::string& s, const std::string& key, std::string& err)
{
  if (!asn1::string_to_enum(e, s)) {
    err = "value '" + s + "' of " + key + " is not a valid choice";
    return false;
  }
  return true;
}

} // namespace

uint32_t sib2_tbs_bits(const cell_config& c)
{
  return c.si_sched.empty() ? 0 : c.si_sched[0].tb_bits;
}

bool pack_sib2(const cell_config& c, std::vector<uint8_t>& out, size_t& unpadded_len, std::string& err)
{
  using namespace asn1::rrc;
  const sib2_config& z = c.sib2;
  const std::string  k = "nbiot.sib2.";

  bcch_dl_sch_msg_nb_s   msg;
  sys_info_nb_r13_ies_s& ies = msg.msg.set_c1().set_sys_info_r13().crit_exts.set_sys_info_r13();
  ies.sib_type_and_info_r13.resize(1);
  sib_type2_nb_r13_s&         s2 = ies.sib_type_and_info_r13[0].set_sib2_r13();
  rr_cfg_common_sib_nb_r13_s& rr = s2.rr_cfg_common_r13;

  // ---- RACH
  rach_cfg_common_nb_r13_s& rach = rr.rach_cfg_common_r13;
  if (!set_num(rach.preamb_trans_max_ce_r13, z.preamble_trans_max_ce, k + "rach.preamble_trans_max_ce", err) ||
      !set_num(rach.pwr_ramp_params_r13.preamb_init_rx_target_pwr,
               z.preamble_init_rx_target_power,
               k + "rach.preamble_init_rx_target_power",
               err) ||
      !set_num(rach.pwr_ramp_params_r13.pwr_ramp_step, z.power_ramping_step_db, k + "rach.power_ramping_step", err)) {
    return false;
  }
  rach.rach_info_list_r13.resize(1);
  if (!set_num(rach.rach_info_list_r13[0].ra_resp_win_size_r13, z.ra_response_window, k + "rach.ra_response_window", err) ||
      !set_num(rach.rach_info_list_r13[0].mac_contention_resolution_timer_r13,
               z.mac_contention_timer,
               k + "rach.mac_contention_timer",
               err)) {
    return false;
  }

  // ---- BCCH / PCCH
  rr.bcch_cfg_r13.mod_period_coeff_r13 = bcch_cfg_nb_r13_s::mod_period_coeff_r13_opts::n16;
  if (!set_num(rr.pcch_cfg_r13.default_paging_cycle_r13, z.default_paging_cycle_rf, k + "pcch.default_paging_cycle_rf", err) ||
      !set_str(rr.pcch_cfg_r13.nb_r13, z.nb, k + "pcch.nB", err) ||
      !set_num(rr.pcch_cfg_r13.npdcch_num_repeat_paging_r13,
               z.npdcch_num_repetitions_paging,
               k + "pcch.npdcch_num_repetitions_paging",
               err)) {
    return false;
  }

  // ---- NPRACH
  nprach_cfg_sib_nb_r13_s& npr = rr.nprach_cfg_r13;
  if (std::fabs(z.nprach_cp_length_us - 66.7) < 0.1) {
    npr.nprach_cp_len_r13 = nprach_cfg_sib_nb_r13_s::nprach_cp_len_r13_opts::us66dot7;
  } else if (std::fabs(z.nprach_cp_length_us - 266.7) < 0.1) {
    npr.nprach_cp_len_r13 = nprach_cfg_sib_nb_r13_s::nprach_cp_len_r13_opts::us266dot7;
  } else {
    err = "value " + fmt_num(z.nprach_cp_length_us) + " of " + k + "nprach.cp_length_us must be 66.7 or 266.7";
    return false;
  }
  npr.rsrp_thress_prach_info_list_r13_present = false;
  npr.nprach_params_list_r13.resize(1);
  nprach_params_nb_r13_s& np = npr.nprach_params_list_r13[0];
  if (!set_num(np.nprach_periodicity_r13, z.nprach_periodicity_ms, k + "nprach.periodicity_ms", err) ||
      !set_num(np.nprach_start_time_r13, z.nprach_start_time_ms, k + "nprach.start_time_ms", err) ||
      !set_num(np.nprach_subcarrier_offset_r13, z.nprach_subcarrier_offset, k + "nprach.subcarrier_offset", err) ||
      !set_num(np.nprach_num_subcarriers_r13, z.nprach_num_subcarriers, k + "nprach.nof_subcarriers", err) ||
      !set_str(np.nprach_subcarrier_msg3_range_start_r13,
               z.msg3_subcarrier_range_start,
               k + "nprach.msg3_subcarrier_range_start",
               err) ||
      !set_num(np.max_num_preamb_attempt_ce_r13, z.max_preamble_attempts, k + "nprach.max_preamble_attempts", err) ||
      !set_num(np.num_repeats_per_preamb_attempt_r13,
               z.num_repetitions_per_preamble,
               k + "nprach.num_repetitions_per_preamble",
               err) ||
      !set_num(np.npdcch_num_repeats_ra_r13, z.npdcch_num_repetitions_ra, k + "nprach.npdcch_num_repetitions_ra", err) ||
      !set_num(np.npdcch_start_sf_css_ra_r13, z.npdcch_start_sf_css_ra, k + "nprach.npdcch_start_sf_css_ra", err) ||
      !set_str(np.npdcch_offset_ra_r13, z.npdcch_offset_ra, k + "nprach.npdcch_offset_ra", err)) {
    return false;
  }

  // ---- NPDSCH / NPUSCH
  rr.npdsch_cfg_common_r13.nrs_pwr_r13 = (int8_t)z.nrs_power_dbm;

  npusch_cfg_common_nb_r13_s& npu = rr.npusch_cfg_common_r13;
  npu.ack_nack_num_repeats_msg4_r13.resize(1);
  if (!set_num(npu.ack_nack_num_repeats_msg4_r13[0], z.ack_nack_num_repetitions_msg4, k + "npusch.ack_nack_num_repetitions_msg4", err)) {
    return false;
  }
  // DMRS base sequence / cyclic shift are deliberately NOT signalled: the UE then derives them from the cell id
  // (TS 36.211 10.1.4.1.2), which is what the Phase 2 receiver will assume.
  npu.srs_sf_cfg_r13_present                  = false;
  npu.dmrs_cfg_r13_present                    = false;
  npu.ul_ref_sigs_npusch_r13.group_hop_enabled_r13   = z.group_hopping_enabled;
  npu.ul_ref_sigs_npusch_r13.group_assign_npusch_r13 = (uint8_t)z.group_assignment_npusch;

  rr.dl_gap_r13_present = false;

  // ---- UL power control
  rr.ul_pwr_ctrl_common_r13.p0_nominal_npusch_r13 = (int8_t)z.p0_nominal_npusch;
  rr.ul_pwr_ctrl_common_r13.delta_preamb_msg3_r13 = (int8_t)z.delta_preamble_msg3;
  if (!set_num(rr.ul_pwr_ctrl_common_r13.alpha_r13, z.alpha, k + "ul_power_control.alpha", err)) {
    return false;
  }

  // ---- UE timers
  ue_timers_and_consts_nb_r13_s& t = s2.ue_timers_and_consts_r13;
  if (!set_num(t.t300_r13, z.t300, k + "ue_timers.t300", err) || !set_num(t.t301_r13, z.t301, k + "ue_timers.t301", err) ||
      !set_num(t.t310_r13, z.t310, k + "ue_timers.t310", err) || !set_num(t.n310_r13, z.n310, k + "ue_timers.n310", err) ||
      !set_num(t.t311_r13, z.t311, k + "ue_timers.t311", err) || !set_num(t.n311_r13, z.n311, k + "ue_timers.n311", err)) {
    return false;
  }

  // ---- frequency info / timing advance
  s2.freq_info_r13.ul_carrier_freq_r13_present = false; // default duplex spacing for the band
  s2.freq_info_r13.add_spec_emission_r13       = 1;
  if (!set_str(s2.time_align_timer_common_r13, z.time_alignment_timer, k + "time_alignment_timer", err)) {
    return false;
  }

  // ---- pack and fit
  uint8_t           buf[256] = {};
  asn1::bit_ref     bref(buf, sizeof(buf));
  asn1::SRSASN_CODE ret = msg.pack(bref);
  if (ret != asn1::SRSASN_SUCCESS) {
    err = "SIB2-NB ASN.1 pack failed";
    return false;
  }
  bref.align_bytes_zero();
  unpadded_len = bref.distance_bytes();

  const uint32_t tb_bits = sib2_tbs_bits(c);
  if (unpadded_len * 8 > tb_bits) {
    static const uint32_t sizes[] = {56, 120, 208, 256, 328, 440, 552, 680};
    std::string           fit     = "no si_tb is large enough";
    for (uint32_t sz : sizes) {
      if (unpadded_len * 8 <= sz) {
        fit = "the smallest that fits is " + std::to_string(sz);
        break;
      }
    }
    err = "SIB2-NB is " + std::to_string(unpadded_len * 8) + " bits but nbiot.sib1.sched_info[0].si_tb is " +
          std::to_string(tb_bits) + "; " + fit;
    return false;
  }
  out.assign(tb_bits / 8, 0);
  memcpy(out.data(), buf, unpadded_len);
  return true;
}

} // namespace nbiot
