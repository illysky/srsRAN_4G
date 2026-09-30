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

#include "srsenb/hdr/phy/emtc_config.h"
#include "srsenb/hdr/stack/rrc/rrc_config.h"
#include "srsran/asn1/rrc.h"
#include <cmath>
#include <libconfig.h++>

extern "C" {
#include "srsran/phy/common/phy_common.h"
}

namespace srsenb {
namespace emtc {

namespace {

template <typename T>
bool get(const libconfig::Config& c, const std::string& path, T& v, std::string& err)
{
  if (!c.exists(path)) {
    return true; // optional: the default in config stays
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

bool get_list(const libconfig::Config& c, const std::string& path, std::vector<uint32_t>& v, std::string& err)
{
  if (!c.exists(path)) {
    return true;
  }
  const libconfig::Setting& s = c.lookup(path);
  if (!s.isArray() && !s.isList()) {
    err = "key '" + path + "' must be a list of integers";
    return false;
  }
  v.clear();
  for (int i = 0; i < s.getLength(); i++) {
    v.push_back((uint32_t)(int)s[i]);
  }
  return true;
}

/// Sets an ASN.1 enumerated to the option whose number is n
template <typename E, typename N>
bool set_enum(E& e, N n, const char* what, std::string& err)
{
  for (uint32_t i = 0; i < E::nof_types; i++) {
    E t = (typename E::options)i;
    if (std::fabs((double)t.to_number() - (double)n) < 1e-6) {
      e = t;
      return true;
    }
  }
  err = std::string(what) + ": " + std::to_string(n) + " is not an allowed value";
  return false;
}

bool one_of(uint32_t v, std::initializer_list<uint32_t> l)
{
  for (uint32_t x : l) {
    if (v == x) {
      return true;
    }
  }
  return false;
}

template <typename M>
bool pack(M& msg, uint32_t tbs, std::vector<uint8_t>& out, size_t& len, const char* what, std::string& err)
{
  uint8_t       buf[256] = {};
  asn1::bit_ref bref(buf, sizeof(buf));
  if (msg.pack(bref) != asn1::SRSASN_SUCCESS) {
    err = std::string(what) + ": ASN.1 packing failed";
    return false;
  }
  len = (size_t)bref.distance_bytes();
  if (len * 8 > tbs) {
    err = std::string(what) + " is " + std::to_string(len) + " bytes, more than its " + std::to_string(tbs) +
          "-bit transport block";
    return false;
  }
  out.assign(buf, buf + tbs / 8);
  return true;
}

} // namespace

bool load(const std::string& path, config& o, std::string& err)
{
  libconfig::Config c;
  try {
    c.readFile(path.c_str());
  } catch (const libconfig::FileIOException&) {
    err = "cannot read the file";
    return false;
  } catch (const libconfig::ParseException& e) {
    err = std::string("line ") + std::to_string(e.getLine()) + ": " + e.getError();
    return false;
  }
  c.setAutoConvert(true);

  bool ok = get(c, "emtc.sched_info_sib1_br", o.sched_info_sib1_br, err) &&
            get(c, "emtc.start_symbol", o.start_symbol, err) && get(c, "emtc.si_window_ms", o.si_window_ms, err) &&
            get(c, "emtc.si_repetition_rf", o.si_repetition_rf, err) &&
            get(c, "emtc.rach.first_preamble", o.first_preamble, err) &&
            get(c, "emtc.rach.last_preamble", o.last_preamble, err) &&
            get(c, "emtc.rach.ra_response_window_sf", o.ra_response_window_sf, err) &&
            get(c, "emtc.rach.contention_timer_sf", o.contention_timer_sf, err) &&
            get(c, "emtc.rach.preamble_trans_max_ce", o.preamble_trans_max_ce, err) &&
            get(c, "emtc.prach.config_index", o.prach_config_index, err) &&
            get(c, "emtc.prach.freq_offset", o.prach_freq_offset, err) &&
            get(c, "emtc.prach.repetitions", o.prach_repetitions, err) &&
            get(c, "emtc.prach.max_attempts", o.max_preamble_attempts, err) &&
            get_list(c, "emtc.prach.mpdcch_narrowbands", o.mpdcch_nb_ra, err) &&
            get(c, "emtc.prach.mpdcch_repetitions", o.mpdcch_rep_ra, err) &&
            get(c, "emtc.prach.mpdcch_start_sf", o.mpdcch_start_sf_ra, err) &&
            get(c, "emtc.pucch.n1_an", o.n1_pucch_an, err) && get(c, "emtc.pucch.repetitions_msg4", o.pucch_rep_msg4, err) &&
            get(c, "emtc.paging.narrowband", o.paging_nb, err) &&
            get(c, "emtc.paging.mpdcch_repetitions", o.mpdcch_rep_paging, err) &&
            get(c, "emtc.pdsch_max_repetitions", o.pdsch_max_rep, err) &&
            get(c, "emtc.pusch_max_repetitions", o.pusch_max_rep, err);
  if (!ok) {
    return false;
  }
  if (c.exists("emtc.si")) {
    const libconfig::Setting& l = c.lookup("emtc.si");
    o.si.clear();
    for (int i = 0; i < l.getLength(); i++) {
      config::si_msg m;
      l[i].lookupValue("periodicity_rf", m.periodicity_rf);
      l[i].lookupValue("narrowband", m.nb);
      l[i].lookupValue("tbs", m.tbs);
      o.si.push_back(m);
    }
  }
  if (o.si.empty()) {
    o.si.push_back(config::si_msg{});
  }
  if (o.mpdcch_nb_ra.empty()) {
    o.mpdcch_nb_ra.push_back(3);
  }

  if (srsran_emtc_sib1_br_repetitions(o.sched_info_sib1_br) == 0) {
    err = "emtc.sched_info_sib1_br must be 1..18";
    return false;
  }
  if (o.start_symbol < 1 || o.start_symbol > 3) {
    err = "emtc.start_symbol must be 1..3 (bandwidth above 10 PRB)";
    return false;
  }
  if (!one_of(o.si_window_ms, {20, 40, 60, 80, 120, 160, 200})) {
    err = "emtc.si_window_ms must be 20, 40, 60, 80, 120, 160 or 200";
    return false;
  }
  if (o.si.size() > SRSRAN_EMTC_MAX_SI || o.si.size() * o.si_window_ms > o.si[0].periodicity_rf * 10) {
    err = "emtc.si: the SI windows do not fit the SI periodicity";
    return false;
  }
  if (o.first_preamble > o.last_preamble || o.last_preamble > 63) {
    err = "emtc.rach: first_preamble..last_preamble must be a range within 0..63";
    return false;
  }
  return true;
}

bool build_bcast(const rrc_cfg_t& rrc_cfg,
                 uint32_t         nof_prb,
                 uint32_t         pci,
                 const config&    cfg,
                 bcast&           out,
                 std::string&     err)
{
  using namespace asn1::rrc;

  if (rrc_cfg.cell_list.empty()) {
    err = "no LTE cell";
    return false;
  }
  const cell_cfg_t& cell = rrc_cfg.cell_list[0];
  const uint32_t    n_nb = srsran_emtc_nof_nb(nof_prb);
  auto              nb_ok = [&](uint32_t nb, const char* what) {
    if (nb >= n_nb || srsran_emtc_nb_in_centre(nof_prb, nb)) {
      err = std::string(what) + ": narrowband " + std::to_string(nb) + " is not one of the " + std::to_string(n_nb) +
            " narrowbands, or overlaps the centre 72 subcarriers";
      return false;
    }
    return true;
  };
  for (const auto& s : cfg.si) {
    if (!nb_ok(s.nb, "emtc.si")) {
      return false;
    }
  }
  for (uint32_t nb : cfg.mpdcch_nb_ra) {
    if (!nb_ok(nb, "emtc.prach.mpdcch_narrowbands")) {
      return false;
    }
  }
  if (!nb_ok(cfg.paging_nb, "emtc.paging.narrowband")) {
    return false;
  }

  const sib_type2_s& lte_sib2 = rrc_cfg.sibs[1].sib2();
  if (cfg.prach_config_index != lte_sib2.rr_cfg_common.prach_cfg.prach_cfg_info.prach_cfg_idx ||
      cfg.prach_freq_offset != lte_sib2.rr_cfg_common.prach_cfg.prach_cfg_info.prach_freq_offset) {
    err = "emtc.prach: config_index/freq_offset must be the LTE cell's (" +
          std::to_string(lte_sib2.rr_cfg_common.prach_cfg.prach_cfg_info.prach_cfg_idx) + "/" +
          std::to_string(lte_sib2.rr_cfg_common.prach_cfg.prach_cfg_info.prach_freq_offset) +
          "): LTE-M preambles are detected on the LTE PRACH";
    return false;
  }
  if (cfg.first_preamble < lte_sib2.rr_cfg_common.rach_cfg_common.preamb_info.nof_ra_preambs.to_number()) {
    err = "emtc.rach.first_preamble overlaps the LTE preambles (0.." +
          std::to_string(lte_sib2.rr_cfg_common.rach_cfg_common.preamb_info.nof_ra_preambs.to_number() - 1) + ")";
    return false;
  }

  // ---- schedule
  out.sched                    = {};
  out.sched.nof_prb            = nof_prb;
  out.sched.pci                = pci;
  out.sched.sched_info_sib1_br = cfg.sched_info_sib1_br;
  out.sched.si_window_ms       = cfg.si_window_ms;
  out.sched.si_repetition_rf   = cfg.si_repetition_rf;
  out.sched.nof_si             = (uint32_t)cfg.si.size();
  for (size_t i = 0; i < cfg.si.size(); i++) {
    out.sched.si[i] = {cfg.si[i].periodicity_rf, cfg.si[i].nb, cfg.si[i].tbs};
  }
  out.start_symbol = cfg.start_symbol;

  // ---- SIB1-BR: the cell's SIB1, its SI list replaced by the BR one, and bandwidthReducedAccessRelatedInfo
  sib_type1_s s = rrc_cfg.sib1;
  s.cell_access_related_info.cell_id.from_number((rrc_cfg.enb_id << 8u) + cell.cell_id);
  s.cell_access_related_info.tac.from_number(cell.tac);
  s.freq_band_ind = (uint8_t)srsran_band_get_band(cell.dl_earfcn);
  s.cell_access_related_info.cell_barred.value =
      cell.barred ? sib_type1_s::cell_access_related_info_s_::cell_barred_opts::barred
                  : sib_type1_s::cell_access_related_info_s_::cell_barred_opts::not_barred;
  s.sched_info_list.resize(cfg.si.size());
  for (size_t i = 0; i < cfg.si.size(); i++) {
    if (!set_enum(s.sched_info_list[i].si_periodicity, cfg.si[i].periodicity_rf, "emtc.si.periodicity_rf", err)) {
      return false;
    }
    s.sched_info_list[i].sib_map_info.resize(0);
  }
  s.non_crit_ext_present                                                   = true;
  s.non_crit_ext.non_crit_ext_present                                      = true;
  s.non_crit_ext.non_crit_ext.non_crit_ext_present                         = true;
  s.non_crit_ext.non_crit_ext.non_crit_ext.non_crit_ext_present            = true;
  s.non_crit_ext.non_crit_ext.non_crit_ext.non_crit_ext.non_crit_ext_present = true;
  sib_type1_v1310_ies_s& v1310                                             = s.non_crit_ext.non_crit_ext.non_crit_ext.non_crit_ext.non_crit_ext;
  v1310.bw_reduced_access_related_info_r13_present                         = true;
  auto& br                                                                 = v1310.bw_reduced_access_related_info_r13;
  if (!set_enum(br.si_win_len_br_r13, cfg.si_window_ms, "emtc.si_window_ms", err)) {
    return false;
  }
  // everyRF has no number in the ASN.1 library: go by the index (everyRF, every2ndRF, every4thRF, every8thRF)
  uint32_t rep_idx = 0;
  while ((1U << rep_idx) < cfg.si_repetition_rf && rep_idx < 3) {
    rep_idx++;
  }
  if ((1U << rep_idx) != cfg.si_repetition_rf) {
    err = "emtc.si_repetition_rf must be 1, 2, 4 or 8";
    return false;
  }
  br.si_repeat_pattern_r13 =
      (sib_type1_v1310_ies_s::bw_reduced_access_related_info_r13_s_::si_repeat_pattern_r13_e_::options)rep_idx;
  br.sched_info_list_br_r13_present = true;
  br.sched_info_list_br_r13.resize(cfg.si.size());
  for (size_t i = 0; i < cfg.si.size(); i++) {
    br.sched_info_list_br_r13[i].si_nb_r13 = (uint8_t)(cfg.si[i].nb + 1);
    if (!set_enum(br.sched_info_list_br_r13[i].si_tbs_r13, cfg.si[i].tbs, "emtc.si.tbs", err)) {
      return false;
    }
  }
  br.start_symbol_br_r13   = (uint8_t)cfg.start_symbol;
  br.si_hop_cfg_common_r13 = sib_type1_v1310_ies_s::bw_reduced_access_related_info_r13_s_::si_hop_cfg_common_r13_opts::off;

  {
    bcch_dl_sch_msg_br_s msg;
    msg.msg.set_c1().set_sib_type1_br_r13() = s;
    if (!pack(msg, srsran_emtc_sib1_br_tbs(cfg.sched_info_sib1_br), out.sib1, out.sib1_len, "SIB1-BR", err)) {
      return false;
    }
  }

  // ---- SIB2 for BL UEs: the cell's SIB2 with the Rel-13 BR fields
  sib_type2_s sib2                          = lte_sib2;
  sib2.rr_cfg_common.prach_cfg.root_seq_idx = cell.root_seq_idx;
  if (sib2.freq_info.ul_carrier_freq_present) {
    sib2.freq_info.ul_carrier_freq = cell.ul_earfcn;
  }

  rach_cfg_common_s& rach             = sib2.rr_cfg_common.rach_cfg_common;
  rach.ext                            = true;
  rach.preamb_trans_max_ce_r13_present = true;
  if (!set_enum(rach.preamb_trans_max_ce_r13, cfg.preamble_trans_max_ce, "emtc.rach.preamble_trans_max_ce", err)) {
    return false;
  }
  rach.rach_ce_level_info_list_r13.set_present();
  rach.rach_ce_level_info_list_r13->resize(1);
  rach_ce_level_info_r13_s& lvl                    = (*rach.rach_ce_level_info_list_r13)[0];
  lvl.preamb_map_info_r13.first_preamb_r13         = (uint8_t)cfg.first_preamble;
  lvl.preamb_map_info_r13.last_preamb_r13          = (uint8_t)cfg.last_preamble;
  lvl.rar_hop_cfg_r13                              = rach_ce_level_info_r13_s::rar_hop_cfg_r13_opts::off;
  if (!set_enum(lvl.ra_resp_win_size_r13, cfg.ra_response_window_sf, "emtc.rach.ra_response_window_sf", err) ||
      !set_enum(lvl.mac_contention_resolution_timer_r13, cfg.contention_timer_sf, "emtc.rach.contention_timer_sf", err)) {
    return false;
  }

  rr_cfg_common_sib_s& rr = sib2.rr_cfg_common;
  rr.ext                  = true;

  rr.pcch_cfg_v1310.set_present();
  rr.pcch_cfg_v1310->paging_narrow_bands_r13 = (uint8_t)(cfg.paging_nb + 1);
  if (!set_enum(rr.pcch_cfg_v1310->mpdcch_num_repeat_paging_r13, cfg.mpdcch_rep_paging, "emtc.paging.mpdcch_repetitions", err)) {
    return false;
  }

  rr.pdsch_cfg_common_v1310.set_present();
  rr.pdsch_cfg_common_v1310->pdsch_max_num_repeat_cemode_a_r13_present = true;
  if (!set_enum(rr.pdsch_cfg_common_v1310->pdsch_max_num_repeat_cemode_a_r13, cfg.pdsch_max_rep, "emtc.pdsch_max_repetitions", err)) {
    return false;
  }
  rr.pusch_cfg_common_v1310.set_present();
  rr.pusch_cfg_common_v1310->pusch_max_num_repeat_cemode_a_r13_present = true;
  if (!set_enum(rr.pusch_cfg_common_v1310->pusch_max_num_repeat_cemode_a_r13, cfg.pusch_max_rep, "emtc.pusch_max_repetitions", err)) {
    return false;
  }

  rr.prach_cfg_common_v1310.set_present();
  prach_cfg_sib_v1310_s& pr = *rr.prach_cfg_common_v1310;
  pr.rsrp_thress_prach_info_list_r13.resize(1);
  pr.rsrp_thress_prach_info_list_r13[0] = 0; // unused with a single CE level, but mandatory
  pr.mpdcch_start_sf_css_ra_r13_present = true;
  if (!set_enum(pr.mpdcch_start_sf_css_ra_r13.set_fdd_r13(), cfg.mpdcch_start_sf_ra, "emtc.prach.mpdcch_start_sf", err)) {
    return false;
  }
  pr.prach_params_list_ce_r13.resize(1);
  prach_params_ce_r13_s& pp                 = pr.prach_params_list_ce_r13[0];
  pp.prach_cfg_idx_r13                      = (uint8_t)cfg.prach_config_index;
  pp.prach_freq_offset_r13                  = (uint8_t)cfg.prach_freq_offset;
  pp.max_num_preamb_attempt_ce_r13_present  = true;
  if (!set_enum(pp.max_num_preamb_attempt_ce_r13, cfg.max_preamble_attempts, "emtc.prach.max_attempts", err) ||
      !set_enum(pp.num_repeat_per_preamb_attempt_r13, cfg.prach_repetitions, "emtc.prach.repetitions", err) ||
      !set_enum(pp.mpdcch_num_repeat_ra_r13, cfg.mpdcch_rep_ra, "emtc.prach.mpdcch_repetitions", err)) {
    return false;
  }
  pp.mpdcch_nbs_to_monitor_r13.resize(cfg.mpdcch_nb_ra.size());
  for (size_t i = 0; i < cfg.mpdcch_nb_ra.size(); i++) {
    pp.mpdcch_nbs_to_monitor_r13[i] = (uint8_t)(cfg.mpdcch_nb_ra[i] + 1);
  }
  pp.prach_hop_cfg_r13 = prach_params_ce_r13_s::prach_hop_cfg_r13_opts::off;

  rr.pucch_cfg_common_v1310.set_present();
  rr.pucch_cfg_common_v1310->n1_pucch_an_info_list_r13.resize(1);
  rr.pucch_cfg_common_v1310->n1_pucch_an_info_list_r13[0]        = (uint16_t)cfg.n1_pucch_an;
  rr.pucch_cfg_common_v1310->pucch_num_repeat_ce_msg4_level0_r13_present = true;
  if (!set_enum(rr.pucch_cfg_common_v1310->pucch_num_repeat_ce_msg4_level0_r13, cfg.pucch_rep_msg4, "emtc.pucch.repetitions_msg4", err)) {
    return false;
  }

  {
    bcch_dl_sch_msg_br_s msg;
    sys_info_r8_ies_s&   ies = msg.msg.set_c1().set_sys_info_br_r13().crit_exts.set_sys_info_r8();
    ies.sib_type_and_info.resize(1);
    ies.sib_type_and_info[0].set_sib2() = sib2;
    out.si.resize(1);
    out.si_len.resize(1);
    if (!pack(msg, cfg.si[0].tbs, out.si[0], out.si_len[0], "SI message 1 (SIB2)", err)) {
      return false;
    }
  }
  if (cfg.si.size() > 1) {
    err = "emtc.si: only one SI message (SIB2) is implemented";
    return false;
  }
  return true;
}

} // namespace emtc
} // namespace srsenb
