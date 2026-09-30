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

#include "srsenb/hdr/phy/nbiot_mac.h"
#include "srsran/common/standard_streams.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace srsenb {

namespace {
// The response is one MAC RAR, 7 octets with its subheader, which is the transport block of I_TBS 4 in one subframe
constexpr uint32_t RAR_I_TBS  = 4;
constexpr uint32_t RAR_I_SF   = 0;
constexpr uint32_t RAR_TBS    = 56;
constexpr uint32_t MAX_PENDING = 64;
} // namespace

bool nbiot_ra_config::from_cell(const nbiot::cell_config& cell, nbiot_ra_config& out, std::string& err)
{
  out.nprach_n_rep  = cell.sib2.num_repetitions_per_preamble;
  out.nprach_format = cell.sib2.nprach_cp_length_us > 100.0 ? 1 : 0;
  out.r_max         = cell.sib2.npdcch_num_repetitions_ra;
  out.window_pp     = cell.sib2.ra_response_window;

  const double g = cell.sib2.npdcch_start_sf_css_ra;
  out.g_halves   = (uint32_t)std::lround(2.0 * g);
  if (std::fabs(2.0 * g - (double)out.g_halves) > 1e-6) {
    err = "npdcch_start_sf_css_ra " + std::to_string(g) + " is not a multiple of 0.5";
    return false;
  }

  const std::string& o = cell.sib2.npdcch_offset_ra;
  if (o == "zero") {
    out.offset_eighths = 0;
  } else if (o == "oneEighth") {
    out.offset_eighths = 1;
  } else if (o == "oneFourth") {
    out.offset_eighths = 2;
  } else if (o == "threeEighth") {
    out.offset_eighths = 3;
  } else {
    err = "npdcch_offset_ra '" + o + "' is not one of zero, oneEighth, oneFourth, threeEighth";
    return false;
  }

  uint32_t period = 0;
  srsran_nbiot_search_space_start(out.r_max, out.g_halves, out.offset_eighths, 0, &period);
  if (period == 0) {
    err = "npdcch_num_repetitions_ra " + std::to_string(out.r_max) + " with npdcch_start_sf_css_ra " +
          std::to_string(g) + " gives a search space period below 4 subframes";
    return false;
  }
  if (srsran_nbiot_dci_rep_for_rmax(out.r_max) < 0) {
    err = "npdcch_num_repetitions_ra " + std::to_string(out.r_max) + " has no DCI subframe repetition number";
    return false;
  }
  // The NPDCCH candidate of R = Rmax starts at a multiple of four NPDCCH subframes (or is a single one): the
  // scrambler re-initialisation of 36.211 10.2.5.2 is then the same for every reading of the standard
  if (out.r_max != 1 && out.r_max < 4) {
    err = "npdcch_num_repetitions_ra " + std::to_string(out.r_max) + " is not supported (use 1 or at least 4)";
    return false;
  }
  if (out.window_pp == 0) {
    err = "ra_response_window is 0";
    return false;
  }
  return true;
}

nbiot_mac::nbiot_mac(srslog::basic_logger& logger_) : logger(logger_) {}

nbiot_mac::~nbiot_mac()
{
  if (initiated) {
    srsran_nbiot_dlch_free(&dlch);
  }
}

bool nbiot_mac::init(const nbiot_ra_config& cfg_, const srsran_nbiot_cell_t& nb_cell, srsran_nbiot_dl_sched_t* sched_, std::string& err)
{
  if (sched_ == nullptr) {
    err = "no downlink schedule";
    return false;
  }
  cfg   = cfg_;
  sched = sched_;
  if (srsran_nbiot_dlch_init(&dlch, &nb_cell, 3) != SRSRAN_SUCCESS) {
    err = "the NB-IoT downlink channel coder refused the cell";
    return false;
  }
  if (srsran_nbiot_npdsch_tbs(RAR_I_TBS, RAR_I_SF) != RAR_TBS) {
    err = "internal: transport block size of the random access response";
    srsran_nbiot_dlch_free(&dlch);
    return false;
  }
  if (srsran_nbiot_msg3_grant_check(&cfg.msg3) != SRSRAN_SUCCESS) {
    err = "the Msg3 grant in the configuration is not valid";
    srsran_nbiot_dlch_free(&dlch);
    return false;
  }
  initiated = true;
  return true;
}

uint32_t nbiot_mac::preamble_duration_us(uint32_t n_rep, uint32_t format)
{
  // A symbol group is cyclic prefix plus 5 * 8192 Ts of sequence (Ts = 1 / 30.72 MHz); four groups make one repetition.
  // After every 64 repetitions there is a gap of 40 ms, to be inserted if more repetitions follow (36.211 10.1.6).
  const uint32_t cp_ns    = format == 0 ? 66667 : 266667;
  const uint32_t seq_ns   = 1333333;
  const uint64_t group_ns = cp_ns + seq_ns;
  const uint64_t ns       = (uint64_t)n_rep * 4 * group_ns;
  const uint32_t gaps     = n_rep > 0 ? (n_rep - 1) / 64 : 0;
  return (uint32_t)((ns + 500) / 1000) + gaps * 40000;
}

uint64_t nbiot_mac::window_start(uint64_t start, uint32_t n_rep, uint32_t format)
{
  const uint32_t us = preamble_duration_us(n_rep, format);
  // the subframe that contains the last sample, then 4 (fewer than 64 repetitions) or 41 subframes later
  const uint64_t end_sf = start + (us - 1) / 1000;
  return end_sf + (n_rep >= 64 ? 41 : 4);
}

uint16_t nbiot_mac::alloc_tc_rnti()
{
  // 0x003D..0xFFF3 are C-RNTIs (TS 36.321 Table 7.1-1); stay clear of the RA-RNTIs, which are at most 0x100 + 0x100
  const uint16_t r = next_tc_rnti;
  next_tc_rnti     = next_tc_rnti >= 0xFFF0 ? 0x0200 : (uint16_t)(next_tc_rnti + 1);
  return r;
}

bool nbiot_mac::on_preamble(const nbiot_nprach_detection& d, nbiot_ra_response& resp, std::string& why)
{
  std::lock_guard<std::mutex> guard(lock);
  cnt.preambles++;
  if (!initiated) {
    why = "not initialised";
    return false;
  }

  const uint64_t now = srsran_nbiot_dl_sched_now(sched);
  srsran_nbiot_layout_t layout;
  if (now == 0 || !srsran_nbiot_dl_sched_get_layout(sched, &layout)) {
    cnt.no_layout++;
    why = "the downlink has not started";
    return false;
  }
  if (d.tti >= SRSRAN_NBIOT_SF_PER_HFN || d.n_init >= 64) {
    why = "detection out of range";
    return false;
  }

  // The report carries the TTI modulo 10240. The preamble ended before the subframe the composer is at, so the
  // absolute time is the latest one with that remainder not after 'now'.
  const uint64_t age   = (now % SRSRAN_NBIOT_SF_PER_HFN + SRSRAN_NBIOT_SF_PER_HFN - d.tti) % SRSRAN_NBIOT_SF_PER_HFN;
  if (age > now) {
    why = "detection from before the start";
    return false;
  }
  const uint64_t start = now - age;

  nbiot_ra_response r;
  r.preamble       = d.n_init;
  r.preamble_start = start;
  r.ra_rnti        = (uint16_t)srsran_nbiot_ra_rnti((uint32_t)((start / 10) % 1024), 0);
  r.ta             = srsran_nbiot_ta_from_toa(d.toa);
  r.grant          = cfg.msg3;

  uint32_t period = 0;
  srsran_nbiot_search_space_start(cfg.r_max, cfg.g_halves, cfg.offset_eighths, 0, &period);
  r.window_start = window_start(start, cfg.nprach_n_rep, cfg.nprach_format);
  r.window_end   = r.window_start + (uint64_t)cfg.window_pp * period - 1;

  // The message
  srsran_nbiot_rar_t rar = {};
  rar.rapid              = d.n_init;
  rar.ta                 = r.ta;
  rar.grant              = cfg.msg3;
  // tc_rnti is taken once a candidate has been found, so failed attempts do not use them up
  const uint16_t tc = next_tc_rnti;
  rar.tc_rnti       = tc;
  const int pdu_len = srsran_nbiot_rar_pdu_pack(-1, &rar, 1, r.pdu, sizeof(r.pdu));
  if (pdu_len != (int)sizeof(r.pdu) || pdu_len * 8 != (int)RAR_TBS) {
    why = "internal: random access response packing";
    return false;
  }
  r.pdu_len = (uint32_t)pdu_len;

  srsran_nbiot_dci_n1_t dci = {};
  dci.i_delay               = 0;
  dci.i_sf                  = RAR_I_SF;
  dci.i_mcs                 = RAR_I_TBS;
  dci.i_rep                 = cfg.rar_i_rep;
  dci.dci_rep               = (uint32_t)srsran_nbiot_dci_rep_for_rmax(cfg.r_max);
  uint8_t dci_bits[SRSRAN_NBIOT_DCI_LEN];
  if (srsran_nbiot_dci_n1_pack(&dci, dci_bits) != SRSRAN_SUCCESS) {
    why = "internal: DCI N1 packing";
    return false;
  }

  const uint32_t n_sf  = srsran_nbiot_npdsch_n_sf(RAR_I_SF);
  const uint32_t n_rep = srsran_nbiot_npdsch_n_rep(cfg.rar_i_rep);
  const int      k0d   = srsran_nbiot_npdsch_k0(dci.i_delay, cfg.r_max);
  if (n_sf == 0 || n_rep == 0 || k0d < 0) {
    why = "internal: NPDSCH parameters";
    return false;
  }

  // Earliest subframe that can still be planned, and the first search space start at or after it (and the window)
  const uint64_t min_t = now + cfg.lead_sf;
  uint64_t       t_min = std::max(r.window_start, min_t);

  srsran_nbiot_plan_t pc, pd;
  bool                found = false;
  while (true) {
    const uint64_t k0 = srsran_nbiot_search_space_start(cfg.r_max, cfg.g_halves, cfg.offset_eighths, t_min, &period);
    if (k0 > r.window_end) {
      break;
    }
    if (srsran_nbiot_plan_npdcch(&layout, k0, cfg.r_max, &pc) != SRSRAN_SUCCESS ||
        srsran_nbiot_plan_npdsch(&layout, pc.t[pc.nof_sf - 1], (uint32_t)k0d, n_sf, n_rep, &pd) != SRSRAN_SUCCESS) {
      why = "cannot plan the response";
      return false;
    }
    bool free_sf = true;
    for (uint32_t i = 0; i < pc.nof_sf && free_sf; i++) {
      free_sf = pc.t[i] >= min_t && !srsran_nbiot_dl_sched_busy(sched, pc.t[i]);
    }
    for (uint32_t i = 0; i < pd.nof_sf && free_sf; i++) {
      free_sf = !srsran_nbiot_dl_sched_busy(sched, pd.t[i]);
    }
    if (free_sf) {
      found = true;
      break;
    }
    t_min = k0 + 1;
  }
  if (!found) {
    cnt.late++;
    why = "no search space start left in the response window [" + std::to_string(r.window_start) + ", " +
          std::to_string(r.window_end) + "] (now " + std::to_string(now) + ")";
    return false;
  }

  uint8_t e[SRSRAN_NBIOT_DL_SCHED_MAX_E];
  if (srsran_nbiot_npdcch_encode(&dlch, dci_bits, SRSRAN_NBIOT_DCI_LEN, r.ra_rnti, e) != SRSRAN_SUCCESS ||
      srsran_nbiot_dl_sched_add_npdcch(sched, e, srsran_nbiot_dlch_bits_per_sf(&dlch), &pc, min_t) != SRSRAN_SUCCESS) {
    cnt.no_room++;
    why = "the downlink schedule refused the NPDCCH";
    return false;
  }
  if (srsran_nbiot_npdsch_encode(&dlch, r.pdu, RAR_TBS, n_sf, e) /* packed octets, msb first */ != SRSRAN_SUCCESS ||
      srsran_nbiot_dl_sched_add_npdsch(
          sched, e, n_sf * srsran_nbiot_dlch_bits_per_sf(&dlch), r.ra_rnti, n_sf, &pd, min_t) != SRSRAN_SUCCESS) {
    cnt.no_room++;
    why = "the downlink schedule refused the NPDSCH";
    return false;
  }

  r.tc_rnti       = alloc_tc_rnti();
  r.npdcch_start  = pc.t[0];
  r.npdcch_end    = pc.t[pc.nof_sf - 1];
  r.npdsch_start  = pd.t[0];
  r.npdsch_end    = pd.t[pd.nof_sf - 1];
  cnt.answered++;
  answered.push_back(r);
  if (answered.size() > MAX_PENDING) {
    answered.erase(answered.begin());
  }
  resp = r;
  return true;
}

void nbiot_mac::preamble_detected(const nbiot_nprach_detection& d)
{
  nbiot_ra_response r;
  std::string       why;
  if (on_preamble(d, r, why)) {
    srsran::console("NB-IoT: random access response for preamble %u: RA-RNTI %u, TC-RNTI 0x%04x, TA %u, "
                    "NPDCCH %llu..%llu, NPDSCH %llu..%llu (window %llu..%llu)\n",
                    r.preamble,
                    r.ra_rnti,
                    r.tc_rnti,
                    r.ta,
                    (unsigned long long)r.npdcch_start,
                    (unsigned long long)r.npdcch_end,
                    (unsigned long long)r.npdsch_start,
                    (unsigned long long)r.npdsch_end,
                    (unsigned long long)r.window_start,
                    (unsigned long long)r.window_end);
    logger.info("NB-IoT RAR: preamble %u RA-RNTI %u TC-RNTI 0x%04x TA %u NPDCCH %llu..%llu NPDSCH %llu..%llu",
                r.preamble,
                r.ra_rnti,
                r.tc_rnti,
                r.ta,
                (unsigned long long)r.npdcch_start,
                (unsigned long long)r.npdcch_end,
                (unsigned long long)r.npdsch_start,
                (unsigned long long)r.npdsch_end);
  } else {
    srsran::console("NB-IoT: preamble %u not answered: %s\n", d.n_init, why.c_str());
    logger.warning("NB-IoT RAR: preamble %u not answered: %s", d.n_init, why.c_str());
  }
}

std::vector<nbiot_ra_response> nbiot_mac::pending() const
{
  std::lock_guard<std::mutex> guard(lock);
  return answered;
}

nbiot_mac::counters nbiot_mac::stats() const
{
  std::lock_guard<std::mutex> guard(lock);
  return cnt;
}

} // namespace srsenb
