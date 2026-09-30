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

/**
 * The in-band NB-IoT anchor PRB belongs to NB-IoT in both directions: the NB-IoT downlink overwrites it and the NB-IoT
 * uplink (NPRACH, NPUSCH) is received on it. With cell_cfg_t::nbiot_anchor_prb set, no LTE allocation -- user data,
 * RAR, system information, paging, PUSCH -- may cover that PRB, however busy the cell is.
 *
 * The cell is run saturated (every UE has more data to send and receive than a subframe can carry) and every
 * allocation the scheduler returns is decoded to PRBs. A control run without the reservation must, under the same
 * load, use the anchor PRB: otherwise the test would pass for a scheduler that simply never reaches that PRB.
 */

#include "sched_test_common.h"
#include "srsenb/hdr/stack/mac/sched.h"
#include "srsran/common/common_lte.h"

namespace srsenb {

class anchor_tester : public sched_sim_base
{
  static std::vector<sched_interface::cell_cfg_t> get_cell_cfg(srsran::span<const sched_cell_params_t> cell_params)
  {
    std::vector<sched_interface::cell_cfg_t> cell_cfg_list;
    for (const auto& c : cell_params) {
      cell_cfg_list.push_back(c.cfg);
    }
    return cell_cfg_list;
  }

public:
  explicit anchor_tester(sched*                                          sched_obj_,
                         const sched_interface::sched_args_t&            sched_args,
                         const std::vector<sched_interface::cell_cfg_t>& cell_cfg_list) :
    sched_sim_base(sched_obj_, sched_args, cell_cfg_list),
    sched_ptr(sched_obj_),
    dl_result(cell_cfg_list.size()),
    ul_result(cell_cfg_list.size())
  {}

  sched*   sched_ptr;
  uint32_t anchor = 0;
  // bytes pending per UE and direction; a small value makes the scheduler shrink every grant to what is needed, which
  // is a different code path (and was a broken one next to a reserved RBG) from handing out everything that is free
  uint32_t load_bytes = 1000000;

  std::vector<sched_interface::dl_sched_res_t> dl_result;
  std::vector<sched_interface::ul_sched_res_t> ul_result;

  // what the scheduler handed out, per kind
  uint64_t dl_data_allocs = 0, ul_allocs = 0, other_dl_allocs = 0;
  uint64_t dl_on_anchor = 0, ul_on_anchor = 0, other_dl_on_anchor = 0;

  int advance_tti()
  {
    tti_point tti_rx = get_tti_rx().is_valid() ? get_tti_rx() + 1 : tti_point(0);
    new_tti(tti_rx);

    for (uint32_t cc = 0; cc < get_cell_params().size(); ++cc) {
      TESTASSERT(sched_ptr->dl_sched(to_tx_dl(tti_rx).to_uint(), cc, dl_result[cc]) == SRSRAN_SUCCESS);
      TESTASSERT(sched_ptr->ul_sched(to_tx_ul(tti_rx).to_uint(), cc, ul_result[cc]) == SRSRAN_SUCCESS);
    }

    sf_output_res_t sf_out{get_cell_params(), tti_rx, ul_result, dl_result};
    update(sf_out);
    count(sf_out);
    return SRSRAN_SUCCESS;
  }

  void set_external_tti_events(const sim_ue_ctxt_t& ue_ctxt, ue_tti_events& pending_events) override
  {
    if (ue_ctxt.conres_rx) {
      sched_ptr->ul_bsr(ue_ctxt.rnti, 1, load_bytes);
      sched_ptr->dl_rlc_buffer_state(ue_ctxt.rnti, 3, load_bytes, 0);
      if (get_tti_rx().to_uint() % 5 == 0) {
        for (auto& cc : pending_events.cc_list) {
          cc.dl_cqi = 15;
          cc.ul_snr = 40;
        }
      }
    }
  }

private:
  bool dl_covers_anchor(const srsran_dci_dl_t& dci)
  {
    const uint32_t         nof_prb = get_cell_params()[0].nof_prb();
    srsran_pdsch_grant_t grant   = {};
    srsran_ra_dl_grant_to_grant_prb_allocation(&dci, &grant, nof_prb);
    return grant.prb_idx[0][anchor] or grant.prb_idx[1][anchor];
  }

  void count(const sf_output_res_t& out)
  {
    const uint32_t nof_prb = get_cell_params()[0].nof_prb();
    for (const auto& d : out.dl_cc_result[0].data) {
      dl_data_allocs++;
      dl_on_anchor += dl_covers_anchor(d.dci);
    }
    for (const auto& r : out.dl_cc_result[0].rar) {
      other_dl_allocs++;
      other_dl_on_anchor += dl_covers_anchor(r.dci);
    }
    for (const auto& b : out.dl_cc_result[0].bc) {
      other_dl_allocs++;
      other_dl_on_anchor += dl_covers_anchor(b.dci);
    }
    for (const auto& p : out.ul_cc_result[0].pusch) {
      ul_allocs++;
      prb_interval prbs = prb_interval::riv_to_prbs(p.dci.type2_alloc.riv, nof_prb);
      ul_on_anchor += (anchor >= prbs.start() and anchor < prbs.stop());
    }
  }
};

struct result {
  uint64_t dl_data, ul, other_dl, dl_hit, ul_hit, other_hit;
};

/// Runs a saturated cell and reports what landed where
static int run(uint32_t nof_prb, uint32_t anchor, bool reserve, uint32_t load_bytes, result& res)
{
  sched_interface::cell_cfg_t cell_cfg = generate_default_cell_cfg(nof_prb);
  cell_cfg.nbiot_anchor_prb            = reserve ? (int)anchor : -1;
  std::vector<sched_interface::cell_cfg_t> cells(1, cell_cfg);
  sched_interface::ue_cfg_t                ue_cfg     = generate_default_ue_cfg();
  sched_interface::sched_args_t            sched_args = {};
  sched_args.sched_policy                             = "time_rr";

  sched     sched_obj;
  rrc_dummy rrc{};
  sched_obj.init(&rrc, sched_args);
  anchor_tester tester(&sched_obj, sched_args, cells);
  tester.anchor     = anchor;
  tester.load_bytes = load_bytes;

  for (uint32_t ue_idx = 0; ue_idx < 4; ++ue_idx) {
    uint16_t rnti = 0x46 + ue_idx;
    while (not srsran_prach_tti_opportunity_config_fdd(
        tester.get_cell_params()[0].cfg.prach_config, tester.get_tti_rx().to_uint(), -1)) {
      TESTASSERT(tester.advance_tti() == SRSRAN_SUCCESS);
    }
    TESTASSERT(tester.add_user(rnti, ue_cfg, 16) == SRSRAN_SUCCESS);
    TESTASSERT(tester.advance_tti() == SRSRAN_SUCCESS);
  }

  // From here on the UEs are connected and saturated; the RAR/SIB allocations of the start-up phase are included too
  for (uint32_t i = 0; i < (getenv("SCHED_TEST_TTIS") ? (uint32_t)atoi(getenv("SCHED_TEST_TTIS")) : 4000u); ++i) {
    TESTASSERT(tester.advance_tti() == SRSRAN_SUCCESS);
  }
  res = {tester.dl_data_allocs, tester.ul_allocs, tester.other_dl_allocs,
         tester.dl_on_anchor,   tester.ul_on_anchor, tester.other_dl_on_anchor};
  return SRSRAN_SUCCESS;
}

} // namespace srsenb

int main(int argc, char** argv)
{
  setvbuf(stdout, nullptr, _IOLBF, 0);
  for (const char* name : {"MAC", "TEST"}) {
    srslog::fetch_basic_logger(name).set_level(getenv("SCHED_TEST_LOG") ? (getenv("SCHED_TEST_LOG")[0] == 100 ? srslog::basic_levels::debug : getenv("SCHED_TEST_LOG")[0] == 105 ? srslog::basic_levels::info : srslog::basic_levels::warning)
                                                                        : srslog::basic_levels::none);
  }
  srslog::init();

  // debugging aid: nbiot_anchor_test <nof_prb> <anchor> runs only the run with the reservation
  if (argc >= 3) {
    srsenb::result r = {};
    TESTASSERT(srsenb::run(atoi(argv[1]), atoi(argv[2]), true, argc > 3 ? atoi(argv[3]) : 1000000, r) == SRSRAN_SUCCESS);
    printf("reserved DL data %llu (on anchor %llu), other DL %llu (%llu), UL %llu (%llu)\n",
           (unsigned long long)r.dl_data, (unsigned long long)r.dl_hit, (unsigned long long)r.other_dl,
           (unsigned long long)r.other_hit, (unsigned long long)r.ul, (unsigned long long)r.ul_hit);
    return 0;
  }

  // 25 PRB with the anchor on PRB 17 is the live cell; the others vary RBG size and whether the anchor shares an RBG
  // with a neighbour
  struct scenario {
    uint32_t nof_prb, anchor, load_bytes;
  } scenarios[] = {
      {25, 17, 1000000}, {25, 22, 1000000}, {25, 2, 1000000}, {50, 14, 1000000}, {100, 44, 1000000}, {15, 7, 1000000},
      // light and medium load: grants are shrunk to the pending data
      {25, 17, 60},      {25, 17, 400},     {25, 17, 1500},    {25, 17, 4000},    {25, 2, 60},        {25, 2, 1500},
      {50, 14, 400},     {100, 44, 1500},
  };

  for (const auto& s : scenarios) {
    srsenb::result control = {}, reserved = {};
    TESTASSERT(srsenb::run(s.nof_prb, s.anchor, false, s.load_bytes, control) == SRSRAN_SUCCESS);
    TESTASSERT(srsenb::run(s.nof_prb, s.anchor, true, s.load_bytes, reserved) == SRSRAN_SUCCESS);
    printf("%3u PRB, anchor %3u, %7u B pending: control DL %5llu (on anchor %4llu) UL %5llu (%4llu) | "
           "reserved DL %5llu (%llu) UL %5llu (%llu), other DL %llu (%llu)\n",
           s.nof_prb,
           s.anchor,
           s.load_bytes,
           (unsigned long long)control.dl_data,
           (unsigned long long)control.dl_hit,
           (unsigned long long)control.ul,
           (unsigned long long)control.ul_hit,
           (unsigned long long)reserved.dl_data,
           (unsigned long long)reserved.dl_hit,
           (unsigned long long)reserved.ul,
           (unsigned long long)reserved.ul_hit,
           (unsigned long long)reserved.other_dl,
           (unsigned long long)reserved.other_hit);

    // with the reservation: never on the anchor, in any allocation kind ...
    TESTASSERT(reserved.dl_hit == 0 and reserved.ul_hit == 0 and reserved.other_hit == 0);
    // ... while the cell carries (at least) as much traffic as without it: the reservation must not stall anybody.
    // A cell that stops scheduling would trivially never touch the anchor.
    TESTASSERT(reserved.dl_data * 10 >= control.dl_data * 9 and reserved.ul * 10 >= control.ul * 9);
    TESTASSERT(reserved.dl_data > 1000 and reserved.ul > 1000);
    // the saturated control runs must reach the anchor, or the test proves nothing about it
    if (s.load_bytes >= 1000000) {
      TESTASSERT(control.dl_hit > 100 and control.ul_hit > 100);
    }
  }
  printf("PASS\n");
  return 0;
}
