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

#include "srsenb/hdr/stack/mac/sched_emtc.h"
#include <cmath>
#include "srsran/support/srsran_assert.h"
#include "srsran/common/standard_streams.h"
#include "srsran/mac/pdu.h"
#include "srsran/phy/ue/ue_ul.h"
#include "srsran/common/string_helpers.h"
#include "srsran/srsran.h"

extern "C" {
#include "srsran/phy/phch/emtc.h"
#include "srsran/phy/phch/ra.h"
}

namespace srsenb {

sched_emtc::sched_emtc() : logger(srslog::fetch_basic_logger("MAC")) {}

sched_emtc::~sched_emtc()
{
  if (rar_sb_init) {
    for (auto& sb : rar_sb) {
      srsran_softbuffer_tx_free(&sb);
    }
  }
}

void sched_emtc::set_cell(std::shared_ptr<const emtc::config> cfg_,
                          std::shared_ptr<const emtc::bcast>  bcast,
                          const cell_cfg_t&                   cell_cfg)
{
  std::lock_guard<std::mutex> lock(mutex);
  cfg               = std::move(cfg_);
  bc                = std::move(bcast);
  cell              = cell_cfg.cell;
  pucch_delta_shift = std::max<uint32_t>(cell_cfg.delta_pucch_shift, 1);
  pucch_nrb_cqi     = cell_cfg.nrb_cqi;
  pucch_ncs_an      = cell_cfg.ncs_an;
  if (cfg && !rar_sb_init) {
    for (auto& sb : rar_sb) {
      srsran_assert(srsran_softbuffer_tx_init(&sb, cell.nof_prb) == SRSRAN_SUCCESS, "LTE-M: RAR softbuffer");
    }
    rar_sb_init = true;
  }
}

void sched_emtc::set_pcch_source(pcch_source src)
{
  std::lock_guard<std::mutex> lock(mutex);
  pcch_src = std::move(src);
}

void sched_emtc::plan_paging(uint32_t po)
{
  if (!pcch_src) {
    return;
  }
  // PNB = floor(UE_ID / (N * Ns)) mod Nn (TS 36.304 7.1): the PCCH of a PO goes on every paging narrowband. The
  // MPDCCH of paging narrowband n is on s_((n + PCI) mod N_S) of the SIB1-BR set, its PDSCH where the DCI points: there
  uint8_t*                buf = rar_buf[rar_buf_idx].data();
  srsran_softbuffer_tx_t* sb  = &rar_sb[rar_buf_idx];
  memset(buf, 0, rar_buf[0].size());
  const uint32_t len = pcch_src(po, buf, 26); // the largest 6-2 TBS: 208 bits
  if (len == 0) {
    return;
  }
  const uint32_t pd  = tti_add(po, 2);
  uint32_t       nbs = 0;
  for (uint32_t pnb = 0; pnb < cfg->paging_nbs; pnb++) {
    const uint32_t nb = srsran_emtc_paging_mpdcch_nb(cell.nof_prb, cell.id, pnb);
    if (nb_free_dl(po, nb) && nb_free_dl(pd, nb)) {
      nbs |= 1U << nb;
    } else {
      srsran::console("LTE-M: paging narrowband %d busy at PO %d\n", nb, po);
    }
  }
  if (nbs == 0) {
    return;
  }
  rar_buf_idx = (rar_buf_idx + 1) % NOF_RAR_BUF;
  uint32_t i_tbs = 0;
  while (srsran_emtc_tbs_1c(i_tbs) < len * 8) {
    i_tbs++;
  }
  for (uint32_t nb = 0; nb < 32; nb++) {
    if ((nbs & (1U << nb)) == 0) {
      continue;
    }
    mac_interface_phy_lte::emtc_mpdcch_t mp = {};
    mp.rnti                                 = SRSRAN_PRNTI;
    mp.nb                                   = nb;
    mp.common                               = true;
    mp.nof_bits = srsran_emtc_dci_6_2_pack(cell.nof_prb, nb, i_tbs, 0, 0, mp.dci);
    dl[po].mpdcch.push_back(mp);
    dl[po].nbs |= 1U << nb;

    pdsch_plan pp     = {};
    pp.rar            = true;
    pp.phy.rnti       = SRSRAN_PRNTI;
    pp.phy.nb         = nb;
    pp.phy.rb_start   = 0;
    pp.phy.nof_rb     = 6;
    pp.phy.tbs        = srsran_emtc_tbs_1c(i_tbs);
    pp.phy.mod        = SRSRAN_MOD_QPSK;
    pp.phy.rv         = 0;
    pp.phy.data       = buf;
    pp.phy.softbuffer = sb;
    dl[pd].pdsch.push_back(pp);
    dl[pd].nbs |= 1U << nb;
    srsran::console("LTE-M: paging %d B (TBS %d) at PO %d on narrowband %d, PDSCH %d\n",
                    len,
                    srsran_emtc_tbs_1c(i_tbs),
                    po,
                    nb,
                    pd);
  }
}

bool sched_emtc::is_br_preamble(uint32_t preamble) const
{
  return cfg && preamble >= cfg->first_preamble && preamble <= cfg->last_preamble;
}

void sched_emtc::add_rnti(uint16_t rnti)
{
  std::lock_guard<std::mutex> lock(mutex);
  ues[rnti] = {};
}

bool sched_emtc::has_rnti(uint16_t rnti)
{
  std::lock_guard<std::mutex> lock(mutex);
  return ues.count(rnti) > 0;
}

uint16_t sched_emtc::ra_rnti(uint32_t prach_tti) const
{
  return srsran_emtc_ra_rnti((prach_tti / 10) % 1024, prach_tti % 10, 0);
}

bool sched_emtc::nb_free_dl(uint32_t tti, uint32_t nb)
{
  if (srsran_emtc_bcast_nbs(&bc->sched, (tti / 10) % 1024, tti % 10) & (1U << nb)) {
    return false;
  }
  auto it = dl.find(tti);
  return it == dl.end() || (it->second.nbs & (1U << nb)) == 0;
}

uint64_t sched_emtc::dl_prbs(uint32_t tti_tx_dl)
{
  std::lock_guard<std::mutex> lock(mutex);
  if (!cfg) {
    return 0;
  }
  uint64_t prbs = srsran_emtc_bcast_prbs(&bc->sched, (tti_tx_dl / 10) % 1024, tti_tx_dl % 10);
  auto     it   = dl.find(tti_tx_dl);
  if (it != dl.end()) {
    for (uint32_t nb = 0; nb < srsran_emtc_nof_nb(cell.nof_prb); nb++) {
      if (it->second.nbs & (1U << nb)) {
        prbs |= srsran_emtc_nb_mask(cell.nof_prb, nb);
      }
    }
  }
  return prbs;
}

uint64_t sched_emtc::ul_prbs(uint32_t tti_tx_ul)
{
  std::lock_guard<std::mutex> lock(mutex);
  auto                        it = ul_resv.find(tti_tx_ul);
  return it == ul_resv.end() ? 0 : it->second;
}

bool sched_emtc::plan_rar(const std::vector<dl_sched_rar_info_t>& ras)
{
  if (ras.empty() || cfg->mpdcch_nb_ra.empty() || !last_dl_valid) {
    return false;
  }
  const uint32_t p  = ras[0].prach_tti;
  const uint32_t nb = cfg->mpdcch_nb_ra[ras[0].preamble_idx % 2 % cfg->mpdcch_nb_ra.size()];

  // TBS column N_PRB^1A = 3 (TPC LSB set), QPSK: the smallest I_TBS that holds every RAR
  const uint32_t n_rar  = (uint32_t)ras.size();
  uint32_t       i_tbs  = 0;
  int            tbs    = 0;
  for (; i_tbs <= 9; i_tbs++) {
    tbs = srsran_ra_tbs_from_idx(i_tbs, 3);
    if (tbs >= (int)(56 * n_rar)) {
      break;
    }
  }
  if (i_tbs > 9 || tbs / 8 > (int)rar_buf[0].size()) {
    logger.warning("LTE-M: %d RARs do not fit one RAR PDSCH", n_rar);
    return false;
  }

  // Msg3 PRBs in the UL narrowband
  const int first_ul = srsran_emtc_nb_first_prb(cell.nof_prb, cfg->msg3_nb);
  if (first_ul < 0) {
    return false;
  }
  const uint32_t msg3_prb0 = (uint32_t)first_ul + cfg->msg3_rb_start;
  uint64_t       msg3_mask = 0;
  for (uint32_t n = 0; n < cfg->msg3_nof_rb; n++) {
    msg3_mask |= 1ULL << (msg3_prb0 + n);
  }

  // RAR window: from 3 subframes after the preamble, ra-ResponseWindowSize long; MPDCCH in even subframes, not
  // before the TTIs the LTE scheduler may already have handled
  const int first = std::max(3, tti_diff(last_dl_tti, p) + 3);
  for (int d = first; d < 3 + (int)cfg->ra_response_window_sf; d++) {
    const uint32_t m = tti_add(p, d);
    if (m % 2 != 0) {
      continue;
    }
    const uint32_t pd = tti_add(m, 2);
    const uint32_t u  = tti_add(pd, 6);
    if (!nb_free_dl(m, nb) || !nb_free_dl(pd, nb) || (ul_resv[u] & msg3_mask)) {
      continue;
    }

    // RAR MAC PDU: E/T/RAPID subheaders, then R/TA/UL grant/TC-RNTI (TS 36.321 6.1.5, 6.2.3)
    uint8_t*                buf = rar_buf[rar_buf_idx].data();
    srsran_softbuffer_tx_t* sb  = &rar_sb[rar_buf_idx];
    rar_buf_idx                 = (rar_buf_idx + 1) % NOF_RAR_BUF;
    memset(buf, 0, rar_buf[0].size());
    uint8_t* w     = buf;
    uint32_t grant = srsran_emtc_rar_grant_ce_a(cell.nof_prb,
                                                cfg->msg3_nb,
                                                srsran_emtc_riv(cfg->msg3_rb_start, cfg->msg3_nof_rb),
                                                0,
                                                cfg->msg3_mcs,
                                                3,
                                                false,
                                                false,
                                                0);
    for (uint32_t i = 0; i < n_rar; i++) {
      *w++ = (uint8_t)(((i + 1 < n_rar) ? 0x80 : 0) | 0x40 | (ras[i].preamble_idx & 0x3f));
    }
    for (uint32_t i = 0; i < n_rar; i++) {
      uint32_t ta = std::min<uint32_t>(ras[i].ta_cmd, 2047);
      *w++        = (uint8_t)((ta >> 4) & 0x7f);
      *w++        = (uint8_t)(((ta & 0xf) << 4) | ((grant >> 16) & 0xf));
      *w++        = (uint8_t)((grant >> 8) & 0xff);
      *w++        = (uint8_t)(grant & 0xff);
      *w++        = (uint8_t)(ras[i].temp_crnti >> 8);
      *w++        = (uint8_t)(ras[i].temp_crnti & 0xff);
    }

    const uint16_t rarnti = ra_rnti(p);
    for (const auto& ra : ras) {
      auto ue = ues.find(ra.temp_crnti);
      if (ue != ues.end()) {
        ue->second.nb = nb; // Msg3/4 MPDCCH narrowband index 0 in the grant: NB_RAR (TS 36.213 Table 6.2-B)
      }
    }

    // MPDCCH: DCI 6-1A on the Type2-CSS, CRC scrambled by the RA-RNTI
    mac_interface_phy_lte::emtc_mpdcch_t mp = {};
    mp.rnti                                 = rarnti;
    mp.nb                                   = nb;
    mp.common                               = true;
    srsran_emtc_dci_t dci                   = {};
    dci.nb                                  = nb;
    dci.riv                                 = srsran_emtc_riv(0, 6);
    dci.mcs                                 = i_tbs;
    dci.tpc                                 = 1; // LSB: N_PRB^1A = 3
    mp.nof_bits = srsran_emtc_dci_6_1a_pack(cell.nof_prb, cfg->dci_srs_6_1a, &dci, mp.dci);
    dl[m].mpdcch.push_back(mp);
    dl[m].nbs |= 1U << nb;

    pdsch_plan pp     = {};
    pp.rar            = true;
    pp.phy.rnti       = rarnti;
    pp.phy.nb         = nb;
    pp.phy.rb_start   = 0;
    pp.phy.nof_rb     = 6;
    pp.phy.tbs        = (uint32_t)tbs;
    pp.phy.mod        = SRSRAN_MOD_QPSK;
    pp.phy.rv         = 0;
    pp.phy.data       = buf;
    pp.phy.softbuffer = sb;
    dl[pd].pdsch.push_back(pp);
    dl[pd].nbs |= 1U << nb;

    // Msg3: PUSCH six subframes after the RAR PDSCH, no PDCCH
    int      i_tbs_ul = (int)cfg->msg3_mcs; // I_MCS 0..10: I_TBS = I_MCS, QPSK
    ul_grant g        = {};
    g.dci.rnti        = ras[0].temp_crnti;
    g.dci.format      = SRSRAN_DCI_FORMAT0;
    g.dci.type2_alloc.riv = srsran_ra_type2_to_riv(cfg->msg3_nof_rb, msg3_prb0, cell.nof_prb);
    g.dci.freq_hop_fl     = srsran_dci_ul_t::SRSRAN_RA_PUSCH_HOP_DISABLED;
    g.dci.tb.mcs_idx      = cfg->msg3_mcs;
    g.dci.tb.rv           = 0;
    g.tbs_bytes           = (uint32_t)srsran_ra_tbs_from_idx((uint32_t)i_tbs_ul, cfg->msg3_nof_rb) / 8;
    g.pid                 = u % SRSRAN_FDD_NOF_HARQ;
    for (uint32_t i = 0; i < n_rar; i++) {
      g.rnti     = ras[i].temp_crnti;
      g.dci.rnti = ras[i].temp_crnti;
      ul[u].push_back(g);
    }
    ul_resv[u] |= msg3_mask;

    for (const auto& ra : ras) {
      srsran::console("LTE-M RACH: preamble %d, TA %d, temp C-RNTI 0x%x: RAR (RA-RNTI 0x%x) MPDCCH TTI %d, PDSCH %d, "
                      "Msg3 %d on PRB %d-%d\n",
                      ra.preamble_idx,
                      ra.ta_cmd,
                      ra.temp_crnti,
                      rarnti,
                      m,
                      pd,
                      u,
                      msg3_prb0,
                      msg3_prb0 + cfg->msg3_nof_rb - 1);
    }
    return true;
  }
  srsran::console("LTE-M RACH: no room for the RAR of preamble %d (PRACH TTI %d)\n", ras[0].preamble_idx, p);
  return false;
}

uint32_t sched_emtc::pucch_prb(uint32_t n_pucch, uint32_t tti) const
{
  // PUCCH format 1/1a of a BL/CE UE (TS 36.211 5.4.3): one PRB for the subframe, m' = m +/- 1 in odd hopping intervals
  const uint32_t c    = 3;
  const uint32_t ncs  = pucch_ncs_an;
  uint32_t       m    = pucch_nrb_cqi;
  if (n_pucch >= c * ncs / pucch_delta_shift) {
    m = (n_pucch - c * ncs / pucch_delta_shift) / (c * SRSRAN_NRE / pucch_delta_shift) + pucch_nrb_cqi + (ncs + 7) / 8;
  }
  const uint32_t hop = std::max<uint32_t>(cfg->ul_hop_interval, 1);
  if ((tti / hop) % 2 == 1) {
    m = (m % 2 == 0) ? m + 1 : m - 1;
  }
  return (m % 2 == 0) ? m / 2 : cell.nof_prb - 1 - m / 2;
}

bool sched_emtc::ue_uci_at(const ue_ctxt& ue, uint32_t tti)
{
  return srsran_ue_ul_sr_send_tti(&ue.pucch, tti) > 0 || srsran_cqi_periodic_send(&ue.cqi, tti, SRSRAN_FDD) ||
         srsran_cqi_periodic_ri_send(&ue.cqi, tti, SRSRAN_FDD);
}

bool sched_emtc::ue_tx_at(const ue_ctxt& ue, uint32_t tti)
{
  return ue.tx_busy[tti % NOF_TTI] || ue_uci_at(ue, tti);
}

bool sched_emtc::ue_can_rx(const ue_ctxt& ue, uint32_t tti)
{
  return !ue_tx_at(ue, tti_add(tti, -1)) && !ue_tx_at(ue, tti) && !ue_tx_at(ue, tti_add(tti, 1));
}

bool sched_emtc::ue_can_tx(const ue_ctxt& ue, uint32_t tti)
{
  return !ue.rx_busy[tti_add(tti, -1)] && !ue.rx_busy[tti] && !ue.rx_busy[tti_add(tti, 1)];
}

bool sched_emtc::plan_ue_dl(uint16_t rnti, ue_ctxt& ue, uint32_t tti_tx_dl)
{
  static const uint32_t rv_seq[4] = {0, 2, 3, 1};
  static const uint32_t max_tx    = 4;
  dl_harq&              h         = ue.dlh;

  if (h.active && h.waiting) {
    if (tti_diff(tti_tx_dl, h.ack_tti) <= 4) {
      return false;
    }
    h.waiting = false; // no HARQ-ACK reported: DTX
    srsran::console("LTE-M: 0x%x no HARQ-ACK for the PDSCH of TTI %d\n", rnti, tti_add(h.ack_tti, -4));
  }
  if (h.active && h.nof_tx >= max_tx) {
    srsran::console("LTE-M: 0x%x DL HARQ %d dropped after %d transmissions\n", rnti, h.pid, h.nof_tx);
    h.active = false;
    return false;
  }

  // New transmission: what is pending, in the order the MAC PDU is built
  ue_pdsch up = {};
  uint32_t need = 0;
  if (!h.active) {
    if (ue.conres && ue.dl_bytes[0] == 0) {
      return false; // Msg4 carries the RRC answer on SRB0 with the Contention Resolution CE
    }
    need += ue.conres ? 7 : 0;
    for (uint32_t lcid = 0; lcid < ue.dl_bytes.size(); lcid++) {
      need += ue.dl_bytes[lcid] > 0 ? ue.dl_bytes[lcid] + (lcid == 0 ? 2 : 3) : 0;
    }
    if (need == 0) {
      return false;
    }
  }

  // Type2-CSS start (even subframe, T = 2), PDSCH two subframes later, HARQ-ACK four after that
  const uint32_t n1 = cfg->n1_pucch_an; // n_ECCE = 0 (L' = 24), HARQ-ACK resource offset 0
  for (int d = 2; d < 40; d++) {
    const uint32_t m = tti_add(tti_tx_dl, d);
    if (m % 2 != 0) {
      continue;
    }
    const uint32_t pd  = tti_add(m, 2);
    const uint32_t a   = tti_add(pd, 4);
    const uint32_t prb = pucch_prb(n1, a);
    if (!nb_free_dl(m, ue.nb) || !nb_free_dl(pd, ue.nb) || (ul_resv[a] & (1ULL << prb))) {
      continue;
    }
    if (!ue_can_rx(ue, m) || !ue_can_rx(ue, pd) || !ue_can_tx(ue, a) || ue_uci_at(ue, a)) {
      continue;
    }

    if (!h.active) {
      uint32_t i_tbs = 0;
      for (; i_tbs < 9; i_tbs++) {
        if (srsran_ra_tbs_from_idx(i_tbs, 6) >= (int)(need * 8)) {
          break;
        }
      }
      h.i_tbs         = i_tbs;
      h.tbs           = (uint32_t)srsran_ra_tbs_from_idx(i_tbs, 6);
      h.pid           = (h.pid + 1) % SRSRAN_FDD_NOF_HARQ;
      h.ndi[h.pid]    = !h.ndi[h.pid];
      h.nof_tx        = 0;
      h.active        = true;
      uint32_t left   = h.tbs / 8;
      up.tbs_bytes    = left;
      if (ue.conres && left >= 7) {
        up.pdu.push_back({(uint32_t)srsran::dl_sch_lcid::CON_RES_ID, 6});
        left -= 7;
        ue.conres = false;
      }
      for (uint32_t lcid = 0; lcid < ue.dl_bytes.size(); lcid++) {
        uint32_t hdr = lcid == 0 ? 2 : 3;
        if (ue.dl_bytes[lcid] == 0 || left <= hdr || (lcid == 0 && ue.dl_bytes[lcid] + hdr > left)) {
          continue;
        }
        uint32_t n = std::min(ue.dl_bytes[lcid], left - hdr);
        up.pdu.push_back({lcid, n});
        ue.dl_bytes[lcid] -= n;
        left -= n + hdr;
      }
    }
    const uint32_t rv = rv_seq[h.nof_tx % 4];
    h.nof_tx++;
    h.waiting = true;
    h.ack_tti = a;

    mac_interface_phy_lte::emtc_mpdcch_t mp = {};
    mp.rnti                                 = rnti;
    mp.nb                                   = ue.nb;
    mp.common                               = true; // Type2-CSS until an MPDCCH UE-specific search space is configured
    srsran_emtc_dci_t dci                   = {};
    dci.nb                                  = ue.nb;
    dci.riv                                 = srsran_emtc_riv(0, 6);
    dci.mcs                                 = h.i_tbs;
    dci.harq_pid                            = h.pid;
    dci.ndi                                 = h.ndi[h.pid];
    dci.rv                                  = rv;
    dci.tpc                                 = 1; // 0 dB
    mp.nof_bits = srsran_emtc_dci_6_1a_pack(cell.nof_prb, cfg->dci_srs_6_1a, &dci, mp.dci);
    dl[m].mpdcch.push_back(mp);
    dl[m].nbs |= 1U << ue.nb;

    pdsch_plan pp     = {};
    pp.phy.rnti       = rnti;
    pp.phy.nb         = ue.nb;
    pp.phy.rb_start   = 0;
    pp.phy.nof_rb     = 6;
    pp.phy.tbs        = h.tbs;
    pp.phy.mod        = SRSRAN_MOD_QPSK;
    pp.phy.rv         = rv;
    pp.phy.harq_ack   = true;
    pp.phy.n_pucch    = n1;
    pp.phy.pid        = h.pid;
    up.rnti           = rnti;
    up.pid            = h.pid;
    up.tbs_bytes      = h.tbs / 8;
    pp.ue             = up;
    dl[pd].pdsch.push_back(pp);
    dl[pd].nbs |= 1U << ue.nb;
    ul_resv[a] |= 1ULL << prb;
    ue.rx_busy.set(m);
    ue.rx_busy.set(pd);
    ue.tx_busy.set(a);

    std::string what;
    for (const auto& e : up.pdu) {
      what += e.lcid == (uint32_t)srsran::dl_sch_lcid::CON_RES_ID ? std::string(" ConRes")
                                                                  : " LCID" + std::to_string(e.lcid) + ":" + std::to_string(e.nbytes);
    }
    srsran::console("LTE-M DL: 0x%x HARQ %d tx %d (rv %d, TBS %d)%s: MPDCCH %d, PDSCH %d, HARQ-ACK %d on PRB %d\n",
                    rnti,
                    h.pid,
                    h.nof_tx,
                    rv,
                    h.tbs,
                    h.nof_tx == 1 ? what.c_str() : " retx",
                    m,
                    pd,
                    a,
                    prb);
    return true;
  }
  return false;
}

/// Closed-loop PUSCH power control (accumulated TPC, TS 36.213 Table 5.1.1.1-2: 0 -1 dB, 1 0 dB, 2 +1 dB, 3 +3 dB).
/// At most one step per 40 ms, so the SNR average has seen the last one before the next.
uint32_t sched_emtc::ul_tpc(ue_ctxt& ue, uint32_t tti)
{
  static const float target_snr = 13.0f;
  if (ue.ul_snr < -100 || tti_diff(tti, ue.tpc_tti) < 40) {
    return 1;
  }
  uint32_t tpc = 1;
  if (ue.ul_snr < target_snr - 3) {
    tpc = 3;
  } else if (ue.ul_snr < target_snr - 1) {
    tpc = 2;
  } else if (ue.ul_snr > target_snr + 3) {
    tpc = 0;
  }
  if (tpc != 1) {
    ue.tpc_tti = tti;
  }
  return tpc;
}

bool sched_emtc::plan_ue_ul(uint16_t rnti, ue_ctxt& ue, uint32_t tti_tx_dl)
{
  static const uint32_t rv_seq[4] = {0, 2, 3, 1};
  static const uint32_t max_tx    = 4;
  static const uint32_t nof_rb    = 4; // PRBs 0-3 of the PUSCH narrowband (NB 2: clear of the NB-IoT UL at PRB 17)
  ul_harq&              h         = ue.ulh;

  if (!ue.connected) {
    return false;
  }
  if (h.active && h.waiting) {
    if (tti_diff(tti_tx_dl, h.pusch_tti) <= 4) {
      return false;
    }
    h.waiting = false; // no CRC reported
  }
  if (h.active && h.nof_tx >= max_tx) {
    srsran::console("LTE-M: 0x%x UL HARQ %d dropped after %d transmissions\n", rnti, h.pid, h.nof_tx);
    h.active = false;
  }
  uint32_t pending = 0;
  for (uint32_t b : ue.ul_lcg) {
    pending += b;
  }
  if (!h.active && !ue.sr && pending == 0) {
    return false;
  }

  const int first_ul = srsran_emtc_nb_first_prb(cell.nof_prb, cfg->msg3_nb);
  if (first_ul < 0) {
    return false;
  }
  uint64_t mask = 0;
  for (uint32_t n = 0; n < nof_rb; n++) {
    mask |= 1ULL << ((uint32_t)first_ul + n);
  }

  // MPDCCH in a Type2-CSS start subframe, PUSCH four subframes after it (TS 36.213 8.0)
  for (int d = 2; d < 40; d++) {
    const uint32_t m = tti_add(tti_tx_dl, d);
    if (m % 2 != 0) {
      continue;
    }
    const uint32_t u = tti_add(m, 4);
    if (!nb_free_dl(m, ue.nb) || (ul_resv[u] & mask)) {
      continue;
    }
    if (!ue_can_rx(ue, m) || !ue_can_tx(ue, u) || ue_uci_at(ue, u)) {
      continue;
    }
    if (!h.active) {
      uint32_t i_mcs = ue.ul_mcs_max; // on a bare SR, room for an RRC message when the link allows MCS 9
      if (pending > 0) {
        for (i_mcs = 0; i_mcs < ue.ul_mcs_max; i_mcs++) {
          if (srsran_ra_tbs_from_idx(i_mcs, nof_rb) >= (int)((pending + 4) * 8)) {
            break;
          }
        }
      }
      h.i_mcs  = i_mcs;
      h.tbs    = (uint32_t)srsran_ra_tbs_from_idx(i_mcs, nof_rb);
      h.pid    = (h.pid + 1) % SRSRAN_FDD_NOF_HARQ;
      h.ndi[h.pid] = !h.ndi[h.pid];
      h.nof_tx = 0;
      h.active = true;
      ue.sr    = false;
      uint32_t granted = h.tbs / 8;
      for (auto& b : ue.ul_lcg) {
        uint32_t n = std::min(b, granted);
        b -= n;
        granted -= n;
      }
    }
    const uint32_t rv = rv_seq[h.nof_tx % 4];
    h.nof_tx++;
    h.waiting   = true;
    h.pusch_tti = u;

    mac_interface_phy_lte::emtc_mpdcch_t mp = {};
    mp.rnti                                 = rnti;
    mp.nb                                   = ue.nb;
    mp.common                               = true;
    srsran_emtc_dci_t dci                   = {};
    dci.nb                                  = cfg->msg3_nb;
    dci.riv                                 = srsran_emtc_riv(0, nof_rb);
    dci.mcs                                 = h.i_mcs;
    dci.harq_pid                            = h.pid;
    dci.ndi                                 = h.ndi[h.pid];
    dci.rv                                  = rv;
    dci.tpc                                 = ul_tpc(ue, u);
    mp.nof_bits = srsran_emtc_dci_6_0a_pack(cell.nof_prb, cfg->dci_srs_6_1a, &dci, mp.dci);
    dl[m].mpdcch.push_back(mp);
    dl[m].nbs |= 1U << ue.nb;

    ul_grant g            = {};
    g.rnti                = rnti;
    g.dci.rnti            = rnti;
    g.dci.format          = SRSRAN_DCI_FORMAT0;
    g.dci.type2_alloc.riv = srsran_ra_type2_to_riv(nof_rb, (uint32_t)first_ul, cell.nof_prb);
    g.dci.freq_hop_fl     = srsran_dci_ul_t::SRSRAN_RA_PUSCH_HOP_DISABLED;
    g.dci.tb.mcs_idx      = h.i_mcs;
    g.dci.tb.rv           = rv;
    g.dci.tb.ndi          = h.ndi[h.pid];
    g.tbs_bytes           = h.tbs / 8;
    g.pid                 = h.pid;
    g.current_tx_nb       = h.nof_tx - 1;
    ul[u].push_back(g);
    ul_resv[u] |= mask;
    ue.rx_busy.set(m);
    ue.tx_busy.set(u);

    srsran::console("LTE-M UL: 0x%x HARQ %d tx %d (rv %d, MCS %d, TBS %d, TPC %d, SNR %.1f dB, %d B pending): MPDCCH %d, "
                    "PUSCH %d on PRB %d-%d\n",
                    rnti,
                    h.pid,
                    h.nof_tx,
                    rv,
                    h.i_mcs,
                    h.tbs,
                    dci.tpc,
                    ue.ul_snr,
                    pending,
                    m,
                    u,
                    first_ul,
                    first_ul + (int)nof_rb - 1);
    return true;
  }
  return false;
}

void sched_emtc::plan_dl(uint32_t tti_tx_dl)
{
  plan_paging(tti_add(tti_tx_dl, 2));
  const uint32_t stale = tti_add(tti_tx_dl, -8);
  for (auto& it : ues) {
    it.second.rx_busy.reset(stale);
    it.second.tx_busy.reset(stale);
    plan_ue_dl(it.first, it.second, tti_tx_dl);
    plan_ue_ul(it.first, it.second, tti_tx_dl);
  }
}

void sched_emtc::gc(uint32_t tti_tx_dl)
{
  for (auto it = dl.begin(); it != dl.end();) {
    it = tti_diff(it->first, tti_tx_dl) < 0 ? dl.erase(it) : std::next(it);
  }
  for (auto it = ul.begin(); it != ul.end();) {
    it = tti_diff(it->first, tti_tx_dl) < 0 ? ul.erase(it) : std::next(it);
  }
  for (auto it = ul_resv.begin(); it != ul_resv.end();) {
    it = tti_diff(it->first, tti_tx_dl) < 0 ? ul_resv.erase(it) : std::next(it);
  }
}

void sched_emtc::get_dl(uint32_t tti_tx_dl, mac_interface_phy_lte::dl_sched_t& res, std::vector<ue_pdsch>& ue_pdus)
{
  std::lock_guard<std::mutex> lock(mutex);
  res.nof_emtc_mpdcch = 0;
  res.nof_emtc_pdsch  = 0;
  if (!cfg) {
    return;
  }
  if (!last_dl_valid || tti_diff(tti_tx_dl, last_dl_tti) > 0) {
    last_dl_tti   = tti_tx_dl;
    last_dl_valid = true;
  }
  auto it = dl.find(tti_tx_dl);
  if (it != dl.end()) {
    for (const auto& m : it->second.mpdcch) {
      if (res.nof_emtc_mpdcch < mac_interface_phy_lte::EMTC_MAX_GRANTS) {
        res.emtc_mpdcch[res.nof_emtc_mpdcch++] = m;
      }
    }
    for (auto& p : it->second.pdsch) {
      if (res.nof_emtc_pdsch >= mac_interface_phy_lte::EMTC_MAX_GRANTS) {
        break;
      }
      if (p.rar) {
        srsran_softbuffer_tx_reset(p.phy.softbuffer);
      } else {
        p.ue.idx = res.nof_emtc_pdsch;
        ue_pdus.push_back(p.ue);
      }
      res.emtc_pdsch[res.nof_emtc_pdsch++] = p.phy;
    }
    dl.erase(it);
  }
  gc(tti_tx_dl);
  plan_dl(tti_tx_dl);
}

void sched_emtc::get_ul(uint32_t tti_tx_ul, std::vector<ul_grant>& grants)
{
  std::lock_guard<std::mutex> lock(mutex);
  auto                        it = ul.find(tti_tx_ul);
  if (it != ul.end()) {
    grants.insert(grants.end(), it->second.begin(), it->second.end());
    ul.erase(it);
  }
}

// ---- sched_interface

int sched_emtc::cell_cfg(const std::vector<cell_cfg_t>& cell_cfg)
{
  return SRSRAN_SUCCESS;
}

int sched_emtc::reset()
{
  std::lock_guard<std::mutex> lock(mutex);
  ues.clear();
  dl.clear();
  ul.clear();
  ul_resv.clear();
  return SRSRAN_SUCCESS;
}

int sched_emtc::ue_cfg(uint16_t rnti, const ue_cfg_t& cfg_)
{
  std::lock_guard<std::mutex> lock(mutex);
  ue_ctxt& ue  = ues[rnti];
  ue.configured = true;
  ue.pucch      = cfg_.pucch_cfg;
  ue.cqi        = cfg_.supported_cc_list.empty() ? srsran_cqi_report_cfg_t{}
                                                 : cfg_.supported_cc_list[0].dl_cfg.cqi_report;
  if (ue.pucch.sr_configured) {
    srsran::console("LTE-M: 0x%x SR I_sr=%d, CQI %s (pmi_idx %d)\n",
                    rnti,
                    ue.pucch.I_sr,
                    ue.cqi.periodic_configured ? "periodic" : "off",
                    ue.cqi.pmi_idx);
  }
  return SRSRAN_SUCCESS;
}

int sched_emtc::ue_rem(uint16_t rnti)
{
  std::lock_guard<std::mutex> lock(mutex);
  ues.erase(rnti);
  return SRSRAN_SUCCESS;
}

bool sched_emtc::ue_exists(uint16_t rnti)
{
  return has_rnti(rnti);
}

int sched_emtc::bearer_ue_cfg(uint16_t rnti, uint32_t lc_id, const mac_lc_ch_cfg_t& cfg_)
{
  return SRSRAN_SUCCESS;
}

int sched_emtc::bearer_ue_rem(uint16_t rnti, uint32_t lc_id)
{
  return SRSRAN_SUCCESS;
}

uint32_t sched_emtc::get_ul_buffer(uint16_t rnti)
{
  return 0;
}

uint32_t sched_emtc::get_dl_buffer(uint16_t rnti)
{
  std::lock_guard<std::mutex> lock(mutex);
  auto                        it = ues.find(rnti);
  uint32_t                    n  = 0;
  if (it != ues.end()) {
    for (uint32_t b : it->second.dl_bytes) {
      n += b;
    }
  }
  return n;
}

int sched_emtc::dl_rlc_buffer_state(uint16_t rnti, uint32_t lc_id, uint32_t tx_queue, uint32_t prio_tx_queue)
{
  std::lock_guard<std::mutex> lock(mutex);
  auto                        it = ues.find(rnti);
  if (it == ues.end() || lc_id >= it->second.dl_bytes.size()) {
    return SRSRAN_ERROR;
  }
  it->second.dl_bytes[lc_id] = tx_queue + prio_tx_queue;
  return SRSRAN_SUCCESS;
}

int sched_emtc::dl_mac_buffer_state(uint16_t rnti, uint32_t ce_code, uint32_t nof_cmds)
{
  std::lock_guard<std::mutex> lock(mutex);
  auto                        it = ues.find(rnti);
  if (it == ues.end()) {
    return SRSRAN_ERROR;
  }
  if (ce_code == (uint32_t)srsran::dl_sch_lcid::CON_RES_ID) {
    it->second.conres = true;
  }
  return SRSRAN_SUCCESS;
}

int sched_emtc::dl_ack_info(uint32_t tti, uint16_t rnti, uint32_t enb_cc_idx, uint32_t tb_idx, bool ack)
{
  std::lock_guard<std::mutex> lock(mutex);
  auto                        it = ues.find(rnti);
  if (it == ues.end() || !it->second.dlh.waiting || it->second.dlh.ack_tti != tti % NOF_TTI) {
    return 0;
  }
  dl_harq& h = it->second.dlh;
  h.waiting  = false;
  srsran::console("LTE-M: 0x%x HARQ-ACK in TTI %d: %s\n", rnti, tti, ack ? "ACK" : "NACK");
  if (!ack) {
    return 0;
  }
  h.active                = false;
  it->second.connected    = true;
  return (int)(h.tbs / 8);
}

int sched_emtc::dl_rach_info(uint32_t enb_cc_idx, dl_sched_rar_info_t rar_info)
{
  std::lock_guard<std::mutex> lock(mutex);
  if (!cfg) {
    return SRSRAN_ERROR;
  }
  return plan_rar({rar_info}) ? SRSRAN_SUCCESS : SRSRAN_ERROR;
}

int sched_emtc::dl_ri_info(uint32_t tti, uint16_t rnti, uint32_t enb_cc_idx, uint32_t ri_value)
{
  return SRSRAN_SUCCESS;
}

int sched_emtc::dl_pmi_info(uint32_t tti, uint16_t rnti, uint32_t enb_cc_idx, uint32_t pmi_value)
{
  return SRSRAN_SUCCESS;
}

int sched_emtc::dl_cqi_info(uint32_t tti, uint16_t rnti, uint32_t enb_cc_idx, uint32_t cqi_value)
{
  return SRSRAN_SUCCESS;
}

int sched_emtc::dl_sb_cqi_info(uint32_t tti, uint16_t rnti, uint32_t enb_cc_idx, uint32_t sb_idx, uint32_t cqi)
{
  return SRSRAN_SUCCESS;
}

int sched_emtc::ul_crc_info(uint32_t tti, uint16_t rnti, uint32_t enb_cc_idx, bool crc)
{
  srsran::console("LTE-M: PUSCH from 0x%x in TTI %d: CRC %s\n", rnti, tti, crc ? "OK" : "KO");
  std::lock_guard<std::mutex> lock(mutex);
  auto                        it = ues.find(rnti);
  if (it != ues.end() && it->second.ulh.waiting && it->second.ulh.pusch_tti == tti % NOF_TTI) {
    ue_ctxt& ue       = it->second;
    ue.ulh.waiting    = false;
    if (ue.ulh.nof_tx == 1) {
      // MCS cap from first transmissions: down 2 on a failure, up 1 after 10 in a row decoded
      if (!crc) {
        ue.ul_mcs_max = std::max(2u, ue.ul_mcs_max - std::min(2u, ue.ul_mcs_max));
        ue.ul_ok_run  = 0;
      } else if (++ue.ul_ok_run >= 10 && ue.ul_mcs_max < 9) {
        ue.ul_mcs_max++;
        ue.ul_ok_run = 0;
      }
    }
    if (crc) {
      ue.ulh.active = false;
    }
  }
  return SRSRAN_SUCCESS;
}

int sched_emtc::ul_sr_info(uint32_t tti, uint16_t rnti)
{
  std::lock_guard<std::mutex> lock(mutex);
  auto                        it = ues.find(rnti);
  if (it == ues.end()) {
    return SRSRAN_ERROR;
  }
  if (!it->second.ulh.active) {
    srsran::console("LTE-M: SR from 0x%x in TTI %d\n", rnti, tti);
  }
  it->second.sr = true;
  return SRSRAN_SUCCESS;
}

int sched_emtc::ul_bsr(uint16_t rnti, uint32_t lcg_id, uint32_t bsr)
{
  std::lock_guard<std::mutex> lock(mutex);
  auto                        it = ues.find(rnti);
  if (it == ues.end() || lcg_id >= it->second.ul_lcg.size()) {
    return SRSRAN_ERROR;
  }
  it->second.ul_lcg[lcg_id] = bsr;
  return SRSRAN_SUCCESS;
}

int sched_emtc::ul_phr(uint16_t rnti, int phr, uint32_t ul_nof_prb)
{
  return SRSRAN_SUCCESS;
}

int sched_emtc::ul_snr_info(uint32_t tti, uint16_t rnti, uint32_t enb_cc_idx, float snr, uint32_t ul_ch_code)
{
  if (ul_ch_code != mac_interface_phy_lte::PUSCH || !std::isfinite(snr)) {
    return SRSRAN_SUCCESS;
  }
  std::lock_guard<std::mutex> lock(mutex);
  auto                        it = ues.find(rnti);
  if (it != ues.end()) {
    float& avg = it->second.ul_snr;
    avg        = avg < -100 ? snr : 0.7f * avg + 0.3f * snr;
  }
  return SRSRAN_SUCCESS;
}

int sched_emtc::dl_sched(uint32_t tti, uint32_t enb_cc_idx, dl_sched_res_t& sched_result)
{
  return SRSRAN_SUCCESS;
}

int sched_emtc::ul_sched(uint32_t tti, uint32_t enb_cc_idx, ul_sched_res_t& sched_result)
{
  return SRSRAN_SUCCESS;
}

int sched_emtc::set_pdcch_order(uint32_t enb_cc_idx, dl_sched_po_info_t pdcch_order_info)
{
  return SRSRAN_ERROR;
}

std::array<int, SRSRAN_MAX_CARRIERS> sched_emtc::get_enb_ue_cc_map(uint16_t rnti)
{
  std::array<int, SRSRAN_MAX_CARRIERS> m = {};
  m.fill(-1);
  m[0] = 0;
  return m;
}

std::array<int, SRSRAN_MAX_CARRIERS> sched_emtc::get_enb_ue_activ_cc_map(uint16_t rnti)
{
  return get_enb_ue_cc_map(rnti);
}

int sched_emtc::ul_buffer_add(uint16_t rnti, uint32_t lcid, uint32_t bytes)
{
  std::lock_guard<std::mutex> lock(mutex);
  auto                        it = ues.find(rnti);
  if (it != ues.end()) {
    it->second.ul_lcg[0] += bytes;
  }
  return SRSRAN_SUCCESS;
}

} // namespace srsenb
