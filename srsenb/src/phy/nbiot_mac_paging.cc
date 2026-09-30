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

// Paging of NB-IoT UEs in RRC_IDLE (TS 36.304 7, 36.331 5.3.2): Paging-NB on the NPDSCH, scheduled by DCI format N2
// with the P-RNTI in the Type-1 common search space, which starts at the UE's paging occasion (36.213 16.6).

#include "srsenb/hdr/phy/nbiot_mac.h"
#include "srsran/asn1/rrc_nbiot.h"
#include "srsran/common/standard_streams.h"
#include <algorithm>
#include <cstring>

namespace srsenb {

namespace {

constexpr uint16_t P_RNTI         = 0xFFFE;
constexpr uint32_t N2_LEN         = 15;  ///< DCI format N2, paging (36.212 6.4.3.3)
constexpr uint32_t PAGING_LEAD_SF = 20;  ///< a paging occasion closer than this is left for the next one
constexpr uint32_t PAGING_PLAN_SF = 200; ///< planned this early, before connected UEs take its subframes
constexpr uint32_t PAGING_CYCLES  = 2;   ///< a page is repeated for this many paging cycles after the last request

std::string id_str(const nbiot_paging_id& id)
{
  char b[32];
  if (id.s_tmsi) {
    snprintf(b, sizeof(b), "S-TMSI %02x-%08x", id.mmec, id.m_tmsi);
  } else {
    snprintf(b, sizeof(b), "IMSI (%zu digits)", id.imsi.size());
  }
  return b;
}

bool same_id(const nbiot_paging_id& a, const nbiot_paging_id& b)
{
  return a.s_tmsi == b.s_tmsi && (a.s_tmsi ? a.mmec == b.mmec && a.m_tmsi == b.m_tmsi : a.imsi == b.imsi);
}

/// PCCH-Message-NB with one paging record per identity; returns the length in bytes or -1
int pack_paging_nb(const std::vector<const nbiot_paging_id*>& ids, uint8_t* out, uint32_t max)
{
  asn1::rrc::pcch_msg_nb_s msg;
  auto&                    p = msg.msg.set_c1().paging_r13();
  p.paging_record_list_r13_present = true;
  for (const nbiot_paging_id* id : ids) {
    asn1::rrc::paging_record_nb_r13_s r;
    if (id->s_tmsi) {
      auto& s = r.ue_id_r13.set_s_tmsi();
      s.mmec.from_number(id->mmec);
      s.m_tmsi.from_number(id->m_tmsi);
    } else {
      auto& imsi = r.ue_id_r13.set_imsi();
      imsi.resize(id->imsi.size());
      std::copy(id->imsi.begin(), id->imsi.end(), imsi.begin());
    }
    p.paging_record_list_r13.push_back(r);
  }
  asn1::bit_ref bref(out, max);
  if (msg.pack(bref) != asn1::SRSASN_SUCCESS) {
    return -1;
  }
  return bref.distance_bytes();
}

/// DCI format N2 with Flag = 1 (paging), one bit per byte, msb first
void pack_n2(uint32_t i_sf, uint32_t i_mcs, uint32_t i_rep, uint32_t dci_rep, uint8_t* bits)
{
  uint32_t p = 0;
  auto     put = [&](uint32_t v, uint32_t n) {
    for (uint32_t i = 0; i < n; i++) {
      bits[p++] = (uint8_t)((v >> (n - 1 - i)) & 1u);
    }
  };
  put(1, 1);
  put(i_sf, 3);
  put(i_mcs, 4);
  put(i_rep, 4);
  put(dci_rep, 3);
}

/// DCI subframe repetition number of R = Rmax in the Type-1 CSS (Table 16.6-2, Rmax <= 128)
uint32_t type1_dci_rep(uint32_t r_max)
{
  uint32_t i = 0;
  while ((1u << i) < r_max) {
    i++;
  }
  return i;
}

uint64_t next_po(const nbiot_ra_config& c, uint32_t ue_id, const nbiot_paging_id& id, uint64_t t, uint64_t* end = nullptr)
{
  if (id.edrx_hf > 0) {
    return nbiot_mac::paging_occasion_edrx(c, ue_id, id, t, end);
  }
  if (end != nullptr) {
    *end = 0;
  }
  return nbiot_mac::paging_occasion(c, ue_id, t);
}

} // namespace

uint32_t nbiot_mac::hashed_id(uint32_t m_tmsi)
{
  // 32-bit FCS over b31..b0 of the S-TMSI, msb first: CRC-32 with generator 0x04C11DB7, preset and inverted
  uint32_t c = 0xFFFFFFFFu;
  for (int i = 31; i >= 0; i--) {
    const uint32_t bit = ((m_tmsi >> i) & 1u) ^ (c >> 31);
    c                  = (c << 1) ^ (bit ? 0x04C11DB7u : 0u);
  }
  return ~c;
}

uint64_t nbiot_mac::paging_occasion_edrx(const nbiot_ra_config& c,
                                         uint32_t               ue_id,
                                         const nbiot_paging_id& id,
                                         uint64_t               t,
                                         uint64_t*              window_end)
{
  // PH: H-SFN mod T_eDRX,H = UE_ID_H mod T_eDRX,H; the window starts in SFN 256 * i_eDRX of the PH and lasts L * 100
  // frames, possibly into the next hyperframe. UE_ID_H: the 12 msbs of the Hashed_ID, as P-RNTI is on the NPDCCH.
  const uint64_t hf       = 10240;
  const uint64_t T        = id.edrx_hf;
  const uint64_t ue_id_h  = hashed_id(id.m_tmsi) >> 20;
  const uint64_t ptw_sfn  = 256 * ((ue_id_h / T) % 4);
  uint64_t       h        = t / hf >= T ? t / hf - T : 0;
  h += (ue_id_h % T + T - h % T) % T;
  for (; h <= t / hf + 2 * T; h += T) {
    const uint64_t start = h * hf + ptw_sfn * 10;
    const uint64_t end   = start + (uint64_t)id.ptw_rf * 10 - 1;
    if (end < t) {
      continue;
    }
    const uint64_t po = paging_occasion(c, ue_id, std::max(t, start));
    if (po <= end) {
      if (window_end != nullptr) {
        *window_end = end;
      }
      return po;
    }
  }
  return UINT64_MAX;
}

uint64_t nbiot_mac::paging_occasion(const nbiot_ra_config& c, uint32_t ue_id, uint64_t t)
{
  // TS 36.304 7.1: N = min(T, nB), Ns = max(1, nB / T); PF: SFN mod T = (T div N) (UE_ID mod N);
  // i_s = floor(UE_ID / N) mod Ns; 7.2 (FDD): the subframe of i_s
  const uint64_t T  = c.paging_t_rf;
  const uint64_t nb = T * c.paging_nb_num / c.paging_nb_den;
  const uint64_t N  = std::min(T, nb);
  const uint32_t ns = c.paging_nb_den == 1 ? c.paging_nb_num : 1;
  static const uint32_t po_sf[3][4] = {{9, 9, 9, 9}, {4, 9, 9, 9}, {0, 4, 5, 9}};
  const uint32_t        row         = ns == 1 ? 0 : ns == 2 ? 1 : 2;
  const uint32_t        sf          = po_sf[row][(ue_id / N) % ns];
  const uint64_t        pf          = (T / N) * (ue_id % N);

  // SFN = frame mod 1024 and T divides 1024, so the absolute frame count can be used directly
  uint64_t f = t / 10;
  f += (pf + T - f % T) % T;
  if (f * 10 + sf < t) {
    f += T;
  }
  return f * 10 + sf;
}

void nbiot_mac::page(uint32_t ue_id, const nbiot_paging_id& id)
{
  std::lock_guard<std::mutex> guard(lock);
  if (!initiated || cfg.paging_t_rf == 0) {
    return;
  }
  const uint64_t now = srsran_nbiot_dl_sched_now(sched);
  uint64_t       end = 0;
  const uint64_t po  = next_po(cfg, ue_id, id, now + PAGING_LEAD_SF, &end);
  if (po == UINT64_MAX) {
    srsran::console("NB-IoT: no paging occasion for %s\n", id_str(id).c_str());
    return;
  }
  // In eDRX the page waits for the UE's next paging time window, which may be many hyperframes away
  const uint64_t expiry = std::max(now + (uint64_t)PAGING_CYCLES * cfg.paging_t_rf * 10 + PAGING_PLAN_SF, end + 1);
  for (page_entry& e : pages) {
    if (same_id(e.id, id)) {
      e.expiry = std::max(e.expiry, expiry);
      e.ue_id  = ue_id;
      e.id     = id;
      return;
    }
  }
  page_entry e;
  e.ue_id  = ue_id;
  e.id     = id;
  e.expiry = expiry;
  pages.push_back(e);
  if (id.edrx_hf > 0) {
    srsran::console("NB-IoT: paging %s, UE_ID %u, eDRX %u hyperframes, window %u frames: first occasion at H-SFN %llu "
                    "SFN %llu (in %.2f s), window ends at %llu\n",
                    id_str(id).c_str(),
                    ue_id,
                    id.edrx_hf,
                    id.ptw_rf,
                    (unsigned long long)(po / 10240) % 1024,
                    (unsigned long long)(po / 10) % 1024,
                    (po - now) / 1000.0,
                    (unsigned long long)end);
  } else {
    srsran::console("NB-IoT: paging %s, UE_ID %u, first occasion at %llu (now %llu)\n",
                    id_str(id).c_str(),
                    ue_id,
                    (unsigned long long)po,
                    (unsigned long long)now);
  }
}

void nbiot_mac::paging_answered(uint8_t mmec, uint32_t m_tmsi)
{
  std::lock_guard<std::mutex> guard(lock);
  for (auto it = pages.begin(); it != pages.end();) {
    if (it->id.s_tmsi && it->id.mmec == mmec && it->id.m_tmsi == m_tmsi) {
      srsran::console("NB-IoT: %s answered the page after %u paging message(s)\n",
                      id_str(it->id).c_str(),
                      it->sent);
      it = pages.erase(it);
    } else {
      ++it;
    }
  }
}

void nbiot_mac::schedule_paging(uint64_t now, std::vector<std::string>& log)
{
  pages.erase(std::remove_if(pages.begin(), pages.end(), [&](const page_entry& e) { return now > e.expiry; }),
              pages.end());
  if (pages.empty()) {
    return;
  }

  // The earliest occasion not yet planned
  uint64_t po = UINT64_MAX;
  for (const page_entry& e : pages) {
    po = std::min(po, next_po(cfg, e.ue_id, e.id, std::max(now + PAGING_LEAD_SF, e.last_po + 1)));
  }
  if (po > now + PAGING_PLAN_SF) {
    return;
  }
  std::vector<page_entry*>             due;
  std::vector<const nbiot_paging_id*> ids;
  for (page_entry& e : pages) {
    if (next_po(cfg, e.ue_id, e.id, std::max(now + PAGING_LEAD_SF, e.last_po + 1)) == po && ids.size() < 16) {
      due.push_back(&e);
      ids.push_back(&e.id);
    }
  }
  for (page_entry* e : due) {
    e->last_po = po;
  }

  uint8_t   msg[64];
  const int len = pack_paging_nb(ids, msg, sizeof(msg));
  char      line[256];
  if (len <= 0) {
    log.emplace_back("NB-IoT: Paging-NB does not pack");
    return;
  }

  // Smallest block in the fewest subframes at a code rate of at most 0.7
  const uint32_t e_sf = srsran_nbiot_dlch_bits_per_sf(&dlch);
  int            i_sf = -1, i_tbs = -1;
  uint32_t       tbs  = 0;
  for (uint32_t s = 0; s < 8 && i_sf < 0; s++) {
    for (uint32_t t = 0; t < 13; t++) {
      const int b = srsran_nbiot_npdsch_tbs(t, s);
      if (b >= len * 8 && (b + 24) * 10 <= (int)(srsran_nbiot_npdsch_n_sf(s) * e_sf) * 7) {
        i_sf  = (int)s;
        i_tbs = (int)t;
        tbs   = (uint32_t)b;
        break;
      }
    }
  }
  if (i_sf < 0) {
    log.emplace_back("NB-IoT: no transport block holds the Paging-NB");
    return;
  }
  const uint32_t n_sf = srsran_nbiot_npdsch_n_sf((uint32_t)i_sf);

  srsran_nbiot_layout_t layout;
  srsran_nbiot_plan_t   pc, pd;
  if (!srsran_nbiot_dl_sched_get_layout(sched, &layout) ||
      srsran_nbiot_plan_npdcch(&layout, po, cfg.paging_r_max, &pc) != SRSRAN_SUCCESS ||
      srsran_nbiot_plan_npdsch(&layout, pc.t[pc.nof_sf - 1], 0, n_sf, 1, &pd) != SRSRAN_SUCCESS) {
    log.emplace_back("NB-IoT: paging occasion " + std::to_string(po) + " cannot be planned");
    return;
  }
  const uint64_t min_t = now + cfg.lead_sf;
  bool           free  = true;
  for (uint32_t i = 0; i < pc.nof_sf && free; i++) {
    free = pc.t[i] >= min_t && !srsran_nbiot_dl_sched_busy(sched, pc.t[i]);
  }
  for (uint32_t i = 0; i < pd.nof_sf && free; i++) {
    free = !srsran_nbiot_dl_sched_busy(sched, pd.t[i]);
  }
  if (!free) {
    snprintf(line, sizeof(line), "NB-IoT: paging occasion %llu is taken, paging at the next one", (unsigned long long)po);
    log.emplace_back(line);
    return;
  }

  uint8_t pdu[SRSRAN_NBIOT_DL_SCHED_MAX_E / 8] = {};
  memcpy(pdu, msg, (size_t)len);
  uint8_t bits[N2_LEN];
  pack_n2((uint32_t)i_sf, (uint32_t)i_tbs, 0, type1_dci_rep(cfg.paging_r_max), bits);
  uint8_t e[SRSRAN_NBIOT_DL_SCHED_MAX_E];
  if (srsran_nbiot_npdcch_encode(&dlch, bits, N2_LEN, P_RNTI, e) != SRSRAN_SUCCESS ||
      srsran_nbiot_dl_sched_add_npdcch(sched, e, e_sf, &pc, min_t) != SRSRAN_SUCCESS) {
    log.emplace_back("NB-IoT: the downlink schedule refused the paging NPDCCH");
    return;
  }
  if (srsran_nbiot_npdsch_encode(&dlch, pdu, tbs, n_sf, e) != SRSRAN_SUCCESS ||
      srsran_nbiot_dl_sched_add_npdsch(sched, e, n_sf * e_sf, P_RNTI, n_sf, &pd, min_t) != SRSRAN_SUCCESS) {
    log.emplace_back("NB-IoT: the downlink schedule refused the paging NPDSCH");
    return;
  }

  std::string who;
  for (page_entry* p : due) {
    p->sent++;
    who += " " + id_str(p->id);
  }
  snprintf(line,
           sizeof(line),
           "NB-IoT: Paging-NB (%d bytes, TBS %u) at SFN %llu sf %llu: NPDCCH %llu..%llu (R %u), NPDSCH %llu..%llu:%s",
           len,
           tbs,
           (unsigned long long)((po / 10) % 1024),
           (unsigned long long)(po % 10),
           (unsigned long long)pc.t[0],
           (unsigned long long)pc.t[pc.nof_sf - 1],
           cfg.paging_r_max,
           (unsigned long long)pd.t[0],
           (unsigned long long)pd.t[pd.nof_sf - 1],
           who.c_str());
  log.emplace_back(line);
}

} // namespace srsenb
