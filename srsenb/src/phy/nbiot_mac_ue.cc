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

// Scheduling of NB-IoT UEs in RRC_CONNECTED: one HARQ process each way, and one transaction at a time per UE (a
// category NB1 UE is half duplex). Downlink: DCI N1 in the UE-specific search space, NPDSCH, HARQ-ACK on NPUSCH
// format 2 (not decoded; RLC AM recovers losses). Uplink: DCI N0, NPUSCH format 1, decoded by the NPUSCH receiver.

#include "srsenb/hdr/phy/nbiot_mac.h"
#include "srsran/common/standard_streams.h"
#include <algorithm>
#include <cstring>

extern "C" {
#include "srsran/phy/phch/ra_nbiot.h"
}

namespace srsenb {

namespace {

constexpr uint32_t UE_TIMEOUT_SF   = 30000; ///< forget a UE whose uplink has been silent this long
constexpr uint32_t PLAN_AHEAD_SF   = 12;    ///< plan a UE only when its next opportunity is this close
constexpr uint32_t POLL_ACTIVE_SF  = 40;    ///< uplink grant without a buffer report, while the UE is active
constexpr uint32_t POLL_IDLE_SF    = 320;   ///< ... and after ACTIVE_SF without uplink data
constexpr uint32_t ACTIVE_SF       = 5000;
constexpr uint32_t UL_TURN_SF      = 100;   ///< longest a downlink burst may keep the uplink waiting: with no SR in NB-IoT
                                            ///< a UE with data and no grant starts random access and stops monitoring
                                            ///< its search space
constexpr uint32_t UL_RESULT_SF    = 400;   ///< the decoder answers well within this after the NPUSCH
constexpr uint32_t MAX_UL_FAILS    = 4;
constexpr uint32_t MAX_DL_TBS      = 680;  ///< category NB1 (36.306 4.1C)
constexpr uint32_t MAX_UL_TBS      = 1000;
constexpr uint32_t UL_K0[4]        = {8, 16, 32, 64};       // Table 16.5.1-1
constexpr uint32_t UL_N_RU[8]      = {1, 2, 3, 4, 5, 6, 8, 10}; // Table 16.5.1.1-2

// Upper bounds of the buffer size levels of 36.321 Table 6.1.3.1-1
constexpr uint32_t BSR_BYTES[64] = {0,     10,    12,    14,    17,    19,    22,     26,    31,    36,    42,
                                    49,    57,    67,    78,    91,    107,   125,    146,   171,   200,   234,
                                    274,   321,   376,   440,   515,   603,   706,    826,   967,   1132,  1326,
                                    1552,  1817,  2127,  2490,  2915,  3413,  3995,   4677,  5476,  6411,  7505,
                                    8787,  10287, 12043, 14099, 16507, 19325, 22624,  26487, 31009, 36304, 42502,
                                    49759, 58255, 68201, 79846, 93479, 109439, 128125, 150000, 150001};

std::string hex(const uint8_t* p, uint32_t n)
{
  std::string s;
  char        b[4];
  for (uint32_t i = 0; i < n; i++) {
    snprintf(b, sizeof(b), "%02x", p[i]);
    s += b;
  }
  return s;
}

/// Tones of a 15 kHz grant (Table 16.5.1.1-1): I_sc and the first subcarrier
bool isc_for(uint32_t n_sc, uint32_t* i_sc, uint32_t* first)
{
  switch (n_sc) {
    case 1:
      *i_sc  = 6;
      *first = 6;
      return true;
    case 3:
      *i_sc  = 13;
      *first = 3;
      return true;
    case 6:
      *i_sc  = 17;
      *first = 6;
      return true;
    case 12:
      *i_sc  = 18;
      *first = 0;
      return true;
    default:
      return false;
  }
}

uint32_t ul_slots_per_ru(uint32_t n_sc)
{
  return n_sc == 1 ? 16 : n_sc == 3 ? 8 : n_sc == 6 ? 4 : 2;
}

/// Single tone: I_MCS to (Qm, I_TBS) of Table 16.5.1.2-1. More tones: QPSK, I_TBS = I_MCS.
void ul_mcs(uint32_t n_sc, uint32_t i_mcs, uint32_t* qm, uint32_t* i_tbs)
{
  if (n_sc > 1) {
    *qm    = 2;
    *i_tbs = i_mcs;
    return;
  }
  static const uint32_t q[11] = {1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2};
  static const uint32_t t[11] = {0, 2, 1, 3, 4, 5, 6, 7, 8, 9, 10};
  *qm                         = q[std::min(i_mcs, 10u)];
  *i_tbs                      = t[std::min(i_mcs, 10u)];
}

/**
 * Downlink MAC PDU of tb bytes from SDUs (TS 36.321 6.1.2): every SDU but the last has a 7-bit length; one or two
 * spare octets become padding subheaders in front, more a length on the last SDU and a padding subheader after it.
 */
bool build_dl_pdu(const std::vector<std::pair<uint32_t, std::vector<uint8_t> > >& sdus, uint32_t tb, uint8_t* out)
{
  if (sdus.empty()) {
    return false;
  }
  uint32_t payload = 0;
  for (const auto& s : sdus) {
    if (s.second.size() >= 128) {
      return false;
    }
    payload += (uint32_t)s.second.size();
  }
  const uint32_t k    = (uint32_t)sdus.size();
  const uint32_t base = 2 * (k - 1) + 1 + payload;
  if (base > tb) {
    return false;
  }
  const uint32_t pad = tb - base;
  uint32_t       p   = 0;
  memset(out, 0, tb);
  if (pad <= 2) {
    for (uint32_t i = 0; i < pad; i++) {
      out[p++] = 0x3F;
    }
  }
  for (uint32_t i = 0; i < k; i++) {
    const bool last_sdu = i + 1 == k;
    if (!last_sdu || pad > 2) {
      out[p++] = (uint8_t)(0x20 | (sdus[i].first & 0x1f));
      out[p++] = (uint8_t)sdus[i].second.size();
    } else {
      out[p++] = (uint8_t)(sdus[i].first & 0x1f);
    }
  }
  if (pad > 2) {
    out[p++] = 0x1F;
  }
  for (const auto& s : sdus) {
    memcpy(out + p, s.second.data(), s.second.size());
    p += (uint32_t)s.second.size();
  }
  return p <= tb;
}

} // namespace

bool nbiot_mac::overlaps_nprach(uint64_t a, uint64_t b) const
{
  if (cfg.nprach_period_ms == 0 || b < a) {
    return false;
  }
  const uint64_t P   = cfg.nprach_period_ms;
  const uint64_t S   = cfg.nprach_start_ms;
  const uint64_t dur = (preamble_duration_us(cfg.nprach_n_rep, cfg.nprach_format) + 999) / 1000 + 1;
  const uint64_t lo  = a > S + dur ? (a - S - dur) / P : 0;
  for (uint64_t k = lo;; k++) {
    const uint64_t o = k * P + S;
    if (o > b) {
      return false;
    }
    if (o + dur > a) {
      return true;
    }
  }
}

bool nbiot_mac::find_candidate(const search_space&                                                        ss,
                               uint64_t                                                                   t_min,
                               uint64_t                                                                   t_max,
                               uint32_t                                                                   k0d,
                               uint32_t                                                                   n_sf,
                               uint32_t                                                                   n_rep,
                               const std::function<bool(const srsran_nbiot_plan_t&, const srsran_nbiot_plan_t&)>& check,
                               srsran_nbiot_plan_t&                                                       pc,
                               srsran_nbiot_plan_t&                                                       pd)
{
  const uint64_t        now = srsran_nbiot_dl_sched_now(sched);
  srsran_nbiot_layout_t layout;
  if (now == 0 || !srsran_nbiot_dl_sched_get_layout(sched, &layout)) {
    return false;
  }
  const uint64_t min_t  = now + cfg.lead_sf;
  uint32_t       period = 0;
  t_min                 = std::max(t_min, min_t);
  while (true) {
    const uint64_t k0 = srsran_nbiot_search_space_start(ss.r_max, ss.g_halves, ss.offset_eighths, t_min, &period);
    if (k0 > t_max || period == 0) {
      return false;
    }
    if (srsran_nbiot_plan_npdcch(&layout, k0, ss.r_max, &pc) != SRSRAN_SUCCESS) {
      return false;
    }
    pd.nof_sf = 0;
    if (n_sf > 0 &&
        srsran_nbiot_plan_npdsch(&layout, pc.t[pc.nof_sf - 1], k0d, n_sf, n_rep, &pd) != SRSRAN_SUCCESS) {
      return false;
    }
    bool ok = true;
    for (uint32_t i = 0; i < pc.nof_sf && ok; i++) {
      ok = pc.t[i] >= min_t && !srsran_nbiot_dl_sched_busy(sched, pc.t[i]);
    }
    for (uint32_t i = 0; i < pd.nof_sf && ok; i++) {
      ok = !srsran_nbiot_dl_sched_busy(sched, pd.t[i]);
    }
    if (ok && (!check || check(pc, pd))) {
      return true;
    }
    t_min = k0 + 1;
  }
}

bool nbiot_mac::parse_ul_pdu(const uint8_t*       pdu,
                             uint32_t             len,
                             uint16_t*            crnti,
                             int*                 bsr_bytes,
                             std::vector<ul_sdu>& sdus,
                             std::string&         why)
{
  // UL-SCH subheaders R/F2/E/LCID [F/L(7) | F/L(15) | L(16)] (36.321 6.1.2): control elements of fixed size and the
  // last subheader carry no length
  struct sub {
    uint32_t lcid;
    int      len; ///< -1: the rest of the PDU
  };
  std::vector<sub> subs;
  uint32_t         pos = 0;
  while (pos < len) {
    const uint8_t  h    = pdu[pos++];
    const bool     f2   = (h >> 6) & 1;
    const bool     more = (h >> 5) & 1;
    const uint32_t lcid = h & 0x1fu;
    sub            s    = {lcid, -1};
    switch (lcid) {
      case 26: // power headroom
      case 28: // truncated BSR
      case 29: // short BSR
        s.len = 1;
        break;
      case 27: // C-RNTI
        s.len = 2;
        break;
      case 30: // long BSR
        s.len = 3;
        break;
      case 31: // padding
        s.len = more ? 0 : -1;
        break;
      default:
        if (lcid > 10) {
          why = "LCID " + std::to_string(lcid) + " is not handled";
          return false;
        }
        if (more) {
          if (pos >= len) {
            why = "header runs past the PDU";
            return false;
          }
          if (f2) {
            if (pos + 1 >= len) {
              why = "header runs past the PDU";
              return false;
            }
            s.len = ((int)pdu[pos] << 8) | pdu[pos + 1];
            pos += 2;
          } else if (pdu[pos] & 0x80) {
            if (pos + 1 >= len) {
              why = "header runs past the PDU";
              return false;
            }
            s.len = (((int)pdu[pos] & 0x7f) << 8) | pdu[pos + 1];
            pos += 2;
          } else {
            s.len = pdu[pos++];
          }
        }
    }
    subs.push_back(s);
    if (!more) {
      break;
    }
  }
  for (const sub& s : subs) {
    const uint32_t n = s.len < 0 ? len - std::min(pos, len) : (uint32_t)s.len;
    if (pos + n > len) {
      why = "LCID " + std::to_string(s.lcid) + " of " + std::to_string(n) + " bytes runs past the PDU";
      return false;
    }
    const uint8_t* p = pdu + pos;
    switch (s.lcid) {
      case 27:
        *crnti = (uint16_t)((p[0] << 8) | p[1]);
        break;
      case 28:
      case 29:
        *bsr_bytes = (int)BSR_BYTES[p[0] & 0x3f];
        break;
      case 30: {
        const uint32_t v = ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2];
        *bsr_bytes       = (int)(BSR_BYTES[(v >> 18) & 0x3f] + BSR_BYTES[(v >> 12) & 0x3f] + BSR_BYTES[(v >> 6) & 0x3f] +
                           BSR_BYTES[v & 0x3f]);
        break;
      }
      case 26:
      case 31:
        break;
      default:
        if (n > 0) {
          sdus.push_back({s.lcid, std::vector<uint8_t>(p, p + n)});
        }
    }
    pos += n;
  }
  return true;
}

void nbiot_mac::release_ue(uint16_t rnti)
{
  std::lock_guard<std::mutex> guard(lock);
  if (ues.erase(rnti)) {
    srsran::console("NB-IoT: UE 0x%04x released\n", rnti);
  }
}

bool nbiot_mac::schedule_dl(ue_ctx& ue, uint64_t t_min, std::vector<std::string>& log)
{
  nbiot_rrc_interface_mac* r = rrc.load();
  if (r == nullptr) {
    return false;
  }
  static const uint32_t order[3] = {NBIOT_LCID_SRB1BIS, NBIOT_LCID_SRB1, NBIOT_LCID_DRB1};
  uint32_t              pending[3];
  uint32_t              need = 0;
  for (uint32_t i = 0; i < 3; i++) {
    pending[i] = r->dl_buffer(ue.rnti, order[i]);
    // MAC subheader, plus a byte because RLC AM reports one byte less than the PDU that carries the tail of an SDU
    need += pending[i] > 0 ? pending[i] + 3 : 0;
  }
  if (need == 0) {
    return false;
  }

  // Fewest subframes, then the smallest block, that holds everything at a code rate of at most 0.7; if nothing does,
  // the largest block
  const uint32_t e_sf = srsran_nbiot_dlch_bits_per_sf(&dlch);
  int            i_sf = -1, i_tbs = -1;
  uint32_t       tbs  = 0;
  int            big_sf = -1, big_tbs = -1;
  uint32_t       big    = 0;
  for (uint32_t s = 0; s < 8; s++) {
    for (uint32_t t = 0; t < 13; t++) {
      const uint32_t b = srsran_nbiot_npdsch_tbs(t, s);
      if (b == 0 || b > MAX_DL_TBS || (b + 24) * 10 > srsran_nbiot_npdsch_n_sf(s) * e_sf * 7) {
        continue;
      }
      if (b >= need * 8 && (i_sf < 0 || ((uint32_t)i_sf == s && b < tbs))) {
        i_sf  = (int)s;
        i_tbs = (int)t;
        tbs   = b;
      }
      if (b > big) {
        big     = b;
        big_sf  = (int)s;
        big_tbs = (int)t;
      }
    }
  }
  if (i_sf < 0) {
    i_sf  = big_sf;
    i_tbs = big_tbs;
    tbs   = big;
  }
  if (i_sf < 0) {
    return false;
  }
  const uint32_t n_sf = srsran_nbiot_npdsch_n_sf((uint32_t)i_sf);
  const int      k0d  = srsran_nbiot_npdsch_k0(0, uss().r_max);

  // See that there is room before anything is taken out of RLC
  srsran_nbiot_plan_t pc, pd;
  auto                ack_clear = [this](const srsran_nbiot_plan_t&, const srsran_nbiot_plan_t& d) {
    const uint64_t n = d.t[d.nof_sf - 1];
    return !overlaps_nprach(n + HARQ_ACK_K0, n + HARQ_ACK_K0 + HARQ_ACK_SF - 1);
  };
  if (k0d < 0 || !find_candidate(uss(), t_min, t_min + 64, (uint32_t)k0d, n_sf, 1, ack_clear, pc, pd)) {
    return false;
  }

  const uint32_t                                             tb        = tbs / 8;
  uint32_t                                                   remaining = tb;
  std::vector<std::pair<uint32_t, std::vector<uint8_t> > > sdus;
  for (uint32_t i = 0; i < 3; i++) {
    if (pending[i] == 0 || remaining < 4) {
      continue;
    }
    std::vector<uint8_t> buf(remaining);
    const int            n = r->read_pdu(ue.rnti, order[i], buf.data(), remaining - 2);
    if (n > 0) {
      buf.resize((uint32_t)n);
      remaining -= (uint32_t)n + 2;
      sdus.emplace_back(order[i], std::move(buf));
    }
  }
  uint8_t pdu[MAX_DL_TBS / 8];
  if (!build_dl_pdu(sdus, tb, pdu)) {
    return false;
  }

  srsran_nbiot_dci_n1_t dci = {};
  dci.i_delay               = 0;
  dci.i_sf                  = (uint32_t)i_sf;
  dci.i_mcs                 = (uint32_t)i_tbs;
  dci.i_rep                 = 0;
  ue.dl_ndi ^= 1u;
  dci.ndi          = ue.dl_ndi;
  dci.harq_ack_res = 0;
  dci.dci_rep      = 0;
  dl_alloc    a;
  std::string why;
  if (!plan_dl(uss(), ue.rnti, dci, pdu, tbs, t_min, t_min + 64, true, a, why)) {
    char line[160];
    snprintf(line, sizeof(line), "NB-IoT: downlink to 0x%04x lost after taking it from RLC: %s", ue.rnti, why.c_str());
    log.emplace_back(line);
    return false;
  }
  ue.busy_until = a.npdsch_end + HARQ_ACK_K0 + HARQ_ACK_SF + UE_GAP_SF;

  std::string what;
  for (const auto& s : sdus) {
    what += " LCID " + std::to_string(s.first) + ":" + std::to_string(s.second.size());
  }
  char line[256];
  snprintf(line,
           sizeof(line),
           "NB-IoT: DL 0x%04x TBS %u (I_TBS %d, %u SF) NPDCCH %llu NPDSCH %llu..%llu NDI %u:%s",
           ue.rnti,
           tbs,
           i_tbs,
           n_sf,
           (unsigned long long)a.npdcch_start,
           (unsigned long long)a.npdsch_start,
           (unsigned long long)a.npdsch_end,
           dci.ndi,
           what.c_str());
  log.emplace_back(line);
  return true;
}

bool nbiot_mac::schedule_ul(ue_ctx& ue, uint64_t t_min, std::vector<std::string>& log)
{
  if (!request_npusch) {
    return false;
  }
  const bool         in_css = ue.contention;
  const search_space ss     = in_css ? css() : uss();

  srsran_nbiot_dci_n0_t dci = {};
  uint32_t              tbs = 0;
  uint32_t              n_sc = ue.ul_n_sc, first = 0, qm = 2;
  if (ue.ul_retx && ue.last_tbs > 0) {
    dci = ue.last_n0;
    tbs = ue.last_tbs;
    // the tones of the grant being repeated
    n_sc = dci.i_sc < 12 ? 1 : dci.i_sc < 16 ? 3 : dci.i_sc < 18 ? 6 : 12;
    uint32_t i_sc;
    isc_for(n_sc, &i_sc, &first);
    uint32_t i_tbs;
    ul_mcs(n_sc, dci.i_mcs, &qm, &i_tbs);
  } else {
    // Enough for what the UE reported, or a small block to let it report
    const uint32_t need = std::min<uint32_t>(ue.bsr_bytes > 0 ? ue.bsr_bytes + 6 : 24, MAX_UL_TBS / 8);
    uint32_t       i_sc = 0;
    if (!isc_for(n_sc, &i_sc, &first)) {
      return false;
    }
    const uint32_t bits_ru = ul_slots_per_ru(n_sc) * 6 * n_sc * 2;
    int            best_ru = -1, best_mcs = -1, big_ru = -1, big_mcs = -1;
    uint32_t       best = 0, big = 0;
    const uint32_t max_mcs = n_sc > 1 ? 12 : 10;
    for (uint32_t ru = 0; ru < 8; ru++) {
      for (uint32_t m = n_sc > 1 ? 0 : 2; m <= max_mcs; m++) {
        uint32_t q, t;
        ul_mcs(n_sc, m, &q, &t);
        const int b = srsran_ra_nbiot_get_npusch_tbs(t, ru);
        if (b <= 0 || (uint32_t)b > MAX_UL_TBS || ((uint32_t)b + 24) * 10 > UL_N_RU[ru] * bits_ru * 7) {
          continue;
        }
        if ((uint32_t)b >= need * 8 && (best_ru < 0 || ((uint32_t)best_ru == ru && (uint32_t)b < best))) {
          best_ru  = (int)ru;
          best_mcs = (int)m;
          best     = (uint32_t)b;
        }
        if ((uint32_t)b > big) {
          big     = (uint32_t)b;
          big_ru  = (int)ru;
          big_mcs = (int)m;
        }
      }
    }
    if (best_ru < 0) {
      best_ru  = big_ru;
      best_mcs = big_mcs;
      best     = big;
    }
    if (best_ru < 0) {
      return false;
    }
    dci.i_sc    = i_sc;
    dci.i_ru    = (uint32_t)best_ru;
    dci.i_delay = 0;
    dci.i_mcs   = (uint32_t)best_mcs;
    dci.rv      = 0;
    dci.i_rep   = 0;
    tbs         = best;
    uint32_t i_tbs;
    ul_mcs(n_sc, dci.i_mcs, &qm, &i_tbs);
  }
  dci.dci_rep = in_css ? (uint32_t)srsran_nbiot_dci_rep_for_rmax(cfg.r_max) : 0;

  const uint32_t n_ru    = UL_N_RU[dci.i_ru];
  const uint32_t ul_sf   = n_ru * ul_slots_per_ru(n_sc) / 2;
  const uint32_t k0      = UL_K0[dci.i_delay];
  auto           ul_free = [this, k0, ul_sf](const srsran_nbiot_plan_t& c, const srsran_nbiot_plan_t&) {
    const uint64_t s = c.t[c.nof_sf - 1] + k0 + 1;
    return !overlaps_nprach(s, s + ul_sf - 1);
  };
  srsran_nbiot_plan_t pc, pd;
  const uint64_t      t_max = t_min + (in_css ? 256 : 64);
  if (!find_candidate(ss, t_min, t_max, 0, 0, 0, ul_free, pc, pd)) {
    return false;
  }

  // The receiver first: a grant whose transmission nobody listens to is worse than none
  const uint64_t      start = pc.t[pc.nof_sf - 1] + k0 + 1;
  nbiot_npusch_expect e;
  e.start_sf          = start;
  e.cfg.n_sc          = n_sc;
  e.cfg.spacing_hz    = 15000;
  e.cfg.sc            = first;
  e.cfg.n_ru          = n_ru;
  e.cfg.n_rep         = 1;
  e.cfg.rv            = dci.rv ? 2 : 0;
  e.cfg.qm            = qm;
  e.cfg.tbs           = tbs;
  e.cfg.rnti          = ue.rnti;
  e.cfg.cell_id       = cfg.cell_id;
  e.cfg.frame         = (uint32_t)((start / 10) % 1024);
  e.cfg.slot          = (uint32_t)(2 * (start % 10));
  e.cfg.group_hopping = cfg.group_hopping;
  e.cfg.delta_ss      = cfg.delta_ss;
  e.cfg.base_seq      = -1;
  e.rnti              = ue.rnti;
  e.connected         = true;
  std::string why;
  if (!request_npusch(e, why)) {
    log.push_back("NB-IoT: uplink grant not given, the receiver refused it: " + why);
    return false;
  }

  if (!ue.ul_retx) {
    ue.ul_ndi ^= 1u;
  }
  dci.ndi = ue.ul_ndi;
  uint8_t dci_bits[SRSRAN_NBIOT_DCI_LEN];
  uint8_t enc[SRSRAN_NBIOT_DL_SCHED_MAX_E];
  if (srsran_nbiot_dci_n0_pack(&dci, dci_bits) != SRSRAN_SUCCESS ||
      srsran_nbiot_npdcch_encode(&dlch, dci_bits, SRSRAN_NBIOT_DCI_LEN, ue.rnti, enc) != SRSRAN_SUCCESS ||
      srsran_nbiot_dl_sched_add_npdcch(
          sched, enc, srsran_nbiot_dlch_bits_per_sf(&dlch), &pc, srsran_nbiot_dl_sched_now(sched) + cfg.lead_sf) !=
          SRSRAN_SUCCESS) {
    log.push_back("NB-IoT: uplink grant could not be scheduled");
    return false;
  }
  ue.last_n0     = dci;
  ue.last_tbs    = tbs;
  ue.ul_inflight = true;
  ue.ul_deadline = start + ul_sf + UL_RESULT_SF;
  ue.busy_until  = start + ul_sf + UE_GAP_SF;
  ue.last_grant  = srsran_nbiot_dl_sched_now(sched);
  ue.contention  = false;

  char line[256];
  snprintf(line,
           sizeof(line),
           "NB-IoT: UL grant 0x%04x%s: %u tones @%u, %u RU, TBS %u, NDI %u%s, NPDCCH %llu, NPUSCH %llu..%llu",
           ue.rnti,
           in_css ? " (contention resolution, CSS)" : "",
           n_sc,
           first,
           n_ru,
           tbs,
           dci.ndi,
           ue.ul_retx ? " (retx)" : "",
           (unsigned long long)pc.t[0],
           (unsigned long long)start,
           (unsigned long long)(start + ul_sf - 1));
  log.emplace_back(line);
  return true;
}

void nbiot_mac::tick()
{
  std::vector<std::string> log;
  std::vector<uint16_t>    lost;
  nbiot_rrc_interface_mac* r = rrc.load();
  {
    std::lock_guard<std::mutex> guard(lock);
    const uint64_t              now = initiated ? srsran_nbiot_dl_sched_now(sched) : 0;
    if (now == 0) {
      return;
    }
    schedule_paging(now, log);
    for (auto& kv : ues) {
      ue_ctx& ue = kv.second;
      if (ue.ul_inflight) {
        if (now <= ue.ul_deadline) {
          continue;
        }
        ue.ul_inflight = false;
        ue.ul_retx     = true;
        log.push_back("NB-IoT: no decoder result for an uplink grant");
      }
      if (now > ue.last_rx + UE_TIMEOUT_SF) {
        lost.push_back(ue.rnti);
        continue;
      }
      const uint64_t t_min = std::max(ue.busy_until, now + cfg.lead_sf);
      if (t_min > now + PLAN_AHEAD_SF) {
        continue;
      }
      if (ue.contention) {
        schedule_ul(ue, t_min, log);
        continue;
      }
      if ((ue.ul_retx || ue.bsr_bytes > 0 || now >= ue.last_grant + UL_TURN_SF) && schedule_ul(ue, t_min, log)) {
        continue;
      }
      if (schedule_dl(ue, t_min, log)) {
        continue;
      }
      const uint32_t poll = now < ue.last_rx + ACTIVE_SF ? POLL_ACTIVE_SF : POLL_IDLE_SF;
      if (ue.bsr_bytes > 0 || ue.ul_retx || now >= ue.last_grant + poll) {
        schedule_ul(ue, t_min, log);
      }
    }
    for (uint16_t rnti : lost) {
      ues.erase(rnti);
    }
  }
  for (const std::string& l : log) {
    srsran::console("%s\n", l.c_str());
    logger.info("%s", l.c_str());
  }
  for (uint16_t rnti : lost) {
    srsran::console("NB-IoT: UE 0x%04x silent for %u ms, dropped\n", rnti, UE_TIMEOUT_SF);
    if (r != nullptr) {
      r->ue_lost(rnti);
    }
  }
}

void nbiot_mac::npusch_received(const nbiot_npusch_result& res)
{
  const uint16_t      rnti = res.req.rnti;
  std::vector<ul_sdu> sdus;
  std::string         why;
  uint16_t            crnti = 0;
  int                 bsr   = -1;
  bool                ok    = res.res.crc_ok;
  bool                known = false;
  if (ok && !parse_ul_pdu(res.tb.data(), (uint32_t)res.tb.size(), &crnti, &bsr, sdus, why)) {
    srsran::console("NB-IoT: UL 0x%04x PDU %s: %s\n", rnti, hex(res.tb.data(), res.tb.size()).c_str(), why.c_str());
  }
  {
    std::lock_guard<std::mutex> guard(lock);
    auto                        it = ues.find(rnti);
    if (it != ues.end()) {
      known          = true;
      ue_ctx& ue     = it->second;
      ue.ul_inflight = false;
      if (ok) {
        ue.last_rx   = srsran_nbiot_dl_sched_now(sched);
        ue.ul_retx   = false;
        ue.ul_fails  = 0;
        ue.bsr_bytes = bsr >= 0 ? (uint32_t)bsr : 0;
      } else if (++ue.ul_fails >= MAX_UL_FAILS) {
        ue.ul_retx  = false;
        ue.ul_fails = 0;
        if (ue.ul_n_sc > 1) {
          ue.ul_n_sc = ue.ul_n_sc == 12 ? 3 : 1;
        }
      } else {
        ue.ul_retx = true;
      }
    }
  }
  std::string what;
  for (const ul_sdu& s : sdus) {
    what += " LCID " + std::to_string(s.lcid) + ":" + std::to_string(s.data.size());
  }
  srsran::console("NB-IoT: UL 0x%04x %u tones %u RU TBS %u: CRC %s, SNR %.1f dB, CFO %+.0f Hz%s%s\n",
                  rnti,
                  res.req.cfg.n_sc,
                  res.req.cfg.n_ru,
                  res.req.cfg.tbs,
                  ok ? "OK" : "KO",
                  res.res.snr_db,
                  res.res.cfo_hz,
                  bsr >= 0 ? (", BSR " + std::to_string(bsr)).c_str() : "",
                  what.c_str());
  nbiot_rrc_interface_mac* r = rrc.load();
  if (known && r != nullptr) {
    for (const ul_sdu& s : sdus) {
      r->write_pdu(rnti, s.lcid, s.data.data(), (uint32_t)s.data.size());
    }
  }
}

} // namespace srsenb
