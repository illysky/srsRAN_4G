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

#ifndef SRSENB_NBIOT_MAC_H
#define SRSENB_NBIOT_MAC_H

#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "nbiot_sib_builder.h"
#include "srsenb/hdr/phy/nbiot_msg3_worker.h"
#include "srsenb/hdr/phy/nbiot_prach_worker.h"
#include "srsran/interfaces/enb_nbiot_interfaces.h"
#include <atomic>
#include <map>
#include "srsran/srslog/srslog.h"
extern "C" {
#include "srsran/phy/phch/nbiot_dl_sched.h"
#include "srsran/phy/phch/nbiot_dlch.h"
#include "srsran/phy/phch/nbiot_ra.h"
}

namespace srsenb {

/**
 * Random access of the NB-IoT carrier, Msg2 (TS 36.321 5.1.4, 36.213 16.3.3, 16.4.1, 16.6).
 *
 * A detected preamble is answered with a random access response: an NPDCCH carrying DCI format N1 scrambled with the
 * RA-RNTI in the Type-2 common search space, and the NPDSCH it points at holding one MAC RAR (timing advance, Msg3
 * grant, temporary C-RNTI). Everything is planned in absolute subframes and handed to the downlink composer through
 * the shared table; nothing here touches samples.
 */
struct nbiot_ra_config {
  // NPRACH as broadcast in SIB2-NB
  uint32_t nprach_n_rep  = 0;
  uint32_t nprach_format = 0; ///< 0: 66.7 us cyclic prefix, 1: 266.7 us

  // Type-2 common search space (RAR, Msg3 retransmissions, Msg4)
  uint32_t r_max          = 0; ///< npdcch-NumRepetitions-RA
  uint32_t g_halves       = 0; ///< npdcch-StartSF-CSS-RA in halves (3: 1.5, 4: 2, 8: 4 ...)
  uint32_t offset_eighths = 0; ///< npdcch-Offset-RA: 0, 1, 2, 3 eighths
  uint32_t window_pp      = 0; ///< ra-ResponseWindowSize in search-space periods
  uint32_t contention_pp  = 0; ///< mac-ContentionResolutionTimer in search-space periods

  // NPRACH occasions, which NPUSCH of connected UEs stays clear of
  uint32_t nprach_period_ms = 0;
  uint32_t nprach_start_ms  = 0;

  // NPUSCH of the cell
  uint32_t cell_id       = 0;
  bool     group_hopping = false;
  uint32_t delta_ss      = 0;

  // What the response contains
  uint32_t                  rar_i_rep = 2;                ///< repetitions of the RAR NPDSCH: Table 16.4.1.3-2 index
  srsran_nbiot_msg3_grant_t msg3      = {1, 6, 0, 0, 0};  ///< 15 kHz, tone 6, k0 = 12, 1 repetition, 4 RUs of BPSK

  // Planning
  uint32_t lead_sf = 4; ///< subframes after the newest one the composer asked for that still can be planned

  /// From the broadcast cell description; fails with a message for a value no search space can be built from
  static bool from_cell(const nbiot::cell_config& cell, nbiot_ra_config& out, std::string& err);
};

/// A response that was planned, and what the UE is being told to do next
struct nbiot_ra_response {
  uint32_t                  preamble = 0; ///< RAPID = starting subcarrier
  uint16_t                  ra_rnti  = 0;
  uint16_t                  tc_rnti  = 0;
  uint32_t                  ta       = 0;
  srsran_nbiot_msg3_grant_t grant    = {};

  uint64_t preamble_start = 0; ///< absolute subframe in which the preamble nominally started
  uint64_t window_start   = 0; ///< RAR window, first and last subframe
  uint64_t window_end     = 0;
  uint64_t npdcch_start   = 0; ///< first subframe of the search space candidate
  uint64_t npdcch_end     = 0; ///< last NPDCCH subframe n
  uint64_t npdsch_start   = 0;
  uint64_t npdsch_end     = 0; ///< last subframe of the RAR NPDSCH: Msg3 is scheduled from here (k0 of the grant)
  uint8_t  pdu[SRSRAN_NBIOT_RAR_LEN + 1] = {};
  uint32_t pdu_len = 0;
};

class nbiot_mac : public nbiot_mac_interface_rrc
{
public:
  explicit nbiot_mac(srslog::basic_logger& logger);
  ~nbiot_mac();
  nbiot_mac(const nbiot_mac&) = delete;
  nbiot_mac& operator=(const nbiot_mac&) = delete;

  /// lte_cell and nb_cell: the carrier the composers transmit; sched: the table they read
  bool init(const nbiot_ra_config& cfg, const srsran_nbiot_cell_t& nb_cell, srsran_nbiot_dl_sched_t* sched, std::string& err);

  /**
   * Handles a preamble reported by the receiver (any thread). Returns true if a response was planned; otherwise the
   * reason is in 'why'. 'resp' is filled on success.
   */
  bool on_preamble(const nbiot_nprach_detection& d, nbiot_ra_response& resp, std::string& why);

  /// The same, without the result (what the receiver's callback uses); logs the outcome
  void preamble_detected(const nbiot_nprach_detection& d);

  /// Called (from preamble_detected) for every response planned, so the Msg3 it grants can be received
  using response_callback = std::function<void(const nbiot_ra_response&)>;
  void set_response_callback(response_callback cb) { on_response = std::move(cb); }

  /// A Msg3 transport block of the UE given tc_rnti arrived: MAC PDU with the CCCH SDU (TS 36.321 6.1.2, 6.2.1)
  void msg3_received(uint16_t tc_rnti, const uint8_t* pdu, uint32_t len);

  /// Responses whose Msg3 has not been dealt with yet
  std::vector<nbiot_ra_response> pending() const;

  // Connected UEs (TS 36.321 5.3, 5.4; 36.213 16.4, 16.5, 16.6)

  /// Registers an NPUSCH format 1 reception; false if the receiver cannot take it
  using npusch_request = std::function<bool(const nbiot_npusch_expect&, std::string&)>;
  void set_npusch_request(npusch_request f) { request_npusch = std::move(f); }

  void set_rrc(nbiot_rrc_interface_mac* rrc_) override { rrc = rrc_; }
  void release_ue(uint16_t rnti) override;

  /// Schedules the connected UEs; call once per subframe (txrx thread)
  void tick();

  /// The NPUSCH format 1 of a connected UE (expect.connected) came in
  void npusch_received(const nbiot_npusch_result& r);

  struct counters {
    uint64_t preambles = 0, answered = 0, late = 0, no_room = 0, no_layout = 0, msg4 = 0;
  };
  counters stats() const;

  /// Length in microseconds of a preamble of n_rep repetitions, from its first sample to its last (TS 36.211 10.1.6)
  static uint32_t preamble_duration_us(uint32_t n_rep, uint32_t format);

  /// First subframe of the random access response window of a preamble that started in subframe 'start' (TS 36.321 5.1.4)
  static uint64_t window_start(uint64_t start, uint32_t n_rep, uint32_t format);

private:
  srslog::basic_logger&    logger;
  nbiot_ra_config          cfg;
  srsran_nbiot_dlch_t      dlch      = {};
  srsran_nbiot_dl_sched_t* sched     = nullptr;
  bool                     initiated = false;

  response_callback              on_response;
  npusch_request                 request_npusch;
  std::atomic<nbiot_rrc_interface_mac*> rrc{nullptr};
  mutable std::mutex             lock;
  uint16_t                       next_tc_rnti = NBIOT_FIRST_RNTI;
  std::vector<nbiot_ra_response> answered;
  counters                       cnt;

  /// A search space: Rmax, G in halves, offset in eighths of the period
  struct search_space {
    uint32_t r_max, g_halves, offset_eighths;
  };
  search_space css() const { return {cfg.r_max, cfg.g_halves, cfg.offset_eighths}; }

  struct ue_ctx {
    uint16_t rnti = 0;
    uint64_t busy_until = 0;      ///< first subframe a new NPDCCH to the UE may start in
    uint64_t last_rx    = 0;      ///< last subframe an uplink transport block of the UE decoded
    uint64_t last_grant = 0;
    bool     ul_inflight = false; ///< waiting for the decoder
    uint64_t ul_deadline = 0;
    bool     ul_retx     = false; ///< next grant retransmits the last transport block (NDI not toggled)
    uint32_t ul_fails    = 0;
    uint32_t ul_ndi = 0, dl_ndi = 0;
    uint32_t ul_n_sc   = 12;
    srsran_nbiot_dci_n0_t last_n0 = {};
    uint32_t last_tbs  = 0;
    uint32_t bsr_bytes = 0;      ///< what the UE reported it still has
    bool     contention = false; ///< random access with its C-RNTI: the next grant goes in the Type-2 CSS
  };
  std::map<uint16_t, ue_ctx> ues;

  uint16_t alloc_tc_rnti();

  /// Where a downlink transport block went
  struct dl_alloc {
    uint64_t npdcch_start = 0, npdcch_end = 0, npdsch_start = 0, npdsch_end = 0;
  };

  /// True if subframes [a, b] of the uplink overlap an NPRACH occasion
  bool overlaps_nprach(uint64_t a, uint64_t b) const;

  /// First NPDCCH candidate of ss starting in [t_min, t_max] whose subframes are free (and the NPDSCH after it with
  /// k0d, n_sf, if n_sf > 0), and whose acknowledgement / uplink window check(pc, pd) accepts. Caller holds the lock.
  bool find_candidate(const search_space&                                                        ss,
                      uint64_t                                                                   t_min,
                      uint64_t                                                                   t_max,
                      uint32_t                                                                   k0d,
                      uint32_t                                                                   n_sf,
                      uint32_t                                                                   n_rep,
                      const std::function<bool(const srsran_nbiot_plan_t&, const srsran_nbiot_plan_t&)>& check,
                      srsran_nbiot_plan_t&                                                       pc,
                      srsran_nbiot_plan_t&                                                       pd);

  bool schedule_dl(ue_ctx& ue, uint64_t t_min, std::vector<std::string>& log);
  bool schedule_ul(ue_ctx& ue, uint64_t t_min, std::vector<std::string>& log);

  /// Uplink MAC PDU of a connected UE: control elements update ue, SDUs are returned
  struct ul_sdu {
    uint32_t             lcid;
    std::vector<uint8_t> data;
  };
  bool parse_ul_pdu(const uint8_t* pdu, uint32_t len, uint16_t* crnti, int* bsr_bytes, std::vector<ul_sdu>& sdus,
                    std::string& why);

  /// HARQ-ACK of an NPDSCH (resource 0 of Table 16.4.2-1, 15 kHz): NPUSCH format 2 from n + 13, one RU of 2 ms
  static constexpr uint32_t HARQ_ACK_K0 = 13;
  static constexpr uint32_t HARQ_ACK_SF = 2;
  /// Subframes a UE needs after an uplink transmission before it monitors NPDCCH again (36.213 16.6)
  static constexpr uint32_t UE_GAP_SF = 3;

  /// UE-specific search space given in RRCConnectionSetup-NB: Rmax 1, G 8, offset 0
  static search_space uss() { return {1, 16, 0}; }

  /**
   * Plans DCI N1 (to rnti, in search space ss) and the NPDSCH it points at, on the first candidate that starts in
   * [t_min, t_max] with all its subframes free (and, with_ack, its HARQ-ACK clear of NPRACH). pdu holds tbs bits.
   * Caller holds the lock.
   */
  bool plan_dl(const search_space&          ss,
               uint16_t                     rnti,
               const srsran_nbiot_dci_n1_t& dci,
               const uint8_t*               pdu,
               uint32_t                     tbs,
               uint64_t                     t_min,
               uint64_t                     t_max,
               bool                         with_ack,
               dl_alloc&                    out,
               std::string&                 why);

  /// Contention resolution and RRCConnectionSetup-NB for the UE whose Msg3 carried ccch (TS 36.321 5.1.5, 36.331 5.3.3)
  bool send_msg4(uint16_t tc_rnti, const uint8_t* ccch, uint32_t ccch_len, std::string& why);
};

} // namespace srsenb

#endif // SRSENB_NBIOT_MAC_H
