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

#ifndef SRSENB_SCHED_EMTC_H
#define SRSENB_SCHED_EMTC_H

#include "srsenb/hdr/phy/emtc_config.h"
#include "srsenb/hdr/stack/mac/sched_interface.h"
#include "srsran/interfaces/enb_mac_interfaces.h"
#include "srsran/srslog/srslog.h"
#include <array>
#include <bitset>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

extern "C" {
#include "srsran/phy/fec/softbuffer.h"
}

namespace srsenb {

/**
 * Scheduler of the BL/CE UEs (LTE-M, CE mode A, no repetitions) of the first cell. It sits next to the LTE scheduler:
 * the MAC hands it the RACHs on the BR preambles and every call about a BL/CE UE, and each TTI asks it for what goes
 * on the narrowbands (MPDCCH, PDSCH) and which PUSCHs to receive. The PRBs it plans are reported through
 * dl_prbs()/ul_prbs() so that the LTE scheduler leaves them free.
 *
 * Timing (FDD, TS 36.213): the PDSCH of an MPDCCH ending in subframe n is sent in n+2 (7.1.11), the PUSCH of a DCI
 * 6-0A in n+4 (8.0), Msg3 six subframes after the RAR PDSCH (6.1.1). The Type2-CSS starts in even subframes
 * (T = Rmax * G = 2) and the MPDCCH is dropped where SIB1-BR/SI use the narrowband (9.1.5).
 */
class sched_emtc final : public sched_interface
{
public:
  sched_emtc();
  ~sched_emtc() override;

  /// LTE-M on this cell; without it (or before) the scheduler is off
  void set_cell(std::shared_ptr<const emtc::config> cfg,
                std::shared_ptr<const emtc::bcast>  bcast,
                const cell_cfg_t&                   cell_cfg);
  bool enabled() const { return cfg != nullptr; }
  bool is_br_preamble(uint32_t preamble) const;

  /// Takes over rnti before the MAC configures it (on a BR preamble)
  void add_rnti(uint16_t rnti);
  bool has_rnti(uint16_t rnti);

  /// PRBs LTE-M takes in the DL / UL subframe with this TTI (the broadcast and what was planned)
  uint64_t dl_prbs(uint32_t tti_tx_dl);
  uint64_t ul_prbs(uint32_t tti_tx_ul);

  /// PDSCH to a UE whose MAC PDU the MAC has to build (at the PDSCH TTI)
  struct ue_pdsch {
    uint32_t                                                           idx; ///< into dl_sched_t::emtc_pdsch
    uint16_t                                                           rnti;
    uint32_t                                                           pid;
    uint32_t                                                           tbs_bytes;
    srsran::bounded_vector<sched_interface::dl_sched_pdu_t, sched_interface::MAX_RLC_PDU_LIST> pdu;
  };
  /// What goes on the narrowbands in this DL TTI
  void get_dl(uint32_t tti_tx_dl, mac_interface_phy_lte::dl_sched_t& res, std::vector<ue_pdsch>& ue_pdus);

  /// PUSCH to receive in this UL TTI (no PDCCH: the grant went on MPDCCH or in the RAR)
  struct ul_grant {
    uint16_t        rnti;
    srsran_dci_ul_t dci;
    uint32_t        tbs_bytes;
    uint32_t        pid;
    uint32_t        current_tx_nb;
  };
  void get_ul(uint32_t tti_tx_ul, std::vector<ul_grant>& grants);

  // sched_interface (per-UE calls that the MAC routes here for BL/CE UEs)
  int      cell_cfg(const std::vector<cell_cfg_t>& cell_cfg) override;
  int      reset() override;
  int      ue_cfg(uint16_t rnti, const ue_cfg_t& cfg) override;
  int      ue_rem(uint16_t rnti) override;
  bool     ue_exists(uint16_t rnti) override;
  int      bearer_ue_cfg(uint16_t rnti, uint32_t lc_id, const mac_lc_ch_cfg_t& cfg) override;
  int      bearer_ue_rem(uint16_t rnti, uint32_t lc_id) override;
  uint32_t get_ul_buffer(uint16_t rnti) override;
  uint32_t get_dl_buffer(uint16_t rnti) override;
  int      dl_rlc_buffer_state(uint16_t rnti, uint32_t lc_id, uint32_t tx_queue, uint32_t prio_tx_queue) override;
  int      dl_mac_buffer_state(uint16_t rnti, uint32_t ce_code, uint32_t nof_cmds) override;
  int      dl_ack_info(uint32_t tti, uint16_t rnti, uint32_t enb_cc_idx, uint32_t tb_idx, bool ack) override;
  int      dl_rach_info(uint32_t enb_cc_idx, dl_sched_rar_info_t rar_info) override;
  int      dl_ri_info(uint32_t tti, uint16_t rnti, uint32_t enb_cc_idx, uint32_t ri_value) override;
  int      dl_pmi_info(uint32_t tti, uint16_t rnti, uint32_t enb_cc_idx, uint32_t pmi_value) override;
  int      dl_cqi_info(uint32_t tti, uint16_t rnti, uint32_t enb_cc_idx, uint32_t cqi_value) override;
  int      dl_sb_cqi_info(uint32_t tti, uint16_t rnti, uint32_t enb_cc_idx, uint32_t sb_idx, uint32_t cqi) override;
  int      ul_crc_info(uint32_t tti, uint16_t rnti, uint32_t enb_cc_idx, bool crc) override;
  int      ul_sr_info(uint32_t tti, uint16_t rnti) override;
  int      ul_bsr(uint16_t rnti, uint32_t lcg_id, uint32_t bsr) override;
  int      ul_phr(uint16_t rnti, int phr, uint32_t ul_nof_prb) override;
  int      ul_snr_info(uint32_t tti, uint16_t rnti, uint32_t enb_cc_idx, float snr, uint32_t ul_ch_code) override;
  int      dl_sched(uint32_t tti, uint32_t enb_cc_idx, dl_sched_res_t& sched_result) override;
  int      ul_sched(uint32_t tti, uint32_t enb_cc_idx, ul_sched_res_t& sched_result) override;
  int      set_pdcch_order(uint32_t enb_cc_idx, dl_sched_po_info_t pdcch_order_info) override;
  void     set_dl_tti_mask(uint8_t* tti_mask, uint32_t nof_sfs) override {}
  std::array<int, SRSRAN_MAX_CARRIERS> get_enb_ue_cc_map(uint16_t rnti) override;
  std::array<int, SRSRAN_MAX_CARRIERS> get_enb_ue_activ_cc_map(uint16_t rnti) override;
  int                                  ul_buffer_add(uint16_t rnti, uint32_t lcid, uint32_t bytes) override;

private:
  static const uint32_t NOF_TTI = 10240;
  static uint32_t       tti_add(uint32_t tti, int n) { return (uint32_t)((int)tti + n + (int)NOF_TTI) % NOF_TTI; }
  /// b - a in [-NOF_TTI/2, NOF_TTI/2)
  static int tti_diff(uint32_t b, uint32_t a)
  {
    int d = ((int)b - (int)a + (int)NOF_TTI) % (int)NOF_TTI;
    return d >= (int)NOF_TTI / 2 ? d - (int)NOF_TTI : d;
  }

  struct pdsch_plan {
    mac_interface_phy_lte::emtc_pdsch_t phy = {};
    bool                                rar = false;
    ue_pdsch                            ue  = {};
  };
  struct dl_plan {
    std::vector<mac_interface_phy_lte::emtc_mpdcch_t> mpdcch;
    std::vector<pdsch_plan>                           pdsch;
    uint32_t                                          nbs = 0; ///< narrowbands taken (bitmask)
  };
  /// One DL HARQ process per UE (stop and wait)
  struct dl_harq {
    bool                                                                active  = false;
    bool                                                                waiting = false; ///< for the HARQ-ACK
    uint32_t                                                            pid     = 0;
    uint32_t                                                            ack_tti = 0;
    uint32_t                                                            i_tbs   = 0;
    uint32_t                                                            tbs     = 0; ///< bits
    bool                                                                ndi     = false;
    uint32_t                                                            nof_tx  = 0;
  };
  /// One UL HARQ process per UE; CE mode A has no PHICH, retransmissions are granted on MPDCCH
  struct ul_harq {
    bool     active    = false;
    bool     waiting   = false; ///< for the PUSCH CRC
    uint32_t pid       = 0;
    uint32_t pusch_tti = 0;
    uint32_t i_mcs     = 0;
    uint32_t tbs       = 0; ///< bits
    bool     ndi       = false;
    uint32_t nof_tx    = 0;
  };
  struct ue_ctxt {
    bool                                             configured = false;
    bool                                             conres     = false; ///< Contention Resolution CE pending
    bool                                             connected  = false; ///< Msg4 acknowledged
    uint32_t                                         nb         = 0;     ///< MPDCCH/PDSCH narrowband
    std::array<uint32_t, SRSRAN_N_RADIO_BEARERS>     dl_bytes   = {};
    dl_harq                                          dlh;
    bool                                             sr         = false;
    std::array<uint32_t, 4>                          ul_lcg     = {}; ///< BSR per LCG (bytes)
    ul_harq                                          ulh;
    srsran_pucch_cfg_t                               pucch      = {};
    srsran_cqi_report_cfg_t                          cqi        = {};
    /// Half-duplex FDD (type B, TS 36.211 6.2.5): subframes the UE receives in / transmits in
    std::bitset<NOF_TTI>                             rx_busy;
    std::bitset<NOF_TTI>                             tx_busy;
  };

  bool     nb_free_dl(uint32_t tti, uint32_t nb);
  static bool ue_uci_at(const ue_ctxt& ue, uint32_t tti);
  static bool ue_tx_at(const ue_ctxt& ue, uint32_t tti);
  static bool ue_can_rx(const ue_ctxt& ue, uint32_t tti);
  static bool ue_can_tx(const ue_ctxt& ue, uint32_t tti);
  bool     plan_rar(const std::vector<dl_sched_rar_info_t>& ras);
  void     plan_dl(uint32_t tti_tx_dl);
  bool     plan_ue_dl(uint16_t rnti, ue_ctxt& ue, uint32_t tti_tx_dl);
  bool     plan_ue_ul(uint16_t rnti, ue_ctxt& ue, uint32_t tti_tx_dl);
  uint32_t pucch_prb(uint32_t n_pucch, uint32_t tti) const;
  void     gc(uint32_t tti_tx_dl);
  uint16_t ra_rnti(uint32_t prach_tti) const;

  srslog::basic_logger&               logger;
  std::mutex                          mutex;
  std::shared_ptr<const emtc::config> cfg;
  std::shared_ptr<const emtc::bcast>  bc;
  srsran_cell_t                       cell = {};
  uint32_t                            pucch_delta_shift = 1;
  uint32_t                            pucch_nrb_cqi     = 1;
  uint32_t                            pucch_ncs_an      = 0;
  bool                                last_dl_valid = false;
  uint32_t                            last_dl_tti   = 0;

  std::map<uint32_t, dl_plan>               dl;
  std::map<uint32_t, std::vector<ul_grant>> ul;
  std::map<uint32_t, uint64_t>              ul_resv;
  std::map<uint16_t, ue_ctxt>               ues;

  // RAR MAC PDUs (built at planning time, sent two TTIs later)
  static const uint32_t                     NOF_RAR_BUF = 8;
  std::array<std::array<uint8_t, 64>, NOF_RAR_BUF>    rar_buf     = {};
  std::array<srsran_softbuffer_tx_t, NOF_RAR_BUF>     rar_sb      = {};
  uint32_t                                            rar_buf_idx = 0;
  bool                                                rar_sb_init = false;
};

} // namespace srsenb

#endif // SRSENB_SCHED_EMTC_H
