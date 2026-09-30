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

#ifndef SRSENB_RRC_NBIOT_H
#define SRSENB_RRC_NBIOT_H

#include "srsenb/hdr/stack/rrc/rrc_bearer_cfg.h"
#include "srsenb/hdr/stack/rrc/rrc_config.h"
#include "srsenb/hdr/stack/upper/pdcp.h"
#include "srsenb/hdr/stack/upper/rlc.h"
#include "srsran/asn1/rrc_nbiot.h"
#include "srsran/common/bearer_manager.h"
#include "srsran/interfaces/enb_mac_interfaces.h"
#include "srsran/common/task_scheduler.h"
#include "srsran/interfaces/enb_gtpu_interfaces.h"
#include "srsran/interfaces/enb_nbiot_interfaces.h"
#include "srsran/interfaces/enb_rrc_interface_pdcp.h"
#include "srsran/interfaces/enb_rrc_interface_rlc.h"
#include "srsran/interfaces/enb_rrc_interface_s1ap.h"
#include "srsran/interfaces/enb_s1ap_interfaces.h"
#include <map>
#include <mutex>

namespace srsenb {

/**
 * RRC of the NB-IoT carrier (TS 36.331 for NB-IoT) for UEs in RRC_CONNECTED, with its own RLC and PDCP.
 *
 * The MAC (in the PHY) sets up the connection with Msg4 and hands over MAC SDUs. SRB1bis (LCID 3, RLC AM, no PDCP)
 * carries RRCConnectionSetupComplete-NB and NAS until the AS security is on; SRB1 (LCID 1) after. One DRB (LCID 5)
 * carries user data through GTP-U. Towards S1AP it is the RRC of every NB-IoT RNTI (see rrc_s1ap_mux).
 *
 * Everything runs in the stack thread except the methods of nbiot_rrc_interface_mac, which may be called from any.
 */
class rrc_nbiot final : public nbiot_rrc_interface_mac,
                        public rrc_interface_s1ap,
                        public rrc_interface_rlc,
                        public rrc_interface_pdcp,
                        public mac_interface_rlc
{
public:
  rrc_nbiot(srsran::task_sched_handle task_sched_, srslog::basic_logger& logger_, srslog::basic_logger& rlc_logger,
            srslog::basic_logger& pdcp_logger);
  ~rrc_nbiot() override;

  void init(const rrc_cfg_t&       cfg,
            s1ap_interface_rrc*    s1ap,
            gtpu_interface_rrc*    gtpu,
            gtpu_interface_pdcp*   gtpu_pdcp,
            enb_bearer_manager*    bearers,
            srsran::timer_handler* timers);
  void stop();

  /// Stack thread
  void set_mac(nbiot_mac_interface_rrc* mac_);

  pdcp_interface_gtpu* get_pdcp_gtpu() { return &pdcp; }

  // nbiot_rrc_interface_mac
  void     ue_connected(uint16_t rnti) override;
  void     ue_lost(uint16_t rnti) override;
  void     write_pdu(uint16_t rnti, uint32_t lcid, const uint8_t* payload, uint32_t nof_bytes) override;
  int      read_pdu(uint16_t rnti, uint32_t lcid, uint8_t* payload, uint32_t nof_bytes) override;
  uint32_t dl_buffer(uint16_t rnti, uint32_t lcid) override;

  // mac_interface_rlc
  int rlc_buffer_state(uint16_t rnti, uint32_t lc_id, uint32_t tx_queue, uint32_t retx_queue) override;

  // rrc_interface_rlc, rrc_interface_pdcp
  void max_retx_attempted(uint16_t rnti) override;
  void protocol_failure(uint16_t rnti) override;
  void write_pdu(uint16_t rnti, uint32_t lcid, srsran::unique_byte_buffer_t pdu) override;
  void notify_pdcp_integrity_error(uint16_t rnti, uint32_t lcid) override;

  // rrc_interface_s1ap
  void write_dl_info(uint16_t rnti, srsran::unique_byte_buffer_t sdu) override;
  void release_ue(uint16_t rnti) override;
  bool setup_ue_ctxt(uint16_t rnti, const asn1::s1ap::init_context_setup_request_s& msg) override;
  bool modify_ue_ctxt(uint16_t rnti, const asn1::s1ap::ue_context_mod_request_s& msg) override;
  bool has_erab(uint16_t rnti, uint32_t erab_id) const override;
  bool release_erabs(uint32_t rnti) override;
  int  get_erab_addr_in(uint16_t rnti, uint16_t erab_id, transp_addr_t& addr_in, uint32_t& teid_in) const override;
  void set_aggregate_max_bitrate(uint16_t rnti, const asn1::s1ap::ue_aggregate_maximum_bitrate_s& bitrate) override {}
  int  setup_erab(uint16_t                                   rnti,
                  uint16_t                                   erab_id,
                  const asn1::s1ap::erab_level_qos_params_s& qos_params,
                  srsran::const_span<uint8_t>                nas_pdu,
                  const transp_addr_t&                       addr,
                  uint32_t                                   gtpu_teid_out,
                  asn1::s1ap::cause_c&                       cause) override;
  int  modify_erab(uint16_t                                   rnti,
                   uint16_t                                   erab_id,
                   const asn1::s1ap::erab_level_qos_params_s& qos_params,
                   srsran::const_span<uint8_t>                nas_pdu,
                   asn1::s1ap::cause_c&                       cause) override;
  int  release_erab(uint16_t rnti, uint16_t erab_id) override;
  void add_paging_id(uint32_t ueid, const asn1::s1ap::ue_paging_id_c& ue_paging_id) override {}
  int  notify_ue_erab_updates(uint16_t rnti, srsran::const_span<uint8_t> nas_pdu) override;
  void ho_preparation_complete(uint16_t                     rnti,
                               ho_prep_result               result,
                               const asn1::s1ap::ho_cmd_s&  msg,
                               srsran::unique_byte_buffer_t container) override
  {}
  uint16_t start_ho_ue_resource_alloc(const asn1::s1ap::ho_request_s&                                   msg,
                                      const asn1::s1ap::sourceenb_to_targetenb_transparent_container_s& container,
                                      asn1::s1ap::cause_c& failure_cause) override
  {
    return SRSRAN_INVALID_RNTI;
  }
  void set_erab_status(uint16_t rnti, const asn1::s1ap::bearers_subject_to_status_transfer_list_l& erabs) override {}

private:
  /// RLC delivers SRB1bis straight to RRC, everything else to PDCP
  class pdcp_bypass final : public pdcp_interface_rlc
  {
  public:
    explicit pdcp_bypass(rrc_nbiot* parent_) : parent(parent_) {}
    void write_pdu(uint16_t rnti, uint32_t lcid, srsran::unique_byte_buffer_t pdu) override;
    void notify_delivery(uint16_t rnti, uint32_t lcid, const srsran::pdcp_sn_vector_t& sns) override;
    void notify_failure(uint16_t rnti, uint32_t lcid, const srsran::pdcp_sn_vector_t& sns) override;

  private:
    rrc_nbiot* parent;
  };

  struct erab_t {
    uint16_t      id       = 0;
    transp_addr_t address;
    uint32_t      teid_out = 0;
    uint32_t      teid_in  = 0;
    std::vector<uint8_t> nas;
  };

  enum class ue_state { setup, wait_security, wait_reconf, connected, releasing };

  struct ue_t {
    explicit ue_t(const rrc_cfg_t& cfg) : sec(cfg) {}
    uint16_t                   rnti  = 0;
    ue_state                   state = ue_state::setup;
    bool                       s1ap_known = false;
    bool                       security   = false; ///< SRB1 in use
    uint8_t                    transaction_id = 0;
    security_cfg_handler       sec;
    std::map<uint16_t, erab_t> erabs;
    srsran::unique_timer       release_timer;
  };

  srsran::task_sched_handle task_sched;
  srslog::basic_logger&     logger;
  srsran::task_queue_handle queue;
  rrc_cfg_t                 cfg;
  srsenb::rlc               rlc;
  srsenb::pdcp              pdcp;
  pdcp_bypass               bypass;
  s1ap_interface_rrc*       s1ap    = nullptr;
  gtpu_interface_rrc*       gtpu    = nullptr;
  enb_bearer_manager*       bearers = nullptr;
  nbiot_mac_interface_rrc*  mac     = nullptr;
  bool                      running = false;

  std::map<uint16_t, std::unique_ptr<ue_t> > users;

  std::mutex                               buffer_lock;
  std::map<std::pair<uint16_t, uint32_t>, uint32_t> buffers;

  ue_t* find(uint16_t rnti);
  void  add_ue(uint16_t rnti);
  void  remove_ue(uint16_t rnti);
  void  handle_ul_dcch(uint16_t rnti, uint32_t lcid, srsran::unique_byte_buffer_t pdu);
  void  send_dl_dcch(ue_t& ue, const asn1::rrc::dl_dcch_msg_nb_s& msg, const char* what);
  void  send_security_mode_command(ue_t& ue);
  void  send_reconfiguration(ue_t& ue);
  void  send_release(ue_t& ue);
};

/// S1AP talks to one RRC; this one passes NB-IoT RNTIs to rrc_nbiot and the rest to the LTE RRC
class rrc_s1ap_mux final : public rrc_interface_s1ap
{
public:
  rrc_s1ap_mux(rrc_interface_s1ap* lte_, rrc_interface_s1ap* nbiot_) : lte(lte_), nbiot(nbiot_) {}

  void write_dl_info(uint16_t rnti, srsran::unique_byte_buffer_t sdu) override
  {
    pick(rnti)->write_dl_info(rnti, std::move(sdu));
  }
  void release_ue(uint16_t rnti) override { pick(rnti)->release_ue(rnti); }
  bool setup_ue_ctxt(uint16_t rnti, const asn1::s1ap::init_context_setup_request_s& msg) override
  {
    return pick(rnti)->setup_ue_ctxt(rnti, msg);
  }
  bool modify_ue_ctxt(uint16_t rnti, const asn1::s1ap::ue_context_mod_request_s& msg) override
  {
    return pick(rnti)->modify_ue_ctxt(rnti, msg);
  }
  bool has_erab(uint16_t rnti, uint32_t erab_id) const override { return pick(rnti)->has_erab(rnti, erab_id); }
  bool release_erabs(uint32_t rnti) override { return pick((uint16_t)rnti)->release_erabs(rnti); }
  int  get_erab_addr_in(uint16_t rnti, uint16_t erab_id, transp_addr_t& addr_in, uint32_t& teid_in) const override
  {
    return pick(rnti)->get_erab_addr_in(rnti, erab_id, addr_in, teid_in);
  }
  void set_aggregate_max_bitrate(uint16_t rnti, const asn1::s1ap::ue_aggregate_maximum_bitrate_s& bitrate) override
  {
    pick(rnti)->set_aggregate_max_bitrate(rnti, bitrate);
  }
  int setup_erab(uint16_t                                   rnti,
                 uint16_t                                   erab_id,
                 const asn1::s1ap::erab_level_qos_params_s& qos_params,
                 srsran::const_span<uint8_t>                nas_pdu,
                 const transp_addr_t&                       addr,
                 uint32_t                                   gtpu_teid_out,
                 asn1::s1ap::cause_c&                       cause) override
  {
    return pick(rnti)->setup_erab(rnti, erab_id, qos_params, nas_pdu, addr, gtpu_teid_out, cause);
  }
  int modify_erab(uint16_t                                   rnti,
                  uint16_t                                   erab_id,
                  const asn1::s1ap::erab_level_qos_params_s& qos_params,
                  srsran::const_span<uint8_t>                nas_pdu,
                  asn1::s1ap::cause_c&                       cause) override
  {
    return pick(rnti)->modify_erab(rnti, erab_id, qos_params, nas_pdu, cause);
  }
  int  release_erab(uint16_t rnti, uint16_t erab_id) override { return pick(rnti)->release_erab(rnti, erab_id); }
  void add_paging_id(uint32_t ueid, const asn1::s1ap::ue_paging_id_c& ue_paging_id) override
  {
    lte->add_paging_id(ueid, ue_paging_id);
  }
  int notify_ue_erab_updates(uint16_t rnti, srsran::const_span<uint8_t> nas_pdu) override
  {
    return pick(rnti)->notify_ue_erab_updates(rnti, nas_pdu);
  }
  void ho_preparation_complete(uint16_t                     rnti,
                               ho_prep_result               result,
                               const asn1::s1ap::ho_cmd_s&  msg,
                               srsran::unique_byte_buffer_t container) override
  {
    pick(rnti)->ho_preparation_complete(rnti, result, msg, std::move(container));
  }
  uint16_t start_ho_ue_resource_alloc(const asn1::s1ap::ho_request_s&                                   msg,
                                      const asn1::s1ap::sourceenb_to_targetenb_transparent_container_s& container,
                                      asn1::s1ap::cause_c& failure_cause) override
  {
    return lte->start_ho_ue_resource_alloc(msg, container, failure_cause);
  }
  void set_erab_status(uint16_t rnti, const asn1::s1ap::bearers_subject_to_status_transfer_list_l& erabs) override
  {
    pick(rnti)->set_erab_status(rnti, erabs);
  }

private:
  rrc_interface_s1ap* lte;
  rrc_interface_s1ap* nbiot;
  rrc_interface_s1ap* pick(uint16_t rnti) const { return nbiot != nullptr && is_nbiot_rnti(rnti) ? nbiot : lte; }
};

} // namespace srsenb

#endif // SRSENB_RRC_NBIOT_H
