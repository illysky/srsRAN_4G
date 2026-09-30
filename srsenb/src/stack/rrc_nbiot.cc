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

#include "srsenb/hdr/stack/rrc_nbiot.h"
#include "srsran/asn1/rrc_nbiot.h"
#include "srsran/asn1/rrc_utils.h"
#include "srsran/common/standard_streams.h"

namespace srsenb {

namespace {

constexpr uint32_t RELEASE_DELAY_MS = 3000; ///< time the RRCConnectionRelease-NB gets to go out before the UE is removed

/// Our side of RLC AM (the UE's side is the default of 36.331 9.2.1.1 for SRBs, and what the DRB says)
srsran::rlc_config_t rlc_am_cfg()
{
  srsran::rlc_config_t c   = srsran::rlc_config_t::default_rlc_am_config();
  c.am.t_poll_retx         = 500;
  c.am.poll_pdu            = -1;
  c.am.poll_byte           = -1;
  c.am.max_retx_thresh     = 8;
  c.am.t_reordering        = 100;
  c.am.t_status_prohibit   = 0;
  return c;
}

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

template <class M>
std::string to_json(const M& m)
{
  asn1::json_writer js;
  m.to_json(js);
  return js.to_string();
}

} // namespace

rrc_nbiot::rrc_nbiot(srsran::task_sched_handle task_sched_,
                     srslog::basic_logger&     logger_,
                     srslog::basic_logger&     rlc_logger,
                     srslog::basic_logger&     pdcp_logger) :
  task_sched(task_sched_),
  logger(logger_),
  rlc(rlc_logger),
  pdcp(task_sched_, pdcp_logger),
  bypass(this)
{}

rrc_nbiot::~rrc_nbiot()
{
  stop();
}

void rrc_nbiot::init(const rrc_cfg_t&       cfg_,
                     s1ap_interface_rrc*    s1ap_,
                     gtpu_interface_rrc*    gtpu_,
                     gtpu_interface_pdcp*   gtpu_pdcp,
                     enb_bearer_manager*    bearers_,
                     srsran::timer_handler* timers)
{
  cfg     = cfg_;
  s1ap    = s1ap_;
  gtpu    = gtpu_;
  bearers = bearers_;
  // No ciphering on the NB-IoT carrier: user plane and signalling stay readable in the traces; integrity as configured
  cfg.eea_preference_list[0] = srsran::CIPHERING_ALGORITHM_ID_EEA0;
  queue                      = task_sched.make_task_queue();
  rlc.init(&bypass, this, this, timers);
  pdcp.init(&rlc, this, gtpu_pdcp);
  running = true;
}

void rrc_nbiot::stop()
{
  if (running) {
    running = false;
    rlc.stop();
    pdcp.stop();
    users.clear();
  }
}

void rrc_nbiot::set_mac(nbiot_mac_interface_rrc* mac_)
{
  mac = mac_;
  if (mac != nullptr) {
    mac->set_rrc(this);
  }
}

rrc_nbiot::ue_t* rrc_nbiot::find(uint16_t rnti)
{
  auto it = users.find(rnti);
  return it == users.end() ? nullptr : it->second.get();
}

/* ------------------------------------------------------------------------------------------ from the MAC (any thread) */

void rrc_nbiot::ue_connected(uint16_t rnti)
{
  queue.push([this, rnti]() { add_ue(rnti); });
}

void rrc_nbiot::ue_lost(uint16_t rnti)
{
  queue.push([this, rnti]() {
    ue_t* ue = find(rnti);
    if (ue == nullptr) {
      return;
    }
    if (ue->s1ap_known && s1ap->user_exists(rnti)) {
      s1ap->user_release(rnti, asn1::s1ap::cause_radio_network_opts::radio_conn_with_ue_lost);
    } else {
      remove_ue(rnti);
    }
  });
}

void rrc_nbiot::write_pdu(uint16_t rnti, uint32_t lcid, const uint8_t* payload, uint32_t nof_bytes)
{
  std::vector<uint8_t> copy(payload, payload + nof_bytes);
  queue.push([this, rnti, lcid, copy]() mutable {
    if (find(rnti) == nullptr) {
      logger.warning("NB-IoT: uplink for unknown rnti=0x%x", rnti);
      return;
    }
    rlc.write_pdu(rnti, lcid, copy.data(), (uint32_t)copy.size());
  });
}

int rrc_nbiot::read_pdu(uint16_t rnti, uint32_t lcid, uint8_t* payload, uint32_t nof_bytes)
{
  return rlc.read_pdu(rnti, lcid, payload, nof_bytes);
}

uint32_t rrc_nbiot::dl_buffer(uint16_t rnti, uint32_t lcid)
{
  std::lock_guard<std::mutex> guard(buffer_lock);
  auto                        it = buffers.find({rnti, lcid});
  return it == buffers.end() ? 0 : it->second;
}

int rrc_nbiot::rlc_buffer_state(uint16_t rnti, uint32_t lc_id, uint32_t tx_queue, uint32_t retx_queue)
{
  std::lock_guard<std::mutex> guard(buffer_lock);
  buffers[{rnti, lc_id}] = tx_queue + retx_queue;
  return SRSRAN_SUCCESS;
}

/* --------------------------------------------------------------------------------------------------- UE lifetime */

void rrc_nbiot::add_ue(uint16_t rnti)
{
  if (find(rnti) != nullptr) {
    remove_ue(rnti);
  }
  std::unique_ptr<ue_t> ue(new ue_t(cfg));
  ue->rnti          = rnti;
  ue->release_timer = task_sched.get_unique_timer();
  users[rnti]       = std::move(ue);

  // What RRCConnectionSetup-NB gave the UE: SRB1bis and SRB1 with the default RLC AM configuration
  rlc.add_user(rnti);
  rlc.add_bearer(rnti, NBIOT_LCID_SRB1BIS, rlc_am_cfg());
  rlc.add_bearer(rnti, NBIOT_LCID_SRB1, rlc_am_cfg());
  pdcp.add_user(rnti);
  pdcp.add_bearer(rnti, NBIOT_LCID_SRB1, srsran::make_srb_pdcp_config_t(1, false));
  srsran::console("NB-IoT RRC: UE 0x%04x connected (SRB1bis, SRB1)\n", rnti);
  logger.info("NB-IoT: rnti=0x%x in RRC_CONNECTED", rnti);
}

void rrc_nbiot::remove_ue(uint16_t rnti)
{
  if (find(rnti) == nullptr) {
    return;
  }
  if (mac != nullptr) {
    mac->release_ue(rnti);
  }
  gtpu->rem_user(rnti);
  bearers->rem_user(rnti);
  pdcp.rem_user(rnti);
  rlc.rem_user(rnti);
  {
    std::lock_guard<std::mutex> guard(buffer_lock);
    for (auto it = buffers.begin(); it != buffers.end();) {
      it = it->first.first == rnti ? buffers.erase(it) : std::next(it);
    }
  }
  users.erase(rnti);
  srsran::console("NB-IoT RRC: UE 0x%04x removed\n", rnti);
}

/* ------------------------------------------------------------------------------------------------- RLC / PDCP side */

void rrc_nbiot::pdcp_bypass::write_pdu(uint16_t rnti, uint32_t lcid, srsran::unique_byte_buffer_t pdu)
{
  if (lcid == NBIOT_LCID_SRB1BIS) {
    parent->handle_ul_dcch(rnti, lcid, std::move(pdu));
  } else {
    parent->pdcp.write_pdu(rnti, lcid, std::move(pdu));
  }
}

void rrc_nbiot::pdcp_bypass::notify_delivery(uint16_t rnti, uint32_t lcid, const srsran::pdcp_sn_vector_t& sns)
{
  if (lcid != NBIOT_LCID_SRB1BIS) {
    parent->pdcp.notify_delivery(rnti, lcid, sns);
  }
}

void rrc_nbiot::pdcp_bypass::notify_failure(uint16_t rnti, uint32_t lcid, const srsran::pdcp_sn_vector_t& sns)
{
  if (lcid != NBIOT_LCID_SRB1BIS) {
    parent->pdcp.notify_failure(rnti, lcid, sns);
  }
}

void rrc_nbiot::write_pdu(uint16_t rnti, uint32_t lcid, srsran::unique_byte_buffer_t pdu)
{
  handle_ul_dcch(rnti, lcid, std::move(pdu));
}

void rrc_nbiot::max_retx_attempted(uint16_t rnti)
{
  srsran::console("NB-IoT RRC: UE 0x%04x: RLC reached the maximum number of retransmissions\n", rnti);
}

void rrc_nbiot::protocol_failure(uint16_t rnti)
{
  srsran::console("NB-IoT RRC: UE 0x%04x: RLC protocol failure\n", rnti);
}

void rrc_nbiot::notify_pdcp_integrity_error(uint16_t rnti, uint32_t lcid)
{
  srsran::console("NB-IoT RRC: UE 0x%04x: integrity check failed on LCID %u\n", rnti, lcid);
}

/* --------------------------------------------------------------------------------------------------------- RRC */

void rrc_nbiot::send_dl_dcch(ue_t& ue, const asn1::rrc::dl_dcch_msg_nb_s& msg, const char* what)
{
  srsran::unique_byte_buffer_t buf = srsran::make_byte_buffer();
  if (buf == nullptr) {
    return;
  }
  asn1::bit_ref bref(buf->msg, buf->get_tailroom());
  if (msg.pack(bref) != asn1::SRSASN_SUCCESS) {
    srsran::console("NB-IoT RRC: cannot pack %s\n", what);
    return;
  }
  buf->N_bytes = (uint32_t)bref.distance_bytes();
  srsran::console("NB-IoT RRC: 0x%04x <- %s (%u bytes on %s)\n",
                  ue.rnti,
                  what,
                  buf->N_bytes,
                  ue.security ? "SRB1" : "SRB1bis");
  logger.info("NB-IoT DL-DCCH rnti=0x%x %s: %s", ue.rnti, what, to_json(msg).c_str());
  if (ue.security) {
    pdcp.write_sdu(ue.rnti, NBIOT_LCID_SRB1, std::move(buf));
  } else {
    rlc.write_sdu(ue.rnti, NBIOT_LCID_SRB1BIS, std::move(buf));
  }
}

void rrc_nbiot::handle_ul_dcch(uint16_t rnti, uint32_t lcid, srsran::unique_byte_buffer_t pdu)
{
  ue_t* ue = find(rnti);
  if (ue == nullptr || pdu == nullptr) {
    return;
  }
  asn1::rrc::ul_dcch_msg_nb_s msg;
  asn1::cbit_ref              bref(pdu->msg, pdu->N_bytes);
  if (msg.unpack(bref) != asn1::SRSASN_SUCCESS ||
      msg.msg.type().value != asn1::rrc::ul_dcch_msg_type_nb_c::types_opts::c1) {
    srsran::console("NB-IoT RRC: 0x%04x: LCID %u PDU does not decode as UL-DCCH-Message-NB: %s\n",
                    rnti,
                    lcid,
                    hex(pdu->msg, pdu->N_bytes).c_str());
    return;
  }
  logger.info("NB-IoT UL-DCCH rnti=0x%x: %s", rnti, to_json(msg).c_str());
  auto& c1 = msg.msg.c1();
  srsran::console("NB-IoT RRC: 0x%04x -> %s (%u bytes on LCID %u)\n", rnti, c1.type().to_string(), pdu->N_bytes, lcid);

  using t = asn1::rrc::ul_dcch_msg_type_nb_c::c1_c_::types_opts;
  switch (c1.type().value) {
    case t::rrc_conn_setup_complete_r13: {
      auto& ies = c1.rrc_conn_setup_complete_r13().crit_exts.rrc_conn_setup_complete_r13();
      srsran::unique_byte_buffer_t nas = srsran::make_byte_buffer();
      if (nas == nullptr || ies.ded_info_nas_r13.size() > nas->get_tailroom()) {
        return;
      }
      memcpy(nas->msg, ies.ded_info_nas_r13.data(), ies.ded_info_nas_r13.size());
      nas->N_bytes   = (uint32_t)ies.ded_info_nas_r13.size();
      ue->s1ap_known = true;
      const auto cause = asn1::s1ap::rrc_establishment_cause_opts::mo_sig;
      if (ies.s_tmsi_r13_present) {
        s1ap->initial_ue(rnti,
                         0,
                         cause,
                         std::move(nas),
                         (uint32_t)ies.s_tmsi_r13.m_tmsi.to_number(),
                         (uint8_t)ies.s_tmsi_r13.mmec.to_number());
      } else {
        s1ap->initial_ue(rnti, 0, cause, std::move(nas));
      }
      srsran::console("NB-IoT RRC: 0x%04x: InitialUEMessage to the MME\n", rnti);
      break;
    }
    case t::ul_info_transfer_r13: {
      auto& ies = c1.ul_info_transfer_r13().crit_exts.ul_info_transfer_r13();
      srsran::unique_byte_buffer_t nas = srsran::make_byte_buffer();
      if (nas == nullptr || ies.ded_info_nas_r13.size() > nas->get_tailroom()) {
        return;
      }
      memcpy(nas->msg, ies.ded_info_nas_r13.data(), ies.ded_info_nas_r13.size());
      nas->N_bytes = (uint32_t)ies.ded_info_nas_r13.size();
      s1ap->write_pdu(rnti, std::move(nas));
      break;
    }
    case t::security_mode_complete_r13:
      pdcp.enable_encryption(rnti, NBIOT_LCID_SRB1);
      send_reconfiguration(*ue);
      break;
    case t::security_mode_fail_r13:
      srsran::console("NB-IoT RRC: 0x%04x: SecurityModeFailure\n", rnti);
      break;
    case t::rrc_conn_recfg_complete_r13:
      if (ue->state == ue_state::wait_reconf) {
        ue->state = ue_state::connected;
        s1ap->notify_rrc_reconf_complete(rnti);
        srsran::console("NB-IoT RRC: 0x%04x: reconfiguration complete, context set up\n", rnti);
      }
      break;
    default:
      break;
  }
}

void rrc_nbiot::send_security_mode_command(ue_t& ue)
{
  pdcp.config_security(ue.rnti, NBIOT_LCID_SRB1, ue.sec.get_as_sec_cfg());
  pdcp.enable_integrity(ue.rnti, NBIOT_LCID_SRB1);
  ue.security = true;
  ue.state    = ue_state::wait_security;

  asn1::rrc::dl_dcch_msg_nb_s msg;
  auto&                       smc = msg.msg.set_c1().set_security_mode_cmd_r13();
  smc.rrc_transaction_id          = (uint8_t)(ue.transaction_id++ % 4);
  smc.crit_exts.set_c1().set_security_mode_cmd_r8().security_cfg_smc.security_algorithm_cfg =
      ue.sec.get_security_algorithm_cfg();
  send_dl_dcch(ue, msg, "SecurityModeCommand");
}

void rrc_nbiot::send_reconfiguration(ue_t& ue)
{
  asn1::rrc::dl_dcch_msg_nb_s msg;
  auto&                       recfg = msg.msg.set_c1().set_rrc_conn_recfg_r13();
  recfg.rrc_transaction_id          = (uint8_t)(ue.transaction_id++ % 4);
  auto& ies                         = recfg.crit_exts.set_c1().set_rrc_conn_recfg_r13();

  for (auto& kv : ue.erabs) {
    erab_t& e = kv.second;
    if (!e.nas.empty() && ies.ded_info_nas_list_r13.size() < ies.ded_info_nas_list_r13.capacity()) {
      ies.ded_info_nas_list_r13_present = true;
      asn1::dyn_octstring nas;
      nas.resize(e.nas.size());
      memcpy(nas.data(), e.nas.data(), e.nas.size());
      ies.ded_info_nas_list_r13.push_back(nas);
      e.nas.clear();
    }
    if (ies.rr_cfg_ded_r13.drb_to_add_mod_list_r13_present) {
      continue; // one DRB
    }
    ies.rr_cfg_ded_r13.drb_to_add_mod_list_r13_present = true;
    ies.rr_cfg_ded_r13.drb_to_add_mod_list_r13.resize(1);
    auto& drb                     = ies.rr_cfg_ded_r13.drb_to_add_mod_list_r13[0];
    drb.eps_bearer_id_r13_present = true;
    drb.eps_bearer_id_r13         = (uint8_t)e.id;
    drb.drb_id_r13                = 1;
    drb.pdcp_cfg_r13_present      = true;
    drb.pdcp_cfg_r13.discard_timer_r13_present = true;
    drb.pdcp_cfg_r13.discard_timer_r13.value =
        asn1::rrc::pdcp_cfg_nb_r13_s::discard_timer_r13_opts::infinity;
    drb.pdcp_cfg_r13.hdr_compress_r13.set_not_used();
    drb.rlc_cfg_r13_present = true;
    auto& am                = drb.rlc_cfg_r13.set_am();
    am.ul_am_rlc_r13.t_poll_retx_r13.value        = asn1::rrc::t_poll_retx_nb_r13_opts::ms2000;
    am.ul_am_rlc_r13.max_retx_thres_r13.value     = asn1::rrc::ul_am_rlc_nb_r13_s::max_retx_thres_r13_opts::t8;
    drb.lc_ch_id_r13_present                      = true;
    drb.lc_ch_id_r13                              = (uint8_t)NBIOT_LCID_DRB1;
    drb.lc_ch_cfg_r13_present                     = true;
    drb.lc_ch_cfg_r13.prio_r13_present            = true;
    drb.lc_ch_cfg_r13.prio_r13                    = 10;
    drb.lc_ch_cfg_r13.lc_ch_sr_prohibit_r13_present = false;
  }
  ue.state = ue_state::wait_reconf;
  send_dl_dcch(ue, msg, "RRCConnectionReconfiguration-NB");
}

void rrc_nbiot::send_release(ue_t& ue)
{
  asn1::rrc::dl_dcch_msg_nb_s msg;
  auto&                       rel = msg.msg.set_c1().set_rrc_conn_release_r13();
  rel.rrc_transaction_id          = (uint8_t)(ue.transaction_id++ % 4);
  rel.crit_exts.set_c1().set_rrc_conn_release_r13().release_cause_r13.value =
      asn1::rrc::release_cause_nb_r13_opts::other;
  send_dl_dcch(ue, msg, "RRCConnectionRelease-NB");
}

/* ---------------------------------------------------------------------------------------------------------- S1AP */

void rrc_nbiot::write_dl_info(uint16_t rnti, srsran::unique_byte_buffer_t sdu)
{
  ue_t* ue = find(rnti);
  if (ue == nullptr || sdu == nullptr) {
    return;
  }
  asn1::rrc::dl_dcch_msg_nb_s msg;
  auto& ies = msg.msg.set_c1().set_dl_info_transfer_r13().crit_exts.set_c1().set_dl_info_transfer_r13();
  ies.ded_info_nas_r13.resize(sdu->N_bytes);
  memcpy(ies.ded_info_nas_r13.data(), sdu->msg, sdu->N_bytes);
  send_dl_dcch(*ue, msg, "DLInformationTransfer-NB");
}

void rrc_nbiot::release_ue(uint16_t rnti)
{
  ue_t* ue = find(rnti);
  if (ue == nullptr) {
    return;
  }
  if (ue->state != ue_state::releasing) {
    send_release(*ue);
    ue->state = ue_state::releasing;
  }
  ue->release_timer.set(RELEASE_DELAY_MS, [this, rnti](uint32_t) { remove_ue(rnti); });
  ue->release_timer.run();
}

bool rrc_nbiot::setup_ue_ctxt(uint16_t rnti, const asn1::s1ap::init_context_setup_request_s& msg)
{
  ue_t* ue = find(rnti);
  if (ue == nullptr) {
    return false;
  }
  const bool algos = ue->sec.set_security_capabilities(msg->ue_security_cap.value);
  ue->sec.set_security_key(msg->security_key.value);
  srsran::console("NB-IoT RRC: 0x%04x: UE EEA %s EIA %s, preference EIA %d %d %d %d: %s EEA%d EIA%d\n",
                  rnti,
                  msg->ue_security_cap.value.encryption_algorithms.to_string().c_str(),
                  msg->ue_security_cap.value.integrity_protection_algorithms.to_string().c_str(),
                  (int)cfg.eia_preference_list[0],
                  (int)cfg.eia_preference_list[1],
                  (int)cfg.eia_preference_list[2],
                  (int)cfg.eia_preference_list[3],
                  algos ? "selected" : "NO MATCH",
                  (int)ue->sec.get_as_sec_cfg().cipher_algo,
                  (int)ue->sec.get_as_sec_cfg().integ_algo);
  srsran::console("NB-IoT RRC: 0x%04x: InitialContextSetupRequest, %zu E-RAB(s)\n",
                  rnti,
                  msg->erab_to_be_setup_list_ctxt_su_req.value.size());
  send_security_mode_command(*ue);
  return true;
}

bool rrc_nbiot::modify_ue_ctxt(uint16_t rnti, const asn1::s1ap::ue_context_mod_request_s& msg)
{
  return find(rnti) != nullptr;
}

bool rrc_nbiot::has_erab(uint16_t rnti, uint32_t erab_id) const
{
  auto it = users.find(rnti);
  return it != users.end() && it->second->erabs.count((uint16_t)erab_id) > 0;
}

bool rrc_nbiot::release_erabs(uint32_t rnti)
{
  ue_t* ue = find((uint16_t)rnti);
  if (ue == nullptr) {
    return false;
  }
  for (auto& kv : ue->erabs) {
    gtpu->rem_bearer((uint16_t)rnti, kv.first);
    bearers->remove_eps_bearer((uint16_t)rnti, (uint8_t)kv.first);
  }
  ue->erabs.clear();
  return true;
}

int rrc_nbiot::get_erab_addr_in(uint16_t rnti, uint16_t erab_id, transp_addr_t& addr_in, uint32_t& teid_in) const
{
  auto it = users.find(rnti);
  if (it == users.end()) {
    return SRSRAN_ERROR;
  }
  auto e = it->second->erabs.find(erab_id);
  if (e == it->second->erabs.end()) {
    return SRSRAN_ERROR;
  }
  addr_in = e->second.address;
  teid_in = e->second.teid_in;
  return SRSRAN_SUCCESS;
}

int rrc_nbiot::setup_erab(uint16_t                                   rnti,
                          uint16_t                                   erab_id,
                          const asn1::s1ap::erab_level_qos_params_s& qos_params,
                          srsran::const_span<uint8_t>                nas_pdu,
                          const transp_addr_t&                       addr,
                          uint32_t                                   gtpu_teid_out,
                          asn1::s1ap::cause_c&                       cause)
{
  ue_t* ue = find(rnti);
  if (ue == nullptr) {
    cause.set_radio_network().value = asn1::s1ap::cause_radio_network_opts::unknown_enb_ue_s1ap_id;
    return SRSRAN_ERROR;
  }
  if (!ue->erabs.empty()) {
    cause.set_radio_network().value = asn1::s1ap::cause_radio_network_opts::not_supported_qci_value;
    return SRSRAN_ERROR; // one DRB only
  }
  uint32_t v4 = 0;
  if (addr.length() == 32) {
    v4 = addr.to_number();
  } else if (addr.length() == 160) {
    v4 = asn1::bitstring_utils::to_number(addr.data() + 16, 32);
  } else {
    cause.set_transport().value = asn1::s1ap::cause_transport_opts::transport_res_unavailable;
    return SRSRAN_ERROR;
  }
  erab_t e;
  e.id       = erab_id;
  e.address  = addr;
  e.teid_out = gtpu_teid_out;
  e.nas.assign(nas_pdu.begin(), nas_pdu.end());

  uint32_t                   addr_in = 0;
  srsran::expected<uint32_t> teid_in = gtpu->add_bearer(rnti, erab_id, v4, gtpu_teid_out, addr_in);
  if (teid_in.is_error()) {
    cause.set_radio_network().value = asn1::s1ap::cause_radio_network_opts::unspecified;
    return SRSRAN_ERROR;
  }
  e.teid_in = teid_in.value();
  bearers->add_eps_bearer(rnti, (uint8_t)erab_id, srsran::srsran_rat_t::lte, NBIOT_LCID_DRB1);

  // The DRB as the reconfiguration will describe it: RLC AM, PDCP with 7-bit sequence numbers (36.323 6.2.3)
  rlc.add_bearer(rnti, NBIOT_LCID_DRB1, rlc_am_cfg());
  srsran::pdcp_config_t pc(1,
                           srsran::PDCP_RB_IS_DRB,
                           srsran::SECURITY_DIRECTION_DOWNLINK,
                           srsran::SECURITY_DIRECTION_UPLINK,
                           srsran::PDCP_SN_LEN_7,
                           srsran::pdcp_t_reordering_t::ms500,
                           srsran::pdcp_discard_timer_t::infinity,
                           false,
                           srsran::srsran_rat_t::lte);
  pdcp.add_bearer(rnti, NBIOT_LCID_DRB1, pc);
  pdcp.config_security(rnti, NBIOT_LCID_DRB1, ue->sec.get_as_sec_cfg());
  pdcp.enable_encryption(rnti, NBIOT_LCID_DRB1);

  srsran::console("NB-IoT RRC: 0x%04x: E-RAB %u (QCI %u) -> DRB1 on LCID %u, TEID out 0x%x in 0x%x, NAS %zu bytes\n",
                  rnti,
                  erab_id,
                  (unsigned)qos_params.qci,
                  NBIOT_LCID_DRB1,
                  gtpu_teid_out,
                  e.teid_in,
                  e.nas.size());
  ue->erabs[erab_id] = std::move(e);
  return SRSRAN_SUCCESS;
}

int rrc_nbiot::modify_erab(uint16_t                                   rnti,
                           uint16_t                                   erab_id,
                           const asn1::s1ap::erab_level_qos_params_s& qos_params,
                           srsran::const_span<uint8_t>                nas_pdu,
                           asn1::s1ap::cause_c&                       cause)
{
  cause.set_radio_network().value = asn1::s1ap::cause_radio_network_opts::unspecified;
  return SRSRAN_ERROR;
}

int rrc_nbiot::release_erab(uint16_t rnti, uint16_t erab_id)
{
  ue_t* ue = find(rnti);
  if (ue == nullptr || ue->erabs.erase(erab_id) == 0) {
    return SRSRAN_ERROR;
  }
  gtpu->rem_bearer(rnti, erab_id);
  bearers->remove_eps_bearer(rnti, (uint8_t)erab_id);
  return SRSRAN_SUCCESS;
}

int rrc_nbiot::notify_ue_erab_updates(uint16_t rnti, srsran::const_span<uint8_t> nas_pdu)
{
  ue_t* ue = find(rnti);
  if (ue == nullptr) {
    return SRSRAN_ERROR;
  }
  if (nas_pdu.size() > 0) {
    srsran::unique_byte_buffer_t sdu = srsran::make_byte_buffer();
    if (sdu != nullptr && nas_pdu.size() <= sdu->get_tailroom()) {
      memcpy(sdu->msg, nas_pdu.data(), nas_pdu.size());
      sdu->N_bytes = (uint32_t)nas_pdu.size();
      write_dl_info(rnti, std::move(sdu));
    }
  }
  return SRSRAN_SUCCESS;
}

} // namespace srsenb
