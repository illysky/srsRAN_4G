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

#include "srsenb/hdr/phy/phy.h"
#include "srsenb/hdr/phy/lte/nbiot_dl.h"
#include "srsenb/hdr/phy/nbiot_mac.h"
#include "srsenb/hdr/phy/nbiot_msg3_worker.h"
#include "srsenb/hdr/phy/nbiot_prach_worker.h"
extern "C" {
#include "srsran/phy/phch/nbiot_ra.h"
}
#include "srsran/common/band_helper.h"
#include "srsran/common/phy_cfg_nr_default.h"
#include "srsran/common/standard_streams.h"
#include "srsran/common/threads.h"
#include <pthread.h>
#include <sstream>
#include <string.h>
#include <string>
#include <strings.h>
#include <unistd.h>

#define Error(fmt, ...)                                                                                                \
  if (SRSRAN_DEBUG_ENABLED)                                                                                            \
  phy_log.error(fmt, ##__VA_ARGS__)
#define Warning(fmt, ...)                                                                                              \
  if (SRSRAN_DEBUG_ENABLED)                                                                                            \
  phy_log.warning(fmt, ##__VA_ARGS__)
#define Info(fmt, ...)                                                                                                 \
  if (SRSRAN_DEBUG_ENABLED)                                                                                            \
  phy_log.info(fmt, ##__VA_ARGS__)
#define Debug(fmt, ...)                                                                                                \
  if (SRSRAN_DEBUG_ENABLED)                                                                                            \
  phy_log.debug(fmt, ##__VA_ARGS__)

using namespace std;
using namespace asn1::rrc;

namespace srsenb {

static void srsran_phy_handler(phy_logger_level_t log_level, void* ctx, char* str)
{
  phy* r = (phy*)ctx;
  r->srsran_phy_logger(log_level, str);
}

void phy::srsran_phy_logger(phy_logger_level_t log_level, char* str)
{
  switch (log_level) {
    case LOG_LEVEL_INFO_S:
      phy_lib_log.info(" %s", str);
      break;
    case LOG_LEVEL_DEBUG_S:
      phy_lib_log.debug(" %s", str);
      break;
    case LOG_LEVEL_ERROR_S:
      phy_lib_log.error(" %s", str);
      break;
    default:
      break;
  }
}

phy::phy(srslog::sink& log_sink) :
  log_sink(log_sink),
  phy_log(srslog::fetch_basic_logger("PHY", log_sink)),
  phy_lib_log(srslog::fetch_basic_logger("PHY_LIB", log_sink)),
  lte_workers(MAX_WORKERS),
  workers_common(),
  nof_workers(0),
  tx_rx(phy_log)
{}

phy::~phy()
{
  stop();
}

void phy::parse_common_config(const phy_cfg_t& cfg)
{
  // PRACH configuration
  prach_cfg.config_idx       = cfg.prach_cnfg.prach_cfg_info.prach_cfg_idx;
  prach_cfg.hs_flag          = cfg.prach_cnfg.prach_cfg_info.high_speed_flag;
  prach_cfg.root_seq_idx     = cfg.prach_cnfg.root_seq_idx;
  prach_cfg.zero_corr_zone   = cfg.prach_cnfg.prach_cfg_info.zero_correlation_zone_cfg;
  prach_cfg.freq_offset      = cfg.prach_cnfg.prach_cfg_info.prach_freq_offset;
  prach_cfg.num_ra_preambles = cfg.phy_cell_cfg.empty() ? 0 : cfg.phy_cell_cfg.at(0).num_ra_preambles;
  // DMRS
  workers_common.dmrs_pusch_cfg.cyclic_shift        = cfg.pusch_cnfg.ul_ref_sigs_pusch.cyclic_shift;
  workers_common.dmrs_pusch_cfg.delta_ss            = cfg.pusch_cnfg.ul_ref_sigs_pusch.group_assign_pusch;
  workers_common.dmrs_pusch_cfg.group_hopping_en    = cfg.pusch_cnfg.ul_ref_sigs_pusch.group_hop_enabled;
  workers_common.dmrs_pusch_cfg.sequence_hopping_en = cfg.pusch_cnfg.ul_ref_sigs_pusch.seq_hop_enabled;
}

int phy::init(const phy_args_t&            args,
              const phy_cfg_t&             cfg,
              srsran::radio_interface_phy* radio_,
              stack_interface_phy_lte*     stack_lte_,
              stack_interface_phy_nr&      stack_nr_,
              enb_time_interface*          enb_)
{
  if (init_lte(args, cfg, radio_, stack_lte_, enb_) != SRSRAN_SUCCESS) {
    phy_log.error("Couldn't initialize LTE PHY");
    return SRSRAN_ERROR;
  }

  if (init_nr(args, cfg, stack_nr_) != SRSRAN_SUCCESS) {
    phy_log.error("Couldn't initialize NR PHY");
    return SRSRAN_ERROR;
  }

  tx_rx.init(enb_, radio, &lte_workers, &workers_common, &prach, SF_RECV_THREAD_PRIO);
  initialized = true;

  return SRSRAN_SUCCESS;
}

int phy::init(const phy_args_t&            args,
              const phy_cfg_t&             cfg,
              srsran::radio_interface_phy* radio_,
              stack_interface_phy_lte*     stack_lte_,
              enb_time_interface*          enb_)
{
  if (init_lte(args, cfg, radio_, stack_lte_, enb_) != SRSRAN_SUCCESS) {
    phy_log.error("Couldn't initialize LTE PHY");
    return SRSRAN_ERROR;
  }

  tx_rx.init(enb_, radio, &lte_workers, &workers_common, &prach, SF_RECV_THREAD_PRIO);
  initialized = true;

  return SRSRAN_SUCCESS;
}

int phy::init_lte(const phy_args_t&            args,
                  const phy_cfg_t&             cfg,
                  srsran::radio_interface_phy* radio_,
                  stack_interface_phy_lte*     stack_lte_,
                  enb_time_interface*          enb_)
{
  if (cfg.phy_cell_cfg.size() > SRSRAN_MAX_CARRIERS) {
    phy_log.error(
        "Number of carriers=%zd provided above its maximum=%zd", cfg.phy_cell_cfg.size(), SRSRAN_MAX_CARRIERS);
    return SRSRAN_ERROR;
  }

  // Add PHY lib log.
  srslog::basic_levels lib_log_lvl = srslog::str_to_basic_level(args.log.phy_lib_level);
  srslog::basic_levels log_lvl     = srslog::str_to_basic_level(args.log.phy_level);

  phy_lib_log.set_level(lib_log_lvl);
  phy_lib_log.set_hex_dump_max_size(args.log.phy_hex_limit);
  if (log_lvl != srslog::basic_levels::none) {
    srsran_phy_log_register_handler(this, srsran_phy_handler);
  }

  // Create default log.
  phy_log.set_level(log_lvl);
  phy_log.set_hex_dump_max_size(args.log.phy_hex_limit);

  radio       = radio_;
  nof_workers = cfg.phy_cell_cfg.empty() ? 0 : args.nof_phy_threads;

  workers_common.params = args;
  if (not args.nbiot_config.empty()) {
    // before the workers exist: each downlink worker of the NB-IoT carrier gets this table
    workers_common.nbiot_sched.reset(srsran_nbiot_dl_sched_new(), srsran_nbiot_dl_sched_free);
    if (!workers_common.nbiot_sched) {
      phy_log.error("NB-IoT: cannot allocate the downlink schedule");
      return SRSRAN_ERROR;
    }
  }

  workers_common.init(cfg.phy_cell_cfg, cfg.phy_cell_cfg_nr, radio, stack_lte_);
  if (cfg.cfr_config.cfr_enable) {
    workers_common.set_cfr_config(cfg.cfr_config);
  }

  parse_common_config(cfg);

  // Add workers to workers pool and start threads
  if (not cfg.phy_cell_cfg.empty()) {
    lte_workers.init(args, &workers_common, log_sink, WORKERS_THREAD_PRIO);
  }

  // For each carrier, initialise PRACH worker
  for (uint32_t cc = 0; cc < cfg.phy_cell_cfg.size(); cc++) {
    prach_cfg.root_seq_idx = cfg.phy_cell_cfg[cc].root_seq_idx;
    prach.init(cc,
               cfg.phy_cell_cfg[cc].cell,
               prach_cfg,
               stack_lte_,
               phy_log,
               PRACH_WORKER_THREAD_PRIO,
               args.nof_prach_threads);
  }
  prach.set_max_prach_offset_us(args.max_prach_offset_us);

  if (not args.nbiot_config.empty() and not cfg.phy_cell_cfg.empty()) {
    if (init_nbiot_prach(args, cfg) != SRSRAN_SUCCESS) {
      return SRSRAN_ERROR;
    }
  }

  return SRSRAN_SUCCESS;
}

int phy::init_nbiot_prach(const phy_args_t& args, const phy_cfg_t& cfg)
{
  nbiot::cell_config nb_cfg;
  std::string        err;
  if (not lte::nbiot_dl::load(args.nbiot_config, nb_cfg, err)) {
    phy_log.error("NB-IoT: %s", err.c_str());
    return SRSRAN_ERROR;
  }

  nbiot_nprach_params p = {};
  p.lte_nof_prb         = cfg.phy_cell_cfg[0].cell.nof_prb;
  p.anchor_prb          = nb_cfg.nbiot_prb;
  p.periodicity_ms      = nb_cfg.sib2.nprach_periodicity_ms;
  p.start_time_ms       = nb_cfg.sib2.nprach_start_time_ms;
  p.nprach.cell_id      = nb_cfg.n_id_ncell;
  p.nprach.format       = nb_cfg.sib2.nprach_cp_length_us > 100.0 ? 1 : 0;
  p.nprach.n_rep        = nb_cfg.sib2.num_repetitions_per_preamble;
  p.nprach.n_sc_offset  = nb_cfg.sib2.nprach_subcarrier_offset;
  p.nprach.n_sc_nprach  = nb_cfg.sib2.nprach_num_subcarriers;

  nbiot_prach.reset(new nbiot_prach_worker(phy_log));
  if (nbiot_prach->init(p, PRACH_WORKER_THREAD_PRIO, err) != SRSRAN_SUCCESS) {
    phy_log.error("NB-IoT NPRACH receiver: %s", err.c_str());
    srsran::console("NB-IoT NPRACH receiver: %s\n", err.c_str());
    nbiot_prach.reset();
    return SRSRAN_ERROR;
  }
  // Msg2: the random access response goes out through the downlink composers
  nbiot_ra_config ra_cfg;
  if (not nbiot_ra_config::from_cell(nb_cfg, ra_cfg, err)) {
    phy_log.error("NB-IoT random access: %s", err.c_str());
    srsran::console("NB-IoT random access: %s\n", err.c_str());
    return SRSRAN_ERROR;
  }
  srsran_nbiot_cell_t nb_cell = {};
  nb_cell.base                = cfg.phy_cell_cfg[0].cell;
  nb_cell.nbiot_prb           = nb_cfg.nbiot_prb;
  nb_cell.n_id_ncell          = nb_cfg.n_id_ncell;
  nb_cell.nof_ports           = nb_cfg.nof_ports;
  nb_cell.is_r14              = true;
  nb_cell.mode                = nb_cfg.mode;
  nbiot_ra.reset(new nbiot_mac(phy_log));
  if (not nbiot_ra->init(ra_cfg, nb_cell, workers_common.nbiot_sched.get(), err)) {
    phy_log.error("NB-IoT random access: %s", err.c_str());
    srsran::console("NB-IoT random access: %s\n", err.c_str());
    nbiot_ra.reset();
    return SRSRAN_ERROR;
  }
  nbiot_mac* ra = nbiot_ra.get();
  nbiot_prach->set_callback([ra](const nbiot_nprach_detection& d) {
    srsran::console("NB-IoT: NPRACH preamble %u at TTI %u (frame %u), ToA %.2f samples = %.1f us (timing advance %u), "
                    "CFO %+.0f Hz, metric %.0f\n",
                    d.n_init,
                    d.tti,
                    d.tti / 10,
                    d.toa,
                    d.toa / 1.92,
                    srsran_nbiot_ta_from_toa(d.toa),
                    d.cfo_hz,
                    d.metric);
    ra->preamble_detected(d);
  });
  tx_rx.set_nbiot_prach(nbiot_prach.get());

  // Msg3: the NPUSCH each response grants
  nbiot_msg3.reset(new nbiot_msg3_worker(phy_log));
  if (nbiot_msg3->init(cfg.phy_cell_cfg[0].cell.nof_prb, nb_cfg.nbiot_prb, PRACH_WORKER_THREAD_PRIO, err) !=
      SRSRAN_SUCCESS) {
    phy_log.error("NB-IoT Msg3 receiver: %s", err.c_str());
    srsran::console("NB-IoT Msg3 receiver: %s\n", err.c_str());
    nbiot_msg3.reset();
    return SRSRAN_ERROR;
  }
  nbiot_msg3_worker* m3            = nbiot_msg3.get();
  const uint32_t     cell_id       = nb_cfg.n_id_ncell;
  const bool         group_hopping = nb_cfg.sib2.group_hopping_enabled;
  const uint32_t     delta_ss      = nb_cfg.sib2.group_assignment_npusch;
  ra->set_response_callback([m3, cell_id, group_hopping, delta_ss](const nbiot_ra_response& r) {
    nbiot_npusch_expect e;
    uint32_t            k0 = 0;
    if (srsran_nbiot_msg3_grant_to_npusch(&r.grant, r.tc_rnti, cell_id, &e.cfg, &k0) != SRSRAN_SUCCESS) {
      srsran::console("NB-IoT: Msg3 grant of TC-RNTI 0x%04x cannot be received\n", r.tc_rnti);
      return;
    }
    // TS 36.213 16.5.1: the first uplink slot after the end of subframe n + k0, n the last RAR NPDSCH subframe
    e.start_sf          = r.npdsch_end + k0 + 1;
    e.cfg.frame         = (uint32_t)((e.start_sf / 10) % 1024);
    e.cfg.slot          = (uint32_t)(2 * (e.start_sf % 10));
    e.cfg.group_hopping = group_hopping;
    e.cfg.delta_ss      = delta_ss;
    e.rnti              = r.tc_rnti;
    e.preamble          = r.preamble;
    std::string why;
    if (not m3->expect(e, why)) {
      srsran::console("NB-IoT: Msg3 of TC-RNTI 0x%04x not expected: %s\n", r.tc_rnti, why.c_str());
    }
  });
  nbiot_msg3->set_callback([ra](const nbiot_npusch_result& res) {
    srsran::console("NB-IoT: Msg3 of TC-RNTI 0x%04x (preamble %u, subframe %llu, %u RU x %u): CRC %s, SNR %.1f dB, "
                    "CFO %+.0f Hz\n",
                    res.req.rnti,
                    res.req.preamble,
                    (unsigned long long)res.req.start_sf,
                    res.req.cfg.n_ru,
                    res.req.cfg.n_rep,
                    res.res.crc_ok ? "OK" : "KO",
                    res.res.snr_db,
                    res.res.cfo_hz);
    if (res.res.crc_ok) {
      ra->msg3_received(res.req.rnti, res.tb.data(), (uint32_t)res.tb.size());
    }
  });
  tx_rx.set_nbiot_msg3(nbiot_msg3.get());
  srsran::console("NB-IoT: listening for NPRACH (period %u ms, start %u ms, %u subcarriers, %u repetitions)\n",
                  p.periodicity_ms,
                  p.start_time_ms,
                  p.nprach.n_sc_nprach,
                  p.nprach.n_rep);
  return SRSRAN_SUCCESS;
}

void phy::stop()
{
  if (initialized) {
    tx_rx.stop();
    workers_common.stop();
    lte_workers.stop();
    if (nr_workers != nullptr) {
      nr_workers->stop();
    }
    prach.stop();
    if (nbiot_prach) {
      nbiot_prach->stop();
    }
    if (nbiot_msg3) {
      nbiot_msg3->stop();
    }

    initialized = false;
  }
}

/***** MAC->PHY interface **********/

void phy::rem_rnti(uint16_t rnti)
{
  // Remove the RNTI when the TTI finishes, this has a delay up to the pipeline length (3 ms)
  for (uint32_t i = 0; i < nof_workers; i++) {
    lte::sf_worker* w = lte_workers.wait_worker_id(i);
    if (w) {
      w->rem_rnti(rnti);
      w->release();
    }
  }
  if (SRSRAN_RNTI_ISUSER(rnti)) {
    workers_common.ue_db.rem_rnti(rnti);
    workers_common.clear_grants(rnti);
  }
}

void phy::set_mch_period_stop(uint32_t stop)
{
  workers_common.set_mch_period_stop(stop);
}

void phy::set_activation_deactivation_scell(uint16_t rnti, const std::array<bool, SRSRAN_MAX_CARRIERS>& activation)
{
  // Iterate all elements except 0 that is reserved for primary cell
  for (uint32_t scell_idx = 1; scell_idx < SRSRAN_MAX_CARRIERS; scell_idx++) {
    workers_common.ue_db.activate_deactivate_scell(rnti, scell_idx, activation[scell_idx]);
  }
}

void phy::get_metrics(std::vector<phy_metrics_t>& metrics)
{
  std::vector<phy_metrics_t> metrics_tmp;
  for (uint32_t i = 0; i < nof_workers; i++) {
    lte_workers[i]->get_metrics(metrics_tmp);
    metrics.resize(std::max(metrics_tmp.size(), metrics.size()));
    for (uint32_t j = 0; j < metrics_tmp.size(); j++) {
      metrics[j].dl.n_samples += metrics_tmp[j].dl.n_samples;
      metrics[j].dl.mcs += metrics_tmp[j].dl.n_samples * metrics_tmp[j].dl.mcs;

      metrics[j].ul.n_samples += metrics_tmp[j].ul.n_samples;
      metrics[j].ul.n_samples_pucch += metrics_tmp[j].ul.n_samples_pucch;
      metrics[j].ul.mcs += metrics_tmp[j].ul.n_samples * metrics_tmp[j].ul.mcs;
      metrics[j].ul.n += metrics_tmp[j].ul.n_samples * metrics_tmp[j].ul.n;
      metrics[j].ul.pusch_rssi += metrics_tmp[j].ul.n_samples * metrics_tmp[j].ul.pusch_rssi;
      metrics[j].ul.pusch_sinr += metrics_tmp[j].ul.n_samples * metrics_tmp[j].ul.pusch_sinr;
      metrics[j].ul.pucch_rssi += metrics_tmp[j].ul.n_samples_pucch * metrics_tmp[j].ul.pucch_rssi;
      metrics[j].ul.pucch_ni += metrics_tmp[j].ul.n_samples_pucch * metrics_tmp[j].ul.pucch_ni;
      metrics[j].ul.pucch_sinr += metrics_tmp[j].ul.n_samples_pucch * metrics_tmp[j].ul.pucch_sinr;
      metrics[j].ul.turbo_iters += metrics_tmp[j].ul.n_samples * metrics_tmp[j].ul.turbo_iters;
    }
  }
  for (uint32_t j = 0; j < metrics.size(); j++) {
    if (metrics[j].dl.n_samples > 0) {
      metrics[j].dl.mcs /= metrics[j].dl.n_samples;
    }
    if (metrics[j].ul.n_samples > 0) {
      metrics[j].ul.mcs /= metrics[j].ul.n_samples;
      metrics[j].ul.n /= metrics[j].ul.n_samples;
      metrics[j].ul.pusch_rssi /= metrics[j].ul.n_samples;
      metrics[j].ul.pusch_sinr /= metrics[j].ul.n_samples;
      metrics[j].ul.pucch_rssi /= metrics[j].ul.n_samples_pucch;
      metrics[j].ul.pucch_ni /= metrics[j].ul.n_samples_pucch;
      metrics[j].ul.pucch_sinr /= metrics[j].ul.n_samples_pucch;
      metrics[j].ul.turbo_iters /= metrics[j].ul.n_samples;
    }
  }
}

void phy::cmd_cell_gain(uint32_t cell_id, float gain_db)
{
  Info("set_cell_gain: cell_id=%d, gain_db=%.2f", cell_id, gain_db);
  workers_common.set_cell_gain(cell_id, gain_db);
}

void phy::cmd_cell_measure()
{
  workers_common.set_cell_measure_trigger();
}

/***** RRC->PHY interface **********/

void phy::set_config(uint16_t rnti, const phy_rrc_cfg_list_t& phy_cfg_list)
{
  // Update UE Database
  workers_common.ue_db.addmod_rnti(rnti, phy_cfg_list);

  // Iterate over the list and add the RNTIs
  for (const phy_rrc_cfg_t& config : phy_cfg_list) {
    // Add RNTI to eNb cell/carrier.
    // - Do not ignore PCell, it could have changed
    // - Do not remove RNTI from unused workers, it will be removed when the UE is released
    if (config.configured) {
      // Add RNTI to all SF workers
      for (uint32_t w = 0; w < nof_workers; w++) {
        lte_workers[w]->add_rnti(rnti, config.enb_cc_idx);
      }
    }
  }
}

void phy::complete_config(uint16_t rnti)
{
  // Forwards call to the UE Database
  if (workers_common.ue_db.complete_config(rnti) < SRSRAN_SUCCESS) {
    Error("Error completing configuration for RNTI %x. It does not exist.", rnti);
  }
}

void phy::configure_mbsfn(srsran::sib2_mbms_t* sib2, srsran::sib13_t* sib13, const srsran::mcch_msg_t& mcch)
{
  if (sib2->mbsfn_sf_cfg_list_present) {
    if (sib2->nof_mbsfn_sf_cfg == 0) {
      Warning("SIB2 does not have any MBSFN config although it was set as present");
    } else {
      if (sib2->nof_mbsfn_sf_cfg > 1) {
        Warning("SIB2 has %d MBSFN subframe configs - only 1 supported", sib2->nof_mbsfn_sf_cfg);
      }
      mbsfn_config.mbsfn_subfr_cnfg = sib2->mbsfn_sf_cfg_list[0];
    }
  } else {
    fprintf(stderr, "SIB2 has no MBSFN subframe config specified\n");
    return;
  }

  mbsfn_config.mbsfn_notification_cnfg = sib13->notif_cfg;
  if (sib13->nof_mbsfn_area_info > 0) {
    if (sib13->nof_mbsfn_area_info > 1) {
      Warning("SIB13 has %d MBSFN area info elements - only 1 supported", sib13->nof_mbsfn_area_info);
    }
    mbsfn_config.mbsfn_area_info = sib13->mbsfn_area_info_list[0];
  }

  mbsfn_config.mcch = mcch;

  workers_common.configure_mbsfn(&mbsfn_config);
}

// Start GUI
void phy::start_plot()
{
  if (lte_workers.get_nof_workers() > 0) {
    lte_workers[0]->start_plot();
  }
}

int phy::init_nr(const phy_args_t& args, const phy_cfg_t& cfg, stack_interface_phy_nr& stack)
{
  if (cfg.phy_cell_cfg_nr.empty()) {
    return SRSRAN_SUCCESS;
  }

  nr_workers = std::unique_ptr<nr::worker_pool>(new nr::worker_pool(workers_common, stack, log_sink, MAX_WORKERS));

  nr::worker_pool::args_t worker_args = {};
  worker_args.nof_phy_threads         = args.nof_phy_threads;
  worker_args.log.phy_level           = args.log.phy_level;
  worker_args.log.phy_hex_limit       = args.log.phy_hex_limit;
  worker_args.pusch_max_its           = args.nr_pusch_max_its;

  if (not nr_workers->init(worker_args, cfg.phy_cell_cfg_nr)) {
    return SRSRAN_ERROR;
  }

  tx_rx.set_nr_workers(nr_workers.get());

  if (nr_workers->set_common_cfg(common_cfg)) {
    phy_log.error("Couldn't set common PHY config");
    return SRSRAN_ERROR;
  }

  return SRSRAN_SUCCESS;
}

int phy::set_common_cfg(const phy_interface_rrc_nr::common_cfg_t& common_cfg_)
{
  if (nr_workers.get() == nullptr) {
    // if nr_workers are not initialized yet, store the configuration in the phy
    common_cfg = common_cfg_;
    return SRSRAN_SUCCESS;
  }

  return nr_workers->set_common_cfg(common_cfg);
}

} // namespace srsenb
