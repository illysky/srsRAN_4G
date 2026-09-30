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

#include "srsenb/hdr/phy/lte/nbiot_dl.h"

#include <cstring>
#include <vector>

namespace srsenb {
namespace lte {

nbiot_dl::~nbiot_dl()
{
  if (initiated) {
    srsran_enb_dl_nbiot_free(&comp);
  }
}

bool nbiot_dl::load(const std::string& config_file, nbiot::cell_config& cfg, std::string& err)
{
  if (!nbiot::load_config_file(config_file, cfg, err)) {
    return false;
  }
  if (cfg.mode != SRSRAN_NBIOT_MODE_INBAND_SAME_PCI) {
    err = "only in-band same-PCI operation is implemented in srsenb";
    return false;
  }
  if (cfg.si_sched.empty()) {
    err = "sib1.sched_info needs at least one entry (the first carries SIB2-NB)";
    return false;
  }
  return true;
}

bool nbiot_dl::check_host(const nbiot::cell_config& c, const srsran_cell_t& lte_cell, std::string& err)
{
  if (c.lte_nof_prb != lte_cell.nof_prb) {
    err = "nbiot.lte.n_prb=" + std::to_string(c.lte_nof_prb) + " but the eNB runs " + std::to_string(lte_cell.nof_prb) +
          " PRB";
    return false;
  }
  if (c.lte_pci != lte_cell.id) {
    err = "nbiot.lte.pci=" + std::to_string(c.lte_pci) + " but the eNB cell has PCI " + std::to_string(lte_cell.id);
    return false;
  }
  if (c.lte_nof_ports != lte_cell.nof_ports) {
    err = "nbiot.lte.nof_ports=" + std::to_string(c.lte_nof_ports) + " but the eNB runs " +
          std::to_string(lte_cell.nof_ports) + " antenna port(s)";
    return false;
  }
  return true;
}

bool nbiot_dl::init(const std::string&       config_file,
                    const srsran_cell_t&     lte_cell,
                    srslog::basic_logger&    logger_,
                    std::string&             err,
                    srsran_nbiot_dl_sched_t* sched_)
{
  logger = &logger_;
  sched  = sched_;

  if (!load(config_file, cfg, err)) {
    return false;
  }

  if (!check_host(cfg, lte_cell, err)) {
    return false;
  }

  srsran_nbiot_cell_t nb = {};
  nb.base                = lte_cell;
  nb.nbiot_prb           = cfg.nbiot_prb;
  nb.n_id_ncell          = cfg.n_id_ncell;
  nb.nof_ports           = cfg.nof_ports;
  nb.is_r14              = true;
  nb.mode                = cfg.mode;

  if (srsran_enb_dl_nbiot_init(&comp, &nb) != SRSRAN_SUCCESS) {
    err = "the NB-IoT composer refused the cell (see the message above)";
    return false;
  }
  initiated = true;
  srsran_enb_dl_nbiot_set_sched(&comp, sched);

  srsran_mib_nb_t mib = nbiot::to_c_mib(cfg);
  if (srsran_enb_dl_nbiot_set_mib(&comp, &mib) != SRSRAN_SUCCESS) {
    err = "cannot set MIB-NB";
    return false;
  }

  // SIB2-NB goes out in the first SI message. Further entries of schedulingInfoList would be advertised in SIB1-NB
  // but never sent, because no other SIB is implemented: say so instead of letting a UE wait for them.
  std::vector<uint8_t> si;
  size_t               unpadded = 0;
  if (!nbiot::pack_sib2(cfg, si, unpadded, err)) {
    err = "cannot pack SIB2-NB: " + err;
    return false;
  }
  srsran_nbiot_si_params_t p = {};
  p.si_periodicity           = cfg.si_sched[0].periodicity_rf;
  p.si_radio_frame_offset    = cfg.si_radio_frame_offset;
  p.si_repetition_pattern    = cfg.si_sched[0].repetition_pattern;
  p.si_tb                    = cfg.si_sched[0].tb_bits;
  p.si_window_length         = cfg.si_window_ms;
  if (srsran_enb_dl_nbiot_set_si(&comp, 0, &p, si.data(), (uint32_t)si.size()) != SRSRAN_SUCCESS) {
    err = "cannot set the SI message carrying SIB2-NB";
    return false;
  }
  if (cfg.si_sched.size() > 1) {
    logger->warning("NB-IoT: sib1.sched_info has %zu entries but only the first (SIB2-NB) is transmitted",
                    cfg.si_sched.size());
  }

  logger->info("NB-IoT in-band anchor: PRB %u of %u, PCI %u, SIB1-NB TBS %u bits, SIB2-NB in SI every %u frames",
               cfg.nbiot_prb,
               cfg.lte_nof_prb,
               cfg.n_id_ncell,
               nbiot::sib1_tbs_bits(cfg),
               cfg.si_sched[0].periodicity_rf);
  return true;
}

bool nbiot_dl::put_sf(uint32_t hfn, uint32_t tti, cf_t* sf_symbols[SRSRAN_MAX_PORTS])
{
  if (!initiated) {
    return false;
  }

  // SIB1-NB carries the 8 MSBs of the 10 bit hyper-SFN; MIB-NB carries the other two and is built per subframe
  const int32_t msb = (int32_t)((hfn & 0x3FF) >> 2);
  if (msb != sib1_msb) {
    std::vector<uint8_t> sib1;
    size_t               unpadded = 0;
    std::string          err;
    if (!nbiot::pack_sib1(cfg, (uint8_t)msb, sib1, unpadded, err)) {
      logger->error("NB-IoT: cannot pack SIB1-NB: %s", err.c_str());
      return false;
    }
    if (srsran_enb_dl_nbiot_set_sib1(&comp, sib1.data(), (uint32_t)sib1.size()) != SRSRAN_SUCCESS) {
      logger->error("NB-IoT: cannot set SIB1-NB");
      return false;
    }
    sib1_msb = msb;
  }

  // The MAC plans around the broadcast subframes: hand it the layout once the system information is complete
  if (sched != nullptr && !layout_published) {
    srsran_nbiot_layout_t layout = {};
    if (srsran_enb_dl_nbiot_get_layout(&comp, &layout) == SRSRAN_SUCCESS && layout.sib1_set) {
      srsran_nbiot_dl_sched_set_layout(sched, &layout);
      layout_published = true;
    }
  }

  const int r = srsran_enb_dl_nbiot_put_sf(&comp, hfn, tti / 10, tti % 10, sf_symbols);
  if (r < 0) {
    logger->error("NB-IoT: composing subframe failed (tti=%u)", tti);
    return false;
  }
  subframes_put++;

  if (comp.lte_collisions != collisions_seen) {
    // LTE had scheduled data on the anchor PRB. The scheduler is supposed to keep it clear.
    if (collisions_seen == 0 || (comp.lte_collisions - collisions_seen) >= 1000) {
      logger->warning("NB-IoT: LTE transmitted on the anchor PRB %u in %lu subframes; it was overwritten",
                      cfg.nbiot_prb,
                      (unsigned long)comp.lte_collisions);
      collisions_seen = comp.lte_collisions;
    }
  }
  return true;
}

} // namespace lte
} // namespace srsenb
