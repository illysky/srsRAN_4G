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

#ifndef SRSENB_NBIOT_DL_H
#define SRSENB_NBIOT_DL_H

#include <cstdint>
#include <string>

#include "nbiot_sib_builder.h"
extern "C" {
#include "srsran/phy/enb/enb_dl_nbiot.h"
}
#include "srsran/srslog/srslog.h"

namespace srsenb {
namespace lte {

/**
 * The NB-IoT downlink of one LTE carrier (in-band, anchor PRB inside the LTE grid).
 *
 * It owns one srsran_enb_dl_nbiot_t composer and the system information it broadcasts, both built from the same
 * nbiot::cell_config file, so what is broadcast is what is transmitted. The broadcast content is static except for the
 * hyper-SFN most significant bits inside SIB1-NB, which is repacked when they change.
 *
 * One instance per cc_worker. The composer is a pure function of (H-SFN, SFN, subframe) and of the table of planned
 * transmissions, so the workers that build consecutive subframes in parallel share nothing but that table.
 */
class nbiot_dl
{
public:
  nbiot_dl() = default;
  ~nbiot_dl();
  nbiot_dl(const nbiot_dl&) = delete;
  nbiot_dl& operator=(const nbiot_dl&) = delete;

  /// Reads the NB-IoT carrier description and checks it against the LTE cell it is embedded in
  bool init(const std::string&    config_file,
            const srsran_cell_t&  lte_cell,
            srslog::basic_logger& logger,
            std::string&          err,
            srsran_nbiot_dl_sched_t* sched = nullptr);

  /// Reads and validates the file without allocating a composer
  static bool load(const std::string& config_file, nbiot::cell_config& cfg, std::string& err);

  /// The NB-IoT carrier shares the LTE grid, reference signals and cell identity: they must describe the same cell
  static bool check_host(const nbiot::cell_config& cfg, const srsran_cell_t& lte_cell, std::string& err);

  /// Puts the NB-IoT anchor PRB of the subframe with the given (transmit) TTI into the LTE grids.
  /// hfn counts hyper frames since start, unwrapped; only its 10 LSBs go on the air. Returns false on an internal error.
  bool put_sf(uint32_t hfn, uint32_t tti, cf_t* sf_symbols[SRSRAN_MAX_PORTS]);

  const nbiot::cell_config& cell() const { return cfg; }
  uint64_t                  lte_collisions() const { return comp.lte_collisions; }

private:
  nbiot::cell_config    cfg;
  srsran_enb_dl_nbiot_t comp      = {};
  bool                  initiated = false;

  srslog::basic_logger* logger = nullptr;

  // Transmissions the MAC has planned (NPDCCH, addressed NPDSCH), shared by all composers of the cell. May be null.
  srsran_nbiot_dl_sched_t* sched            = nullptr;
  bool                     layout_published = false;

  // H-SFN MSBs the current SIB1-NB payload was packed for (-1: none yet)
  int32_t  sib1_msb         = -1;
  uint64_t collisions_seen  = 0;
  uint64_t subframes_put    = 0;
};

} // namespace lte
} // namespace srsenb

#endif // SRSENB_NBIOT_DL_H
