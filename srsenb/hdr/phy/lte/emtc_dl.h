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

#ifndef SRSENB_EMTC_DL_H
#define SRSENB_EMTC_DL_H

#include "srsenb/hdr/phy/emtc_config.h"
#include "srsran/srslog/srslog.h"
#include <memory>

extern "C" {
#include "srsran/phy/fec/softbuffer.h"
#include "srsran/phy/phch/pdsch.h"
}

namespace srsenb {
namespace lte {

/**
 * The LTE-M downlink of one LTE carrier: SIB1-BR and the BR SI messages in their narrowbands, written into the LTE
 * grid after the LTE channels. The LTE scheduler keeps those PRBs free (srsran_emtc_bcast_prbs), so nothing of LTE is
 * overwritten. One instance per cc_worker; what it transmits is a pure function of the TTI.
 */
class emtc_dl
{
public:
  emtc_dl() = default;
  ~emtc_dl();
  emtc_dl(const emtc_dl&) = delete;
  emtc_dl& operator=(const emtc_dl&) = delete;

  bool init(std::shared_ptr<const emtc::bcast> bcast, const srsran_cell_t& cell, srslog::basic_logger& logger, std::string& err);

  /// Puts the LTE-M transmissions of the subframe with this (transmit) TTI into the grids
  void put_sf(uint32_t tti, cf_t* sf_symbols[SRSRAN_MAX_PORTS]);

private:
  bool encode(uint32_t tti, uint32_t nb, uint32_t lstart, uint32_t rv, uint32_t n_acc, const std::vector<uint8_t>& tb,
              cf_t* sf_symbols[SRSRAN_MAX_PORTS]);

  std::shared_ptr<const emtc::bcast> bc;
  srsran_cell_t                      cell      = {};
  srsran_pdsch_t                     pdsch     = {};
  srsran_softbuffer_tx_t             sb        = {};
  bool                               initiated = false;
  srslog::basic_logger*              logger    = nullptr;
  std::vector<uint8_t>               tb_buf;
};

} // namespace lte
} // namespace srsenb

#endif // SRSENB_EMTC_DL_H
