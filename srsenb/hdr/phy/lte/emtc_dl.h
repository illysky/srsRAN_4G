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
#include "srsran/interfaces/enb_mac_interfaces.h"
#include "srsran/srslog/srslog.h"
#include <memory>

extern "C" {
#include "srsran/phy/fec/softbuffer.h"
#include "srsran/phy/phch/mpdcch.h"
#include "srsran/phy/phch/pdsch.h"
}

namespace srsenb {
namespace lte {

/**
 * The LTE-M downlink of one LTE carrier, written into the LTE grid after the LTE channels: SIB1-BR and the BR SI
 * messages (a pure function of the TTI) and what the MAC scheduled for BL/CE UEs (MPDCCH and narrowband PDSCH). The
 * LTE scheduler keeps those PRBs free, so nothing of LTE is overwritten. One instance per cc_worker.
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
  void put_sf(uint32_t tti, const mac_interface_phy_lte::dl_sched_t& grants, cf_t* sf_symbols[SRSRAN_MAX_PORTS]);

  uint32_t ul_hop_interval() const { return bc ? bc->ul_hop_interval : 1; }

private:
  struct pdsch_args {
    uint16_t                rnti;
    uint32_t                first_prb; ///< absolute
    uint32_t                nof_prb;
    uint32_t                lstart;
    srsran_mod_t            mod;
    uint32_t                tbs; ///< bits
    uint32_t                rv;
    uint32_t                n_acc;
    uint8_t*                data;
    srsran_softbuffer_tx_t* softbuffer;
  };
  bool encode(uint32_t tti, const pdsch_args& a, cf_t* sf_symbols[SRSRAN_MAX_PORTS]);
  bool encode_bcast(uint32_t tti, uint32_t nb, uint32_t lstart, uint32_t rv, uint32_t n_acc, const std::vector<uint8_t>& tb,
                    cf_t* sf_symbols[SRSRAN_MAX_PORTS]);

  std::shared_ptr<const emtc::bcast> bc;
  srsran_cell_t                      cell      = {};
  srsran_pdsch_t                     pdsch     = {};
  srsran_softbuffer_tx_t             sb        = {};
  srsran_mpdcch_t                    mpdcch    = {};
  bool                               initiated = false;
  srslog::basic_logger*              logger    = nullptr;
  std::vector<uint8_t>               tb_buf;
  std::vector<uint8_t>               sib1_buf;
};

} // namespace lte
} // namespace srsenb

#endif // SRSENB_EMTC_DL_H
