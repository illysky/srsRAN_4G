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

#include "srsenb/hdr/phy/lte/emtc_dl.h"

extern "C" {
#include "srsran/phy/phch/ra_dl.h"
}

namespace srsenb {
namespace lte {

emtc_dl::~emtc_dl()
{
  if (initiated) {
    srsran_pdsch_free(&pdsch);
    srsran_softbuffer_tx_free(&sb);
  }
}

bool emtc_dl::init(std::shared_ptr<const emtc::bcast> bcast,
                   const srsran_cell_t&               cell_,
                   srslog::basic_logger&              logger_,
                   std::string&                       err)
{
  bc     = std::move(bcast);
  cell   = cell_;
  logger = &logger_;
  if (srsran_pdsch_init_enb(&pdsch, cell.nof_prb) != SRSRAN_SUCCESS || srsran_pdsch_set_cell(&pdsch, cell) != SRSRAN_SUCCESS) {
    err = "PDSCH init failed";
    return false;
  }
  if (srsran_softbuffer_tx_init(&sb, cell.nof_prb) != SRSRAN_SUCCESS) {
    srsran_pdsch_free(&pdsch);
    err = "softbuffer init failed";
    return false;
  }
  initiated = true;
  return true;
}

bool emtc_dl::encode(uint32_t                    tti,
                     uint32_t                    nb,
                     uint32_t                    lstart,
                     uint32_t                    rv,
                     uint32_t                    n_acc,
                     const std::vector<uint8_t>& tb,
                     cf_t*                       sf_symbols[SRSRAN_MAX_PORTS])
{
  const int first = srsran_emtc_nb_first_prb(cell.nof_prb, nb);
  if (first < 0 || tb.empty()) {
    return false;
  }

  srsran_dl_sf_cfg_t sf = {};
  sf.tti                = tti;
  sf.cfi                = lstart; // the BR start symbol stands in for the control region (above 10 PRB)
  sf.sf_type            = SRSRAN_SF_NORM;

  srsran_pdsch_cfg_t pc     = {};
  srsran_pdsch_grant_t& g   = pc.grant;
  g.tx_scheme               = cell.nof_ports == 1 ? SRSRAN_TXSCHEME_PORT0 : SRSRAN_TXSCHEME_DIVERSITY;
  g.nof_layers              = cell.nof_ports;
  g.nof_tb                  = 1;
  g.nof_prb                 = SRSRAN_EMTC_NB_NOF_PRB;
  g.nof_symb_slot[0]        = SRSRAN_CP_NSYMB(cell.cp);
  g.nof_symb_slot[1]        = SRSRAN_CP_NSYMB(cell.cp);
  for (uint32_t n = 0; n < SRSRAN_EMTC_NB_NOF_PRB; n++) {
    g.prb_idx[0][first + n] = true;
    g.prb_idx[1][first + n] = true;
  }
  g.nof_re            = srsran_ra_dl_grant_nof_re(&cell, &sf, &g);
  g.tb[0].enabled     = true;
  g.tb[0].mod         = SRSRAN_MOD_QPSK;
  g.tb[0].tbs         = (int)tb.size() * 8;
  g.tb[0].rv          = (int)rv;
  g.tb[0].cw_idx      = 0;
  g.tb[0].nof_bits    = g.nof_re * 2;
  pc.rnti             = SRSRAN_SIRNTI;
  pc.softbuffers.tx[0] = &sb;
  pc.bl_ce_scrambling    = true;
  pc.bl_ce_scrambling_sf = srsran_emtc_scrambling_sf(tti, n_acc);

  tb_buf = tb;
  uint8_t* data[SRSRAN_MAX_CODEWORDS] = {tb_buf.data(), nullptr};
  srsran_softbuffer_tx_reset(&sb);
  if (srsran_pdsch_encode(&pdsch, &sf, &pc, data, sf_symbols) != SRSRAN_SUCCESS) {
    logger->error("LTE-M: PDSCH encoding failed in TTI %d (narrowband %d)", tti, nb);
    return false;
  }
  return true;
}

void emtc_dl::put_sf(uint32_t tti, cf_t* sf_symbols[SRSRAN_MAX_PORTS])
{
  if (!initiated || !bc) {
    return;
  }
  const uint32_t sfn = (tti / 10) % 1024;
  const uint32_t sf  = tti % 10;
  uint32_t       nb = 0, rv = 0;

  // SIB1-BR: N_acc = 1, starts at symbol 3 above 10 PRB (TS 36.213 7.1.6.4A)
  if (srsran_emtc_sib1_br(&bc->sched, sfn, sf, &nb, &rv)) {
    encode(tti, nb, cell.nof_prb > 10 ? 3 : 4, rv, 1, bc->sib1, sf_symbols);
  }

  // BR SI messages: N_acc = 4 (FDD)
  const int si = srsran_emtc_si(&bc->sched, sfn, sf, &rv);
  if (si >= 0 && (size_t)si < bc->si.size()) {
    encode(tti, bc->sched.si[si].nb, bc->start_symbol, rv, 4, bc->si[si], sf_symbols);
  }
}

} // namespace lte
} // namespace srsenb
