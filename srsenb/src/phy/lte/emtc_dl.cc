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
#include "srsenb/hdr/phy/emtc_hsfn.h"

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
    srsran_mpdcch_free(&mpdcch);
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
  if (srsran_mpdcch_init(&mpdcch, cell) != SRSRAN_SUCCESS) {
    srsran_pdsch_free(&pdsch);
    srsran_softbuffer_tx_free(&sb);
    err = "MPDCCH init failed (normal CP and at least 6 PRB needed)";
    return false;
  }
  initiated = true;
  return true;
}

bool emtc_dl::encode(uint32_t tti, const pdsch_args& a, cf_t* sf_symbols[SRSRAN_MAX_PORTS])
{
  srsran_dl_sf_cfg_t sf = {};
  sf.tti                = tti;
  sf.cfi                = a.lstart; // the BR start symbol stands in for the control region (above 10 PRB)
  sf.sf_type            = SRSRAN_SF_NORM;

  srsran_pdsch_cfg_t    pc = {};
  srsran_pdsch_grant_t& g  = pc.grant;
  g.tx_scheme              = cell.nof_ports == 1 ? SRSRAN_TXSCHEME_PORT0 : SRSRAN_TXSCHEME_DIVERSITY;
  g.nof_layers             = cell.nof_ports;
  g.nof_tb                 = 1;
  g.nof_prb                = a.nof_prb;
  g.nof_symb_slot[0]       = SRSRAN_CP_NSYMB(cell.cp);
  g.nof_symb_slot[1]       = SRSRAN_CP_NSYMB(cell.cp);
  for (uint32_t n = 0; n < a.nof_prb; n++) {
    g.prb_idx[0][a.first_prb + n] = true;
    g.prb_idx[1][a.first_prb + n] = true;
  }
  g.nof_re             = srsran_ra_dl_grant_nof_re(&cell, &sf, &g);
  g.tb[0].enabled      = true;
  g.tb[0].mod          = a.mod;
  g.tb[0].tbs          = (int)a.tbs;
  g.tb[0].rv           = (int)a.rv;
  g.tb[0].cw_idx       = 0;
  g.tb[0].nof_bits     = g.nof_re * srsran_mod_bits_x_symbol(a.mod);
  pc.rnti              = a.rnti;
  pc.softbuffers.tx[0] = a.softbuffer;
  if (a.n_acc > 1) {
    pc.bl_ce_scrambling    = true;
    pc.bl_ce_scrambling_sf = srsran_emtc_scrambling_sf(tti, a.n_acc);
  }

  uint8_t* data[SRSRAN_MAX_CODEWORDS] = {a.data, nullptr};
  if (srsran_pdsch_encode(&pdsch, &sf, &pc, data, sf_symbols) != SRSRAN_SUCCESS) {
    logger->error("LTE-M: PDSCH encoding failed in TTI %d (rnti 0x%x, PRB %d+%d)", tti, a.rnti, a.first_prb, a.nof_prb);
    return false;
  }
  return true;
}

bool emtc_dl::encode_bcast(uint32_t                    tti,
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
  tb_buf = tb;
  srsran_softbuffer_tx_reset(&sb);
  pdsch_args a = {};
  a.rnti       = SRSRAN_SIRNTI;
  a.first_prb  = (uint32_t)first;
  a.nof_prb    = SRSRAN_EMTC_NB_NOF_PRB;
  a.lstart     = lstart;
  a.mod        = SRSRAN_MOD_QPSK;
  a.tbs        = (uint32_t)tb.size() * 8;
  a.rv         = rv;
  a.n_acc      = n_acc;
  a.data       = tb_buf.data();
  a.softbuffer = &sb;
  return encode(tti, a, sf_symbols);
}

void emtc_dl::put_sf(uint32_t tti, const mac_interface_phy_lte::dl_sched_t& grants, cf_t* sf_symbols[SRSRAN_MAX_PORTS])
{
  if (!initiated || !bc) {
    return;
  }
  const uint32_t sfn  = (tti / 10) % 1024;
  const uint32_t sf   = tti % 10;
  const uint32_t hsfn = emtc::hsfn(tti);
  uint32_t       nb = 0, rv = 0;

  // SIB1-BR: N_acc = 1, starts at symbol 3 above 10 PRB (TS 36.213 7.1.6.4A). The nRF91 (mfw 2.0.4) takes an all-zero
  // SIB2 when it decodes SIB1-BR in the 80 ms period in which the SIB2 window starts, so that period may go without.
  const bool sib1_skip = bc->sib1_skip_si_start && bc->sched.nof_si > 0 && sfn % bc->sched.si[0].periodicity_rf < 8;
  if (!sib1_skip && srsran_emtc_sib1_br(&bc->sched, sfn, sf, &nb, &rv)) {
    sib1_buf = bc->sib1;
    for (int i = 0; i < 10 && bc->sib1_hsfn_bit >= 0; i++) {
      const uint32_t b = (uint32_t)bc->sib1_hsfn_bit + i;
      const uint8_t  m = 0x80u >> (b % 8);
      sib1_buf[b / 8]  = ((hsfn >> (9 - i)) & 1u) ? (sib1_buf[b / 8] | m) : (sib1_buf[b / 8] & ~m);
    }
    encode_bcast(tti, nb, cell.nof_prb > 10 ? 3 : 4, rv, 1, sib1_buf, sf_symbols);
  }

  // BR SI messages: N_acc = 4 (FDD)
  const int si = srsran_emtc_si(&bc->sched, sfn, sf, &rv);
  if (si >= 0 && (size_t)si < bc->si.size()) {
    encode_bcast(tti, bc->sched.si[si].nb, bc->start_symbol, rv, 4, bc->si[si], sf_symbols);
  }

  // What the MAC scheduled for BL/CE UEs in this subframe (CE mode A: N_acc = 1)
  for (uint32_t i = 0; i < grants.nof_emtc_mpdcch && i < mac_interface_phy_lte::EMTC_MAX_GRANTS; i++) {
    const auto& m     = grants.emtc_mpdcch[i];
    const int   first = srsran_emtc_nb_first_prb(cell.nof_prb, m.nb);
    if (first < 0) {
      continue;
    }
    srsran_mpdcch_cfg_t mc = {};
    mc.first_prb           = (uint32_t)first;
    mc.start_symbol        = bc->start_symbol;
    mc.sf_idx              = sf;
    mc.common              = m.common;
    mc.n_id                = m.n_id;
    if (m.rnti == SRSRAN_PRNTI) {
      mc.scrambling_sf_set = true; // N_acc = 4 (FDD) for P-RNTI in every CE mode
      mc.scrambling_sf     = srsran_emtc_scrambling_sf(tti, 4);
    }
    if (srsran_mpdcch_encode(&mpdcch, &mc, m.dci, m.nof_bits, m.rnti, sf_symbols[0]) != SRSRAN_SUCCESS) {
      logger->error("LTE-M: MPDCCH encoding failed in TTI %d (rnti 0x%x)", tti, m.rnti);
    }
    if (getenv("EMTC_DEBUG")) {
      fprintf(stderr, "EMTC_TX: MPDCCH tti=%d rnti=0x%x nb=%d bits=%d\n", tti, m.rnti, m.nb, m.nof_bits);
    }
  }
  for (uint32_t i = 0; i < grants.nof_emtc_pdsch && i < mac_interface_phy_lte::EMTC_MAX_GRANTS; i++) {
    const auto& p     = grants.emtc_pdsch[i];
    const int   first = srsran_emtc_nb_first_prb(cell.nof_prb, p.nb);
    if (first < 0 || p.softbuffer == nullptr || (p.data == nullptr && p.rv == 0)) {
      continue;
    }
    pdsch_args a = {};
    a.rnti       = p.rnti;
    a.first_prb  = (uint32_t)first + p.rb_start;
    a.nof_prb    = p.nof_rb;
    a.lstart     = bc->start_symbol;
    a.mod        = p.mod;
    a.tbs        = p.tbs;
    a.rv         = p.rv;
    a.n_acc      = p.rnti == SRSRAN_PRNTI ? 4 : 1;
    a.data       = p.data;
    a.softbuffer = p.softbuffer;
    bool ok = encode(tti, a, sf_symbols);
    if (getenv("EMTC_DEBUG")) {
      fprintf(stderr, "EMTC_TX: PDSCH tti=%d rnti=0x%x prb=%d+%d tbs=%d ok=%d\n", tti, p.rnti, a.first_prb, a.nof_prb, a.tbs, ok);
    }
  }
}

} // namespace lte
} // namespace srsenb
