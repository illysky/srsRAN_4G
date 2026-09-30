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

#include "srsran/phy/enb/enb_dl_nbiot.h"

#include <stdio.h>
#include <string.h>

#include "srsran/phy/utils/vector.h"

#define GRID_W(q) ((q)->cell.base.nof_prb * SRSRAN_NRE)
#define ANCHOR_COL(q) ((q)->cell.nbiot_prb * SRSRAN_NRE)

static uint32_t crs_v(uint32_t port, uint32_t symbol_in_slot, uint32_t slot)
{
  // TS 36.211 6.10.1.2
  switch (port) {
    case 0:
      return symbol_in_slot == 0 ? 0 : 3;
    case 1:
      return symbol_in_slot == 0 ? 3 : 0;
    case 2:
      return 3 * (slot % 2);
    default:
      return 3 + 3 * (slot % 2);
  }
}

static void find_crs(srsran_enb_dl_nbiot_t* q)
{
  q->nof_crs_re = 0;
  for (uint32_t p = 0; p < q->lte_nof_ports; p++) {
    for (uint32_t slot = 0; slot < 2; slot++) {
      // CRS symbols of this port within the slot; only those after the control region matter here
      uint32_t syms[2];
      uint32_t nsyms = 0;
      if (p < 2) {
        syms[nsyms++] = 0;
        syms[nsyms++] = 4;
      } else {
        syms[nsyms++] = 1;
      }
      for (uint32_t i = 0; i < nsyms; i++) {
        uint32_t l = slot * SRSRAN_CP_NORM_NSYMB + syms[i];
        if (l < SRSRAN_ENB_DL_NBIOT_L_START) {
          continue;
        }
        for (uint32_t m = 0; m < 2; m++) {
          uint32_t k = 6 * m + ((crs_v(p, syms[i], slot) + q->cell.base.id % 6) % 6);
          q->crs_idx[q->nof_crs_re]  = l * GRID_W(q) + ANCHOR_COL(q) + k;
          q->crs_port[q->nof_crs_re] = p;
          q->nof_crs_re++;
        }
      }
    }
  }
}

int srsran_enb_dl_nbiot_init(srsran_enb_dl_nbiot_t* q, const srsran_nbiot_cell_t* cell)
{
  if (q == NULL || cell == NULL) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  bzero(q, sizeof(*q));

  srsran_nbiot_cell_t c = *cell;
  if (!srsran_nbiot_cell_isvalid(&c) || !srsran_nbiot_prb_isvalid(&c)) {
    fprintf(stderr, "enb_dl_nbiot: invalid NB-IoT cell\n");
    return SRSRAN_ERROR;
  }
  if (c.mode != SRSRAN_NBIOT_MODE_INBAND_SAME_PCI) {
    fprintf(stderr, "enb_dl_nbiot: only in-band same-PCI is implemented\n");
    return SRSRAN_ERROR;
  }
  if (c.n_id_ncell != c.base.id) {
    fprintf(stderr, "enb_dl_nbiot: NB-IoT PCI %u must equal the LTE PCI %u in same-PCI mode\n", c.n_id_ncell, c.base.id);
    return SRSRAN_ERROR;
  }
  if (c.nof_ports != c.base.nof_ports || c.nof_ports != 1) {
    // Two ports would need SFBC of NPBCH/NPDSCH and the second NRS port; the code paths exist in the library but
    // nothing has checked them against the specification, so they are not offered.
    fprintf(stderr,
            "enb_dl_nbiot: NB-IoT ports (%u) must equal LTE ports (%u) and be 1 (the verified configuration)\n",
            c.nof_ports,
            c.base.nof_ports);
    return SRSRAN_ERROR;
  }
  if (!c.is_r14) {
    fprintf(stderr, "enb_dl_nbiot: only Rel-14 conventions (rotated NPBCH, BCCH scrambling) are implemented\n");
    return SRSRAN_ERROR;
  }
  if (c.base.cp != SRSRAN_CP_NORM) {
    fprintf(stderr, "enb_dl_nbiot: only normal cyclic prefix is implemented\n");
    return SRSRAN_ERROR;
  }
  // The anchor must not sit on the LTE PSS/SSS/PBCH: those occupy the 72 subcarriers around the carrier centre
  {
    uint32_t centre = c.base.nof_prb * SRSRAN_NRE / 2;
    uint32_t lo     = c.nbiot_prb * SRSRAN_NRE;
    uint32_t hi     = lo + SRSRAN_NRE;
    if (hi > centre - 36 && lo < centre + 36) {
      fprintf(stderr, "enb_dl_nbiot: anchor PRB %u overlaps the LTE centre six PRBs (PSS/SSS/PBCH)\n", c.nbiot_prb);
      return SRSRAN_ERROR;
    }
  }

  q->cell          = c;
  q->lte_nof_ports = c.base.nof_ports;

  if (srsran_npss_generate(q->npss) != SRSRAN_SUCCESS) {
    fprintf(stderr, "enb_dl_nbiot: NPSS generation failed\n");
    goto error;
  }
  srsran_nsss_generate(q->nsss, c.n_id_ncell);

  if (srsran_refsignal_dl_nbiot_init(&q->nrs) || srsran_refsignal_dl_nbiot_set_cell(&q->nrs, c)) {
    fprintf(stderr, "enb_dl_nbiot: NRS init failed\n");
    goto error;
  }
  if (srsran_npbch_init(&q->npbch) || srsran_npbch_set_cell(&q->npbch, c)) {
    fprintf(stderr, "enb_dl_nbiot: NPBCH init failed\n");
    goto error;
  }
  if (srsran_npdsch_init(&q->npdsch) || srsran_npdsch_set_cell(&q->npdsch, c)) {
    fprintf(stderr, "enb_dl_nbiot: NPDSCH init failed\n");
    goto error;
  }
  if (srsran_softbuffer_tx_init(&q->softbuffer, c.base.nof_prb)) {
    fprintf(stderr, "enb_dl_nbiot: softbuffer init failed\n");
    goto error;
  }

  find_crs(q);
  return SRSRAN_SUCCESS;

error:
  srsran_enb_dl_nbiot_free(q);
  return SRSRAN_ERROR;
}

void srsran_enb_dl_nbiot_free(srsran_enb_dl_nbiot_t* q)
{
  if (q == NULL) {
    return;
  }
  srsran_refsignal_dl_nbiot_free(&q->nrs);
  srsran_npbch_free(&q->npbch);
  srsran_npdsch_free(&q->npdsch);
  srsran_softbuffer_tx_free(&q->softbuffer);
  bzero(q, sizeof(*q));
}

int srsran_enb_dl_nbiot_set_mib(srsran_enb_dl_nbiot_t* q, const srsran_mib_nb_t* mib)
{
  if (q == NULL || mib == NULL) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  if (mib->sched_info_sib1 > 11) {
    fprintf(stderr, "enb_dl_nbiot: schedulingInfoSIB1 %u is reserved\n", mib->sched_info_sib1);
    return SRSRAN_ERROR;
  }
  q->mib     = *mib;
  q->mib_set = true;
  return SRSRAN_SUCCESS;
}

int srsran_enb_dl_nbiot_set_sib1(srsran_enb_dl_nbiot_t* q, const uint8_t* payload, uint32_t bytes)
{
  if (q == NULL || payload == NULL || !q->mib_set) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  int tbs = srsran_ra_nbiot_get_sib1_tbs(&q->mib);
  if (tbs <= 0 || bytes * 8 != (uint32_t)tbs || bytes > SRSRAN_ENB_DL_NBIOT_MAX_BCCH_BYTES) {
    fprintf(stderr, "enb_dl_nbiot: SIB1-NB is %u bytes but schedulingInfoSIB1 %u means %d bits\n", bytes, q->mib.sched_info_sib1, tbs);
    return SRSRAN_ERROR;
  }
  memcpy(q->sib1, payload, bytes);
  q->sib1_bytes = bytes;
  q->sib1_set   = true;
  return SRSRAN_SUCCESS;
}

int srsran_enb_dl_nbiot_set_si(srsran_enb_dl_nbiot_t*          q,
                               uint32_t                        idx,
                               const srsran_nbiot_si_params_t* p,
                               const uint8_t*                  payload,
                               uint32_t                        bytes)
{
  if (q == NULL || p == NULL || payload == NULL || idx >= SRSRAN_ENB_DL_NBIOT_MAX_SI) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  if (bytes * 8 != p->si_tb || bytes > SRSRAN_ENB_DL_NBIOT_MAX_BCCH_BYTES || bytes == 0) {
    fprintf(stderr, "enb_dl_nbiot: SI message %u is %u bytes but si-TB is %u bits\n", idx, bytes, p->si_tb);
    return SRSRAN_ERROR;
  }
  // si-TB of SystemInformationBlockType1-NB: b56, b120, b208, b256, b328, b440, b552, b680
  static const uint32_t valid_tb[] = {56, 120, 208, 256, 328, 440, 552, 680};
  bool                  tb_ok      = false;
  for (uint32_t i = 0; i < sizeof(valid_tb) / sizeof(valid_tb[0]); i++) {
    tb_ok |= (p->si_tb == valid_tb[i]);
  }
  if (!tb_ok) {
    fprintf(stderr, "enb_dl_nbiot: si-TB %u is not one of 56 120 208 256 328 440 552 680\n", p->si_tb);
    return SRSRAN_ERROR;
  }
  q->si_params[idx]   = *p;
  q->si_params[idx].n = idx + 1;
  memcpy(q->si[idx], payload, bytes);
  q->si_bytes[idx] = bytes;
  q->si_set[idx]   = true;
  return SRSRAN_SUCCESS;
}

// One subframe of a broadcast (BCCH, SI-RNTI) NPDSCH transmission
static int put_bcch(srsran_enb_dl_nbiot_t*           q,
                    const srsran_nbiot_bcch_pos_t*   pos,
                    bool                             is_sib1,
                    const uint8_t*                   payload,
                    uint32_t                         bytes,
                    uint32_t                         sf_idx,
                    cf_t*                            sf_symbols[SRSRAN_MAX_PORTS])
{
  srsran_ra_nbiot_dl_grant_t grant;
  bzero(&grant, sizeof(grant));
  grant.has_sib1     = is_sib1;
  grant.nof_sf       = pos->nof_sf;
  grant.nof_rep      = 1;
  grant.mcs[0].mod   = SRSRAN_MOD_QPSK;
  grant.mcs[0].tbs   = bytes * 8;
  grant.Qm           = 2;
  grant.l_start      = SRSRAN_ENB_DL_NBIOT_L_START;
  grant.start_sfn    = pos->start_sfn;
  grant.start_sfidx  = 0;
  grant.start_hfn    = 0;

  srsran_npdsch_cfg_t cfg;
  bzero(&cfg, sizeof(cfg));
  if (srsran_npdsch_cfg(&cfg, q->cell, &grant, sf_idx)) {
    return SRSRAN_ERROR;
  }
  cfg.has_bcch = true; // also for SI messages, which srsran_npdsch_cfg() does not know are broadcast
  cfg.sf_idx   = pos->sf_idx;

  uint8_t data[SRSRAN_ENB_DL_NBIOT_MAX_BCCH_BYTES];
  memcpy(data, payload, bytes);

  // The encoder codes the whole transport block and maps the subframe selected by cfg.sf_idx, so it has to be
  // reconfigured for every subframe (cfg.is_encoded false): that is what keeps this stateless.
  return srsran_npdsch_encode_rnti(&q->npdsch, &cfg, &q->softbuffer, data, SRSRAN_SIRNTI, sf_symbols);
}

int srsran_enb_dl_nbiot_put_sf(srsran_enb_dl_nbiot_t* q,
                               uint32_t               hfn,
                               uint32_t               sfn,
                               uint32_t               sf_idx,
                               cf_t*                  sf_symbols[SRSRAN_MAX_PORTS])
{
  if (q == NULL || sf_symbols == NULL || sfn >= 1024 || sf_idx >= 10 || q->lte_nof_ports == 0) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  for (uint32_t p = 0; p < q->lte_nof_ports; p++) {
    if (sf_symbols[p] == NULL) {
      return SRSRAN_ERROR_INVALID_INPUTS;
    }
  }
  if (!q->mib_set) {
    return SRSRAN_ERROR;
  }

  const uint32_t w   = GRID_W(q);
  const uint32_t col = ANCHOR_COL(q);
  int            ret = 0;

  // 1. Take the anchor PRB away from LTE: remember its CRS, note whether anything else was in the way, blank it
  cf_t saved[SRSRAN_ENB_DL_NBIOT_MAX_CRS_RE];
  for (uint32_t i = 0; i < q->nof_crs_re; i++) {
    saved[i] = sf_symbols[q->crs_port[i]][q->crs_idx[i]];
  }
  bool collided = false;
  for (uint32_t p = 0; p < q->lte_nof_ports; p++) {
    for (uint32_t l = SRSRAN_ENB_DL_NBIOT_L_START; l < SRSRAN_CP_NORM_SF_NSYMB; l++) {
      cf_t* re = &sf_symbols[p][l * w + col];
      for (uint32_t k = 0; k < SRSRAN_NRE; k++) {
        uint32_t idx = l * w + col + k;
        if (re[k] != 0) {
          bool is_crs = false;
          for (uint32_t i = 0; i < q->nof_crs_re; i++) {
            if (q->crs_idx[i] == idx && q->crs_port[i] == p) {
              is_crs = true;
              break;
            }
          }
          collided |= !is_crs;
        }
      }
      memset(re, 0, SRSRAN_NRE * sizeof(cf_t));
    }
  }
  if (collided) {
    q->lte_collisions++;
  }

  // The NB-IoT ports are the first nof_ports LTE ports (same PCI: NRS ports are the CRS ports)
  cf_t* nb_grid[SRSRAN_MAX_PORTS] = {NULL};
  for (uint32_t p = 0; p < q->cell.nof_ports; p++) {
    nb_grid[p] = sf_symbols[p];
  }

  // 2. NB-IoT signals
  bool nsss_sf = (sf_idx == 9 && (sfn % 2) == 0);

  if (sf_idx == 5) {
    // one antenna port for all symbols of the NPSS; port 0
    srsran_npss_put_subframe(NULL, q->npss, sf_symbols[0], q->cell.base.nof_prb, q->cell.nbiot_prb);
    ret |= SRSRAN_ENB_DL_NBIOT_HAS_NPSS;
  }
  if (nsss_sf) {
    srsran_nsss_put_subframe(NULL, q->nsss, sf_symbols[0], (int)sfn, q->cell.base.nof_prb, q->cell.nbiot_prb);
    ret |= SRSRAN_ENB_DL_NBIOT_HAS_NSSS;
  }

  // NRS: every NB-IoT downlink subframe, and subframe 0; not where NPSS / NSSS occupy the symbols
  if (sf_idx != 5 && !nsss_sf) {
    for (uint32_t p = 0; p < q->cell.nof_ports; p++) {
      if (srsran_refsignal_nrs_put_sf(q->cell, p, q->nrs.pilots[p][sf_idx], sf_symbols[p]) != SRSRAN_SUCCESS) {
        return SRSRAN_ERROR;
      }
    }
    ret |= SRSRAN_ENB_DL_NBIOT_HAS_NRS;
  }

  if (sf_idx == 0) {
    uint8_t payload[SRSRAN_MIB_NB_LEN];
    srsran_npbch_mib_pack(hfn, sfn & ~63u, q->mib, payload);
    if (srsran_npbch_encode_sf(&q->npbch, payload, nb_grid, sfn) < 0) {
      return SRSRAN_ERROR;
    }
    ret |= SRSRAN_ENB_DL_NBIOT_HAS_NPBCH;
  }

  srsran_nbiot_bcch_pos_t pos;
  if (q->sib1_set && srsran_nbiot_sib1_locate(q->cell.n_id_ncell, &q->mib, sfn, sf_idx, &pos)) {
    if (put_bcch(q, &pos, true, q->sib1, q->sib1_bytes, sf_idx, nb_grid) < 0) {
      return SRSRAN_ERROR;
    }
    ret |= SRSRAN_ENB_DL_NBIOT_HAS_SIB1;
  } else {
    for (uint32_t i = 0; i < SRSRAN_ENB_DL_NBIOT_MAX_SI; i++) {
      if (q->si_set[i] &&
          srsran_nbiot_si_locate(q->cell.n_id_ncell, &q->mib, &q->si_params[i], hfn, sfn, sf_idx, &pos)) {
        if (put_bcch(q, &pos, false, q->si[i], q->si_bytes[i], sf_idx, nb_grid) < 0) {
          return SRSRAN_ERROR;
        }
        ret |= SRSRAN_ENB_DL_NBIOT_HAS_SI;
        break; // SI windows of different messages do not overlap
      }
    }
  }

  // 3. LTE's CRS win over anything NB-IoT put there (NPSS/NSSS elements on CRS REs are "not used"), on every LTE port:
  // the owning port carries its CRS, the other ports are silent on that RE
  for (uint32_t i = 0; i < q->nof_crs_re; i++) {
    for (uint32_t p = 0; p < q->lte_nof_ports; p++) {
      sf_symbols[p][q->crs_idx[i]] = (p == q->crs_port[i]) ? saved[i] : 0;
    }
  }
  return ret;
}
