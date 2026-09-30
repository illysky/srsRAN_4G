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

/**
 * Command line driver for nbiot_dlch.c and nbiot_dl_sched.c, read by nbiot_dlch_check.py, which holds the model they
 * are compared with. Reads commands from stdin, one per line, and prints the result.
 *
 *   CELL nof_prb anchor pci
 *   DCIN1 i_delay i_sf i_mcs i_rep ndi harq dci_rep         -> DCI <23 bits>
 *   DCIN0 i_sc i_ru i_delay i_mcs rv i_rep ndi dci_rep      -> DCI <23 bits>
 *   UNPACKN1 <23 bits> / UNPACKN0 <23 bits>                 -> fields
 *   TABLE nsf|nrep|k0|tbs a [b]                             -> value
 *   NPDCCH rnti reinit_sf pos <23 bits>                     -> E <bits>, then RE l k re im for every non-zero element
 *   NPDSCH rnti nof_sf sf_in_cw pass_sfn pass_sf <tb hex>   -> E <bits>, RE ...
 *   LAYOUT pci sched_info_sib1 [n periodicity offset pattern tb window]...   (up to 3 SI messages)
 *   VALID t                                                 -> 0/1
 *   SS r_max g_halves offset_eighths t_min                  -> start period
 *   PLANC k0 r                                              -> PLAN t reinit pos ...
 *   PLAND n_last k0 n_sf n_rep                              -> PLAN t sf_in_cw pass_sfn pass_sf ...
 */
#include <complex.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "srsran/phy/phch/nbiot_dl_sched.h"
#include "srsran/phy/phch/nbiot_dlch.h"
#include "srsran/phy/phch/nbiot_grid.h"

static srsran_nbiot_dlch_t   dlch;
static srsran_nbiot_layout_t layout;
static cf_t*                 grid;
static uint32_t              nof_prb, anchor_prb;
static bool                  have_cell;

static int parse_bits(const char* s, uint8_t* out, uint32_t n)
{
  if (strlen(s) != n) {
    return -1;
  }
  for (uint32_t i = 0; i < n; i++) {
    out[i] = s[i] == '1';
  }
  return 0;
}

static void print_bits(const uint8_t* b, uint32_t n)
{
  for (uint32_t i = 0; i < n; i++) {
    putchar('0' + b[i]);
  }
  putchar('\n');
}

static void print_grid(void)
{
  const uint32_t w = nof_prb * 12;
  for (uint32_t l = 0; l < 14; l++) {
    for (uint32_t k = 0; k < w; k++) {
      cf_t v = grid[l * w + k];
      if (v != 0) {
        printf("RE %u %u %.7f %.7f\n", l, k, crealf(v), cimagf(v));
      }
    }
  }
}

int main(void)
{
  char line[4096];
  while (fgets(line, sizeof(line), stdin)) {
    char cmd[32] = {0};
    if (sscanf(line, "%31s", cmd) != 1) {
      continue;
    }
    char* args = line + strlen(cmd);

    if (!strcmp(cmd, "CELL")) {
      unsigned pci;
      sscanf(args, "%u %u %u", &nof_prb, &anchor_prb, &pci);
      srsran_nbiot_cell_t nb;
      memset(&nb, 0, sizeof(nb));
      nb.base.nof_prb   = nof_prb;
      nb.base.nof_ports = 1;
      nb.base.id        = pci;
      nb.base.cp        = SRSRAN_CP_NORM;
      nb.nbiot_prb      = anchor_prb;
      nb.n_id_ncell     = pci;
      nb.nof_ports      = 1;
      nb.is_r14         = true;
      nb.mode           = SRSRAN_NBIOT_MODE_INBAND_SAME_PCI;
      if (have_cell) {
        srsran_nbiot_dlch_free(&dlch);
        free(grid);
      }
      if (srsran_nbiot_dlch_init(&dlch, &nb, 3)) {
        printf("ERROR init\n");
        return 2;
      }
      grid      = calloc(14 * nof_prb * 12, sizeof(cf_t));
      have_cell = true;
      printf("NOF_RE %u\n", dlch.nof_re);
    } else if (!strcmp(cmd, "DCIN1")) {
      srsran_nbiot_dci_n1_t d;
      unsigned              a[7];
      sscanf(args, "%u %u %u %u %u %u %u", &a[0], &a[1], &a[2], &a[3], &a[4], &a[5], &a[6]);
      d = (srsran_nbiot_dci_n1_t){a[0], a[1], a[2], a[3], a[4], a[5], a[6]};
      uint8_t bits[SRSRAN_NBIOT_DCI_LEN];
      if (srsran_nbiot_dci_n1_pack(&d, bits)) {
        printf("DCI ERROR\n");
      } else {
        printf("DCI ");
        print_bits(bits, SRSRAN_NBIOT_DCI_LEN);
      }
    } else if (!strcmp(cmd, "DCIN0")) {
      unsigned              a[8];
      sscanf(args, "%u %u %u %u %u %u %u %u", &a[0], &a[1], &a[2], &a[3], &a[4], &a[5], &a[6], &a[7]);
      srsran_nbiot_dci_n0_t d = {a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7]};
      uint8_t               bits[SRSRAN_NBIOT_DCI_LEN];
      if (srsran_nbiot_dci_n0_pack(&d, bits)) {
        printf("DCI ERROR\n");
      } else {
        printf("DCI ");
        print_bits(bits, SRSRAN_NBIOT_DCI_LEN);
      }
    } else if (!strcmp(cmd, "UNPACKN1") || !strcmp(cmd, "UNPACKN0")) {
      char    s[64];
      uint8_t bits[SRSRAN_NBIOT_DCI_LEN];
      sscanf(args, "%63s", s);
      if (parse_bits(s, bits, SRSRAN_NBIOT_DCI_LEN)) {
        printf("UNPACK ERROR\n");
      } else if (!strcmp(cmd, "UNPACKN1")) {
        srsran_nbiot_dci_n1_t d;
        if (srsran_nbiot_dci_n1_unpack(bits, &d)) {
          printf("UNPACK ERROR\n");
        } else {
          printf("UNPACK %u %u %u %u %u %u %u\n", d.i_delay, d.i_sf, d.i_mcs, d.i_rep, d.ndi, d.harq_ack_res, d.dci_rep);
        }
      } else {
        srsran_nbiot_dci_n0_t d;
        if (srsran_nbiot_dci_n0_unpack(bits, &d)) {
          printf("UNPACK ERROR\n");
        } else {
          printf("UNPACK %u %u %u %u %u %u %u %u\n", d.i_sc, d.i_ru, d.i_delay, d.i_mcs, d.rv, d.i_rep, d.ndi, d.dci_rep);
        }
      }
    } else if (!strcmp(cmd, "TABLE")) {
      char     which[16];
      unsigned a = 0, b = 0;
      sscanf(args, "%15s %u %u", which, &a, &b);
      if (!strcmp(which, "nsf")) {
        printf("TABLE %u\n", srsran_nbiot_npdsch_n_sf(a));
      } else if (!strcmp(which, "nrep")) {
        printf("TABLE %u\n", srsran_nbiot_npdsch_n_rep(a));
      } else if (!strcmp(which, "k0")) {
        printf("TABLE %d\n", srsran_nbiot_npdsch_k0(a, b));
      } else {
        printf("TABLE %u\n", srsran_nbiot_npdsch_tbs(a, b));
      }
    } else if (!strcmp(cmd, "NPDCCH") && have_cell) {
      unsigned rnti, reinit, pos;
      char     s[64];
      sscanf(args, "%u %u %u %63s", &rnti, &reinit, &pos, s);
      uint8_t dci[SRSRAN_NBIOT_DCI_LEN];
      uint8_t e[2 * SRSRAN_NBIOT_DLCH_MAX_RE];
      parse_bits(s, dci, SRSRAN_NBIOT_DCI_LEN);
      memset(grid, 0, 14 * nof_prb * 12 * sizeof(cf_t));
      if (srsran_nbiot_npdcch_encode(&dlch, dci, SRSRAN_NBIOT_DCI_LEN, (uint16_t)rnti, e) ||
          srsran_nbiot_npdcch_put_sf(&dlch, e, reinit, pos, grid)) {
        printf("ERROR npdcch\n");
      } else {
        printf("E ");
        print_bits(e, srsran_nbiot_dlch_bits_per_sf(&dlch));
        print_grid();
      }
    } else if (!strcmp(cmd, "NPDSCH") && have_cell) {
      unsigned rnti, nof_sf, sf_in_cw, sfn, sf;
      char     hex[2 * 400];
      sscanf(args, "%u %u %u %u %u %799s", &rnti, &nof_sf, &sf_in_cw, &sfn, &sf, hex);
      uint8_t tb[400];
      uint32_t n = strlen(hex) / 2;
      for (uint32_t i = 0; i < n; i++) {
        unsigned v;
        sscanf(&hex[2 * i], "%2x", &v);
        tb[i] = v;
      }
      static uint8_t e[SRSRAN_NBIOT_DL_SCHED_MAX_E];
      memset(grid, 0, 14 * nof_prb * 12 * sizeof(cf_t));
      if (srsran_nbiot_npdsch_encode(&dlch, tb, n * 8, nof_sf, e) ||
          srsran_nbiot_npdsch_put_sf(&dlch, e, nof_sf, sf_in_cw, (uint16_t)rnti, sfn, sf, grid)) {
        printf("ERROR npdsch\n");
      } else {
        printf("E ");
        print_bits(e, nof_sf * srsran_nbiot_dlch_bits_per_sf(&dlch));
        print_grid();
      }
    } else if (!strcmp(cmd, "SENTINEL")) {
      printf("SENTINEL_DONE\n");
    } else if (!strcmp(cmd, "LAYOUT")) {
      memset(&layout, 0, sizeof(layout));
      unsigned pci, sched;
      int      off = 0, used = 0;
      sscanf(args, "%u %u%n", &pci, &sched, &used);
      layout.cell_id              = pci;
      layout.mib.sched_info_sib1 = sched;
      layout.sib1_set             = true;
      off                         = used;
      for (int i = 0; i < 3; i++) {
        unsigned n, t, o, p, tb, w;
        if (sscanf(args + off, "%u %u %u %u %u %u%n", &n, &t, &o, &p, &tb, &w, &used) != 6) {
          break;
        }
        off += used;
        layout.si[i].n                     = n;
        layout.si[i].si_periodicity        = t;
        layout.si[i].si_radio_frame_offset = o;
        layout.si[i].si_repetition_pattern = p;
        layout.si[i].si_tb                 = tb;
        layout.si[i].si_window_length      = w;
        layout.si_set[i]                   = true;
      }
      printf("LAYOUT OK\n");
    } else if (!strcmp(cmd, "VALID")) {
      unsigned long long t;
      sscanf(args, "%llu", &t);
      printf("VALID %d\n", srsran_nbiot_layout_is_dl_sf(&layout, t));
    } else if (!strcmp(cmd, "SS")) {
      unsigned long long tmin;
      unsigned           r, g, o;
      sscanf(args, "%u %u %u %llu", &r, &g, &o, &tmin);
      uint32_t T;
      uint64_t s = srsran_nbiot_search_space_start(r, g, o, tmin, &T);
      printf("SS %llu %u\n", (unsigned long long)s, T);
    } else if (!strcmp(cmd, "PLANC")) {
      unsigned long long k0;
      unsigned           r;
      sscanf(args, "%llu %u", &k0, &r);
      srsran_nbiot_plan_t p;
      if (srsran_nbiot_plan_npdcch(&layout, k0, r, &p)) {
        printf("PLAN ERROR\n");
      } else {
        printf("PLAN");
        for (uint32_t i = 0; i < p.nof_sf; i++) {
          printf(" %llu,%u,%u", (unsigned long long)p.t[i], p.reinit_sf[i], p.pos_in_group[i]);
        }
        printf("\n");
      }
    } else if (!strcmp(cmd, "PLAND")) {
      unsigned long long n_last;
      unsigned           k0, n_sf, n_rep;
      sscanf(args, "%llu %u %u %u", &n_last, &k0, &n_sf, &n_rep);
      srsran_nbiot_plan_t p;
      if (srsran_nbiot_plan_npdsch(&layout, n_last, k0, n_sf, n_rep, &p)) {
        printf("PLAN ERROR\n");
      } else {
        printf("PLAN");
        for (uint32_t i = 0; i < p.nof_sf; i++) {
          printf(" %llu,%u,%u,%u", (unsigned long long)p.t[i], p.sf_in_cw[i], p.pass_sfn[i], p.pass_sf[i]);
        }
        printf("\n");
      }
    }
    fflush(stdout);
  }
  return 0;
}
