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

/*
 * Batch front end of the NB-IoT random access module for nbiot_ra_check.py (an independent Python model).
 * Reads requests on stdin, one per line, answers one line each:
 *
 *   P bi n  {rapid ta sc_15khz i_sc i_delay i_rep i_mcs tc_rnti} x n   ->  hex of the RAR MAC PDU, or ERR
 *   U hex                                                               ->  bi n {rapid ta sc i_sc delay rep mcs tc_rnti} x n, or ERR
 *   G sc_15khz i_sc i_delay i_rep i_mcs                                 ->  n_sc spacing sc n_ru n_rep qm tbs k0, or ERR
 *   R sfn carrier                                                       ->  RA-RNTI
 *   T toa                                                               ->  TA
 */

#include "srsran/phy/phch/nbiot_ra.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(void)
{
  char line[4096];
  while (fgets(line, sizeof(line), stdin)) {
    char* save = NULL;
    char* op   = strtok_r(line, " \t\r\n", &save);
    if (op == NULL) {
      continue;
    }
    if (op[0] == 'P') {
      int                bi = atoi(strtok_r(NULL, " \t\r\n", &save));
      uint32_t           n  = (uint32_t)atoi(strtok_r(NULL, " \t\r\n", &save));
      srsran_nbiot_rar_t rar[16];
      for (uint32_t i = 0; i < n && i < 16; ++i) {
        rar[i].rapid            = (uint32_t)strtoul(strtok_r(NULL, " \t\r\n", &save), NULL, 10);
        rar[i].ta               = (uint32_t)strtoul(strtok_r(NULL, " \t\r\n", &save), NULL, 10);
        rar[i].grant.sc_15khz   = (uint32_t)strtoul(strtok_r(NULL, " \t\r\n", &save), NULL, 10);
        rar[i].grant.i_sc       = (uint32_t)strtoul(strtok_r(NULL, " \t\r\n", &save), NULL, 10);
        rar[i].grant.i_delay    = (uint32_t)strtoul(strtok_r(NULL, " \t\r\n", &save), NULL, 10);
        rar[i].grant.i_rep      = (uint32_t)strtoul(strtok_r(NULL, " \t\r\n", &save), NULL, 10);
        rar[i].grant.i_mcs      = (uint32_t)strtoul(strtok_r(NULL, " \t\r\n", &save), NULL, 10);
        rar[i].tc_rnti          = (uint32_t)strtoul(strtok_r(NULL, " \t\r\n", &save), NULL, 10);
      }
      uint8_t out[256];
      int     len = srsran_nbiot_rar_pdu_pack(bi, rar, n, out, sizeof(out));
      if (len < 0) {
        printf("ERR\n");
      } else {
        for (int i = 0; i < len; ++i) {
          printf("%02x", out[i]);
        }
        printf("\n");
      }
    } else if (op[0] == 'U') {
      char*    hex = strtok_r(NULL, " \t\r\n", &save);
      uint8_t  pdu[256];
      uint32_t len = 0;
      for (; hex && hex[0] && hex[1] && len < sizeof(pdu); hex += 2) {
        unsigned v;
        sscanf(hex, "%2x", &v);
        pdu[len++] = (uint8_t)v;
      }
      int                bi;
      srsran_nbiot_rar_t rar[SRSRAN_NBIOT_RAR_MAX_PER_PDU];
      int                n = srsran_nbiot_rar_pdu_unpack(pdu, len, &bi, rar, SRSRAN_NBIOT_RAR_MAX_PER_PDU);
      if (n < 0) {
        printf("ERR\n");
      } else {
        printf("%d %d", bi, n);
        for (int i = 0; i < n; ++i) {
          printf(" %u %u %u %u %u %u %u %u",
                 rar[i].rapid,
                 rar[i].ta,
                 rar[i].grant.sc_15khz,
                 rar[i].grant.i_sc,
                 rar[i].grant.i_delay,
                 rar[i].grant.i_rep,
                 rar[i].grant.i_mcs,
                 rar[i].tc_rnti);
        }
        printf("\n");
      }
    } else if (op[0] == 'G') {
      srsran_nbiot_msg3_grant_t g;
      g.sc_15khz = (uint32_t)strtoul(strtok_r(NULL, " \t\r\n", &save), NULL, 10);
      g.i_sc     = (uint32_t)strtoul(strtok_r(NULL, " \t\r\n", &save), NULL, 10);
      g.i_delay  = (uint32_t)strtoul(strtok_r(NULL, " \t\r\n", &save), NULL, 10);
      g.i_rep    = (uint32_t)strtoul(strtok_r(NULL, " \t\r\n", &save), NULL, 10);
      g.i_mcs    = (uint32_t)strtoul(strtok_r(NULL, " \t\r\n", &save), NULL, 10);
      srsran_npusch_cfg_t cfg;
      uint32_t            k0;
      if (srsran_nbiot_msg3_grant_to_npusch(&g, 0x1234, 7, &cfg, &k0)) {
        printf("ERR\n");
      } else {
        printf("%u %u %u %u %u %u %u %u\n", cfg.n_sc, cfg.spacing_hz, cfg.sc, cfg.n_ru, cfg.n_rep, cfg.qm, cfg.tbs, k0);
      }
    } else if (op[0] == 'R') {
      uint32_t sfn = (uint32_t)atoi(strtok_r(NULL, " \t\r\n", &save));
      uint32_t car = (uint32_t)atoi(strtok_r(NULL, " \t\r\n", &save));
      printf("%u\n", srsran_nbiot_ra_rnti(sfn, car));
    } else if (op[0] == 'T') {
      float toa = (float)atof(strtok_r(NULL, " \t\r\n", &save));
      printf("%u\n", srsran_nbiot_ta_from_toa(toa));
    }
    fflush(stdout);
  }
  return 0;
}
