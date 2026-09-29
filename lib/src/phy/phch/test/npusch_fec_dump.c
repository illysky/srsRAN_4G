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
 * Prints what the library's own UL-SCH coding blocks produce for a transport block, so that the independent Python
 * model (npusch_ref.py) can be compared with it bit for bit:
 *
 *   npusch_fec_dump A E RV      stdin: A characters '0' / '1' (the transport block)
 *
 *   crc:   the 24 CRC24A bits          turbo: 3 * (A + 24 + 4) bits, d0[i] d1[i] d2[i] interleaved
 *   rm:    E bits after rate matching
 */

#include "srsran/phy/common/phy_common.h"
#include "srsran/phy/fec/crc.h"
#include "srsran/phy/fec/turbo/rm_turbo.h"
#include "srsran/phy/fec/turbo/turbocoder.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_K 6144

int main(int argc, char** argv)
{
  if (argc != 4) {
    fprintf(stderr, "usage: %s A E RV  (bits on stdin)\n", argv[0]);
    return 2;
  }
  int a  = atoi(argv[1]);
  int e  = atoi(argv[2]);
  int rv = atoi(argv[3]);
  if (a <= 0 || a + 24 > MAX_K || e <= 0 || e > 100000) {
    return 2;
  }

  uint8_t* bits = calloc(MAX_K + 64, 1);
  for (int i = 0; i < a; i++) {
    int c = getchar();
    if (c != '0' && c != '1') {
      fprintf(stderr, "expected %d bits on stdin\n", a);
      return 2;
    }
    bits[i] = (uint8_t)(c - '0');
  }

  srsran_crc_t crc;
  if (srsran_crc_init(&crc, SRSRAN_LTE_CRC24A, 24)) {
    return 1;
  }
  srsran_crc_attach(&crc, bits, a);
  printf("crc: ");
  for (int i = 0; i < 24; i++) {
    printf("%d", bits[a + i]);
  }
  printf("\n");

  int           k = a + 24;
  srsran_tcod_t tcod;
  if (srsran_tcod_init(&tcod, MAX_K)) {
    return 1;
  }
  uint8_t* d = calloc(3 * MAX_K + 64, 1);
  if (srsran_tcod_encode(&tcod, bits, d, k)) {
    fprintf(stderr, "turbo encoder rejected K = %d\n", k);
    return 1;
  }
  printf("turbo: ");
  for (int i = 0; i < 3 * k + 12; i++) {
    printf("%d", d[i]);
  }
  printf("\n");

  uint8_t* w   = calloc(3 * 6528 + 64, 1);
  uint8_t* out = calloc(e + 64, 1);
  srsran_rm_turbo_gentables();
  // The transmit rate matcher only (re)builds its circular buffer when called with RV 0 and reuses it for the other
  // redundancy versions, so RV 0 has to be produced first (rm_turbo_test.c does the same).
  if (srsran_rm_turbo_tx(w, 3 * 6528, d, 3 * k + 12, out, e, 0) < 0) {
    return 1;
  }
  if (rv != 0 && srsran_rm_turbo_tx(w, 3 * 6528, d, 3 * k + 12, out, e, rv) < 0) {
    return 1;
  }
  printf("rm: ");
  for (int i = 0; i < e; i++) {
    printf("%d", out[i]);
  }
  printf("\n");
  srsran_rm_turbo_free_tables();
  srsran_tcod_free(&tcod);
  return 0;
}
