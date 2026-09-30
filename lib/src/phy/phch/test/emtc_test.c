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

#include "srsran/phy/phch/emtc.h"
#include <stdio.h>
#include <stdlib.h>

static int failures = 0;

#define CHECK(cond)                                                                                                    \
  do {                                                                                                                 \
    if (!(cond)) {                                                                                                     \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                                           \
      failures++;                                                                                                      \
    }                                                                                                                  \
  } while (0)

static void test_narrowbands(void)
{
  // 25 PRB: i0 = 12 - 12 = 0, the centre PRB 12 is in no narrowband
  CHECK(srsran_emtc_nof_nb(25) == 4);
  CHECK(srsran_emtc_nb_first_prb(25, 0) == 0);
  CHECK(srsran_emtc_nb_first_prb(25, 1) == 6);
  CHECK(srsran_emtc_nb_first_prb(25, 2) == 13);
  CHECK(srsran_emtc_nb_first_prb(25, 3) == 19);
  CHECK(srsran_emtc_nb_first_prb(25, 4) == -1);
  CHECK(!srsran_emtc_nb_in_centre(25, 0));
  CHECK(srsran_emtc_nb_in_centre(25, 1));
  CHECK(srsran_emtc_nb_in_centre(25, 2));
  CHECK(!srsran_emtc_nb_in_centre(25, 3));
  CHECK(srsran_emtc_nb_mask(25, 3) == (0x3FULL << 19));

  // 50 PRB: i0 = 25 - 24 = 1
  CHECK(srsran_emtc_nof_nb(50) == 8);
  CHECK(srsran_emtc_nb_first_prb(50, 0) == 1);
  CHECK(srsran_emtc_nb_first_prb(50, 7) == 43);
  CHECK(srsran_emtc_nb_in_centre(50, 3) && srsran_emtc_nb_in_centre(50, 4) && !srsran_emtc_nb_in_centre(50, 2));

  // 15 PRB: i0 = 7 - 6 = 1; odd, so the second narrowband skips the centre PRB 7
  CHECK(srsran_emtc_nb_first_prb(15, 0) == 1);
  CHECK(srsran_emtc_nb_first_prb(15, 1) == 8);

  // 6 PRB: one narrowband, the whole carrier
  CHECK(srsran_emtc_nof_nb(6) == 1 && srsran_emtc_nb_first_prb(6, 0) == 0);
}

static void test_sib1_br(void)
{
  CHECK(srsran_emtc_sib1_br_repetitions(0) == 0);
  CHECK(srsran_emtc_sib1_br_repetitions(10) == 4);
  CHECK(srsran_emtc_sib1_br_repetitions(11) == 8);
  CHECK(srsran_emtc_sib1_br_repetitions(18) == 16);
  CHECK(srsran_emtc_sib1_br_tbs(1) == 208 && srsran_emtc_sib1_br_tbs(10) == 504 && srsran_emtc_sib1_br_tbs(18) == 936);

  CHECK(srsran_emtc_rv(0) == 0 && srsran_emtc_rv(1) == 2 && srsran_emtc_rv(2) == 3 && srsran_emtc_rv(3) == 1);

  // 25 PRB, PCI 1, 4 repetitions: subframe 4 of odd frames, alternating narrowband 3 and 0 (the centre ones excluded)
  srsran_emtc_bcast_cfg_t c = {.nof_prb = 25, .pci = 1, .sched_info_sib1_br = 10};
  uint32_t                nb = 99, rv = 99, count = 0;
  const uint32_t          want_nb[4] = {3, 0, 3, 0}, want_rv[4] = {0, 2, 3, 1};
  for (uint32_t sfn = 0; sfn < 8; sfn++) {
    for (uint32_t sf = 0; sf < 10; sf++) {
      if (srsran_emtc_sib1_br(&c, sfn, sf, &nb, &rv)) {
        CHECK(sfn % 2 == 1 && sf == 4);
        CHECK(count < 4 && nb == want_nb[count] && rv == want_rv[count]);
        count++;
      }
    }
  }
  CHECK(count == 4);

  // PCI 0, 8 repetitions: subframe 4 of every frame, starting in narrowband 0
  c.pci                = 0;
  c.sched_info_sib1_br = 11;
  count                = 0;
  for (uint32_t sfn = 16; sfn < 24; sfn++) {
    for (uint32_t sf = 0; sf < 10; sf++) {
      if (srsran_emtc_sib1_br(&c, sfn, sf, &nb, &rv)) {
        CHECK(sf == 4 && nb == (count % 2 == 0 ? 0u : 3u) && rv == srsran_emtc_rv(sfn % 4));
        count++;
      }
    }
  }
  CHECK(count == 8);

  // PCI 1, 16 repetitions: subframes 0 and 9
  c.pci                = 1;
  c.sched_info_sib1_br = 12;
  count                = 0;
  for (uint32_t sfn = 0; sfn < 8; sfn++) {
    for (uint32_t sf = 0; sf < 10; sf++) {
      if (srsran_emtc_sib1_br(&c, sfn, sf, NULL, NULL)) {
        CHECK(sf == 0 || sf == 9);
        count++;
      }
    }
  }
  CHECK(count == 16);
}

static void test_si(void)
{
  // SI 1 (SIB2) every 16 frames in narrowband 0, window 20 ms; SI 2 every 32 frames in narrowband 3
  srsran_emtc_bcast_cfg_t c = {.nof_prb            = 25,
                               .pci                = 1,
                               .sched_info_sib1_br = 10,
                               .si_window_ms       = 20,
                               .si_repetition_rf   = 1,
                               .nof_si             = 2,
                               .si                 = {{16, 0, 328}, {32, 3, 208}}};
  uint32_t rv = 99, n0 = 0, n1 = 0;
  for (uint32_t sfn = 0; sfn < 64; sfn++) {
    for (uint32_t sf = 0; sf < 10; sf++) {
      const int si = srsran_emtc_si(&c, sfn, sf, &rv);
      if (si == 0) {
        CHECK(sfn % 16 < 2);
        CHECK(rv == srsran_emtc_rv(((sfn % 16) * 10 + sf) % 4));
        n0++;
      } else if (si == 1) {
        CHECK(sfn % 32 == 2 || sfn % 32 == 3);
        n1++;
      }
    }
  }
  // 4 windows of 20 subframes for SI 1 and 2 for SI 2. Neither loses a subframe to SIB1-BR: in frames 1 and 3 mod 8 it
  // is in narrowbands 3 and 0 respectively, the other narrowband from the SI message's.
  CHECK(n0 == 80);
  CHECK(n1 == 40);

  // Collision: SI in narrowband 3 during frame 1 (SIB1-BR in narrowband 3 at subframe 4) loses that subframe
  c.nof_si = 1;
  c.si[0]  = (srsran_emtc_si_t){16, 3, 328};
  CHECK(srsran_emtc_si(&c, 1, 4, NULL) == -1);
  CHECK(srsran_emtc_si(&c, 1, 5, NULL) == 0);
  CHECK(srsran_emtc_bcast_prbs(&c, 1, 4) == srsran_emtc_nb_mask(25, 3));

  // every2ndRF with a 40 ms window: frames 0 and 2 of the window only
  c.si_window_ms     = 40;
  c.si_repetition_rf = 2;
  CHECK(srsran_emtc_si(&c, 16, 0, NULL) == 0);
  CHECK(srsran_emtc_si(&c, 17, 0, NULL) == -1);
  CHECK(srsran_emtc_si(&c, 18, 0, NULL) == 0);
  CHECK(srsran_emtc_si(&c, 20, 0, NULL) == -1);
}

static void test_scrambling(void)
{
  CHECK(srsran_emtc_scrambling_sf(1234, 1) == 4);
  CHECK(srsran_emtc_scrambling_sf(8, 4) == 8);
  CHECK(srsran_emtc_scrambling_sf(11, 4) == 8);
  CHECK(srsran_emtc_scrambling_sf(12, 4) == 2);
}

int main(void)
{
  test_narrowbands();
  test_sib1_br();
  test_si();
  test_scrambling();
  printf("%s\n", failures ? "FAILED" : "OK");
  return failures ? 1 : 0;
}
