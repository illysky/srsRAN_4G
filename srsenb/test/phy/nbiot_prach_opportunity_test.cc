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

// NPRACH opportunity timing (TS 36.211 10.1.6): a preamble may start nprach-StartTime milliseconds after the first
// subframe of every radio frame n_f with n_f mod (nprach-Periodicity / 10) == 0. Checked against a direct enumeration
// of that rule over the 1024 frames of a hyper frame, for every periodicity and start time the standard allows.

#include "srsenb/hdr/phy/nbiot_prach_worker.h"
#include "srsran/common/test_common.h"
#include <set>

using srsenb::nbiot_prach_worker;

static int check(uint32_t period_ms, uint32_t start_ms)
{
  std::set<uint32_t> want;
  for (uint32_t f = 0; f < 1024; f++) {
    if (f % (period_ms / 10) == 0) {
      // start_ms after the start of frame f; a start time that runs past the hyper frame wraps to its start
      want.insert((f * 10 + start_ms) % 10240);
    }
  }
  uint32_t got = 0;
  for (uint32_t tti = 0; tti < 10240; tti++) {
    const bool is = nbiot_prach_worker::is_opportunity_start(period_ms, start_ms, tti);
    TESTASSERT(is == (want.count(tti) != 0));
    got += is ? 1 : 0;
  }
  TESTASSERT(got == want.size());
  return SRSRAN_SUCCESS;
}

int main()
{
  // nprach-Periodicity-r13 and nprach-StartTime-r13 of TS 36.331
  const uint32_t periods[] = {40, 80, 160, 240, 320, 640, 1280, 2560};
  const uint32_t starts[]  = {8, 16, 32, 64, 128, 256, 512, 1024, 2048};
  for (uint32_t p : periods) {
    for (uint32_t s : starts) {
      TESTASSERT(check(p, s) == SRSRAN_SUCCESS);
    }
  }

  // a few by hand, so that the enumeration above is not the only statement of the rule
  TESTASSERT(nbiot_prach_worker::is_opportunity_start(640, 8, 8));
  TESTASSERT(not nbiot_prach_worker::is_opportunity_start(640, 8, 7));
  TESTASSERT(not nbiot_prach_worker::is_opportunity_start(640, 8, 9));
  TESTASSERT(nbiot_prach_worker::is_opportunity_start(640, 8, 648));
  TESTASSERT(not nbiot_prach_worker::is_opportunity_start(640, 8, 328)); // half a period: not an opportunity
  TESTASSERT(nbiot_prach_worker::is_opportunity_start(40, 8, 8 + 400));
  TESTASSERT(nbiot_prach_worker::is_opportunity_start(2560, 2048, 2048 + 2560));
  TESTASSERT(not nbiot_prach_worker::is_opportunity_start(2560, 2048, 2048 + 1280));
  // 240 ms does not divide the hyper frame: frames 0, 24, .. 1008, then the pattern restarts at frame 0
  TESTASSERT(nbiot_prach_worker::is_opportunity_start(240, 8, 10080 + 8));
  TESTASSERT(nbiot_prach_worker::is_opportunity_start(240, 8, 8));
  TESTASSERT(not nbiot_prach_worker::is_opportunity_start(240, 8, 10240 - 240 + 240 - 10 + 8));
  // invalid periodicity never matches
  TESTASSERT(not nbiot_prach_worker::is_opportunity_start(0, 8, 8));
  TESTASSERT(not nbiot_prach_worker::is_opportunity_start(45, 8, 8));

  printf("Passed\n");
  return SRSRAN_SUCCESS;
}
