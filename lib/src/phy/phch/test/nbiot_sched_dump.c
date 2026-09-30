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
 * Dumps, for one broadcast configuration, every subframe the library says carries SIB1-NB or an SI message, for
 * nbiot_sched_check.py to compare with a model written from the specification.
 *
 *   nbiot_sched_dump n_id sched_info_sib1 n periodicity offset rep_pattern tb window_ms
 *
 * Output lines:
 *   S sfn sf start_sfn sf_idx nof_sf                  SIB1-NB, all 1024 frames (it repeats every hyperframe)
 *   I hfn sfn sf start_sfn sf_idx nof_sf              SI message, two blocks of four hyperframes' worth of frames:
 *                                                     frame counters [0, 4096) and [2^20 - 2048, 2^20 + 2048)
 */

#include <stdio.h>
#include <stdlib.h>

#include "srsran/phy/phch/nbiot_sched.h"

int main(int argc, char** argv)
{
  if (argc != 9) {
    fprintf(stderr, "usage: %s n_id sched_info_sib1 n periodicity offset rep_pattern tb window_ms\n", argv[0]);
    return 1;
  }
  uint32_t n_id = (uint32_t)atoi(argv[1]);

  srsran_mib_nb_t mib = {};
  mib.sched_info_sib1 = (uint8_t)atoi(argv[2]);

  srsran_nbiot_si_params_t p = {};
  p.n                        = (uint32_t)atoi(argv[3]);
  p.si_periodicity           = (uint32_t)atoi(argv[4]);
  p.si_radio_frame_offset    = (uint32_t)atoi(argv[5]);
  p.si_repetition_pattern    = (uint32_t)atoi(argv[6]);
  p.si_tb                    = (uint32_t)atoi(argv[7]);
  p.si_window_length         = (uint32_t)atoi(argv[8]);

  srsran_nbiot_bcch_pos_t pos;
  for (uint32_t sfn = 0; sfn < 1024; sfn++) {
    for (uint32_t sf = 0; sf < 10; sf++) {
      if (srsran_nbiot_sib1_locate(n_id, &mib, sfn, sf, &pos)) {
        printf("S %u %u %u %u %u\n", sfn, sf, pos.start_sfn, pos.sf_idx, pos.nof_sf);
      }
    }
  }

  const uint64_t wrap    = 1u << 20;
  const uint64_t blocks[2][2] = {{0, 4096}, {wrap - 2048, wrap + 2048}};
  for (int b = 0; b < 2; b++) {
    for (uint64_t f = blocks[b][0]; f < blocks[b][1]; f++) {
      uint32_t hfn = (uint32_t)((f % wrap) / 1024);
      uint32_t sfn = (uint32_t)(f % 1024);
      for (uint32_t sf = 0; sf < 10; sf++) {
        if (srsran_nbiot_si_locate(n_id, &mib, &p, hfn, sfn, sf, &pos)) {
          printf("I %u %u %u %u %u %u\n", hfn, sfn, sf, pos.start_sfn, pos.sf_idx, pos.nof_sf);
        }
      }
    }
  }
  return 0;
}
