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
 * \brief Where the broadcast NPDSCH transmissions of an NB-IoT cell are, as pure functions of the time.
 *
 * The eNB has to send SIB1-NB and the SI messages exactly where a UE will look for them. These functions answer
 * "does subframe (H-SFN, SFN, sf) carry a piece of SIB1-NB / SI message n, and which one?" without any state, so the
 * answer does not depend on the order in which subframes are asked (srsenb builds consecutive subframes on parallel
 * workers).
 *
 * References: TS 36.213 16.4.1.3 (Tables 16.4.1.3-3, 16.4.1.3-4: SIB1-NB repetitions and starting frame),
 * TS 36.331 5.2.1.2a, 5.2.3a (SI-window, si-RepetitionPattern), si-TB field description of SystemInformationBlockType1-NB
 * (56 and 120 bit blocks use 2 subframes, all others 8), TS 36.211 10.2.3.
 */

#ifndef SRSRAN_NBIOT_SCHED_H
#define SRSRAN_NBIOT_SCHED_H

#include <stdbool.h>
#include <stdint.h>

#include "srsran/config.h"
#include "srsran/phy/phch/npbch.h"
#include "srsran/phy/phch/ra_nbiot.h"

/// One subframe of a broadcast NPDSCH repetition (a repetition is the whole transport block, N_SF subframes)
typedef struct SRSRAN_API {
  uint32_t start_sfn; ///< radio frame (0..1023) of the FIRST subframe of this repetition; feeds the scrambler (n_f)
  uint32_t sf_idx;    ///< index of this subframe within the repetition, 0 .. nof_sf-1
  uint32_t nof_sf;    ///< subframes per repetition: 8 for SIB1-NB, 2 or 8 for an SI message
} srsran_nbiot_bcch_pos_t;

/// Number of subframes an SI message of tb_bits occupies per transmission (TS 36.331: 56 and 120 bits -> 2, else 8)
SRSRAN_API uint32_t srsran_nbiot_si_nof_sf(uint32_t tb_bits);

/// True if radio frame sfn / subframe sf_idx belongs to a SIB1-NB transmission. Fills pos when it does.
/// The MIB supplies schedulingInfoSIB1 (repetitions and TBS). Returns false for sf_idx != 4 and for reserved values.
SRSRAN_API bool srsran_nbiot_sib1_locate(uint32_t              n_id_ncell,
                                         const srsran_mib_nb_t* mib,
                                         uint32_t              sfn,
                                         uint32_t              sf_idx,
                                         srsran_nbiot_bcch_pos_t* pos);

/// True if the subframe carries a piece of the SI message described by p (p->n = 1-based position in
/// schedulingInfoList; si_repetition_pattern = 2/4/8/16 radio frames; si_periodicity in radio frames; si_window_length
/// in ms; si_radio_frame_offset). Follows TS 36.331 5.2.3a: the SI-window starts at subframe 0 of the frame with
/// (H-SFN*1024 + SFN) mod T = floor(x/10) + offset, x = (n-1)*w; repetitions start every si_repetition_pattern frames
/// inside the window (a start exactly at the end of the window is outside it); each repetition takes the next
/// N_SF valid subframes (not NPBCH, NPSS, NSSS or SIB1-NB), continuing into following frames if the start frame has
/// too few.
SRSRAN_API bool srsran_nbiot_si_locate(uint32_t                       n_id_ncell,
                                       const srsran_mib_nb_t*          mib,
                                       const srsran_nbiot_si_params_t* p,
                                       uint32_t                       hfn,
                                       uint32_t                       sfn,
                                       uint32_t                       sf_idx,
                                       srsran_nbiot_bcch_pos_t*       pos);

#endif // SRSRAN_NBIOT_SCHED_H
