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

/******************************************************************************
 *  File:         anchor_fe.h
 *
 *  Description:  Uplink front end for in-band NB-IoT. The eNodeB receives the whole LTE uplink at an LTE sample
 *                rate; the NB-IoT receivers (NPRACH, NPUSCH) want the 180 kHz around the anchor PRB at 1.92 MS/s.
 *                This block mixes the anchor carrier to DC and decimates with a linear-phase low-pass, and it
 *                defines exactly which input sample each output sample is aligned to, so that the timing the
 *                NPRACH detector reports can be mapped back to the LTE timeline without a filter-delay bias.
 *
 *  Anchor carrier offset: for PRB p of an N_PRB carrier the uplink anchor centre is
 *                (2 p + 1 - N_PRB) * 90 kHz from the LTE uplink carrier centre (25 PRB, PRB 17: +900 kHz).
 *****************************************************************************/

#ifndef SRSRAN_ANCHOR_FE_H
#define SRSRAN_ANCHOR_FE_H

#include "srsran/config.h"
#include <stdint.h>

#define SRSRAN_ANCHOR_FE_OUT_RATE_HZ 1920000u

/// Passband and stopband edges of the decimation filter, relative to the anchor carrier. The NB-IoT carrier is
/// +-90 kHz wide; the stopband starts early enough that nothing beyond it can alias into +-100 kHz at 1.92 MS/s.
#define SRSRAN_ANCHOR_FE_PASS_HZ 100000u
#define SRSRAN_ANCHOR_FE_STOP_HZ 300000u
#define SRSRAN_ANCHOR_FE_STOP_ATTEN_DB 70.0

#define SRSRAN_ANCHOR_FE_MAX_RATIO 16u   // 30.72 MS/s (100 PRB) in
#define SRSRAN_ANCHOR_FE_CHUNK 2048u     // output samples processed per pass

typedef struct SRSRAN_API {
  uint32_t fs_in_hz;
  int32_t  offset_hz;  // anchor carrier centre relative to the LTE uplink carrier centre
  uint32_t ratio;      // fs_in / 1.92 MS/s
  uint32_t half_len;   // D: the filter has 2 D + 1 taps
  float*   taps;       // [D + 1] one-sided impulse response, taps[0] is the centre; unity gain at DC
  cf_t*    osc;        // exp(-j 2 pi offset n / fs_in) for one period
  uint32_t osc_period;
  cf_t*    mixed;      // scratch
} srsran_anchor_fe_t;

/**
 * fs_in_hz must be a multiple of 1.92 MS/s up to 16x. offset_hz is the anchor carrier centre relative to the radio
 * centre frequency; |offset_hz| + STOP must lie inside the input band.
 */
SRSRAN_API int srsran_anchor_fe_init(srsran_anchor_fe_t* q, uint32_t fs_in_hz, int32_t offset_hz);

SRSRAN_API void srsran_anchor_fe_free(srsran_anchor_fe_t* q);

/// Input samples on each side of an output sample that the filter reads (its group delay).
SRSRAN_API uint32_t srsran_anchor_fe_delay(const srsran_anchor_fe_t* q);

/// Input samples needed to produce n_out output samples.
SRSRAN_API uint32_t srsran_anchor_fe_nof_input(const srsran_anchor_fe_t* q, uint32_t n_out);

/**
 * x[0] is the input sample with absolute index n0 (only n0 mod the mixer period matters). Output sample j is aligned
 * to input sample x[D + j * ratio], i.e. absolute index n0 + D + j * ratio, where D = srsran_anchor_fe_delay().
 * To get an output stream whose first sample is aligned to absolute input index s, pass x pointing at index s - D and
 * n0 = s - D. srsran_anchor_fe_nof_input(q, n_out) samples are read.
 */
SRSRAN_API int srsran_anchor_fe_run(srsran_anchor_fe_t* q, const cf_t* x, uint64_t n0, cf_t* y, uint32_t n_out);

#endif // SRSRAN_ANCHOR_FE_H
