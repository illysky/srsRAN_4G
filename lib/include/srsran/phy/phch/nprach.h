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
 *  File:         nprach.h
 *
 *  Description:  NB-IoT narrowband physical random access channel (NPRACH):
 *                preamble generation (used by tests and a future UE side) and
 *                the eNodeB preamble detector.
 *
 *  Reference:    3GPP TS 36.211 version 14.2.0 Release 14 clause 10.1.6
 *
 *  All time domain samples handled here are at 1.92 MS/s, the rate at which
 *  one NPRACH symbol (1 / 3.75 kHz) is exactly 512 samples and one sample is
 *  16 Ts. The signal must be centred on the NB-IoT uplink carrier (the centre
 *  of the anchor PRB for in-band operation).
 *****************************************************************************/

#ifndef SRSRAN_NPRACH_H
#define SRSRAN_NPRACH_H

#include "srsran/config.h"
#include "srsran/phy/dft/dft.h"
#include <stdbool.h>
#include <stdint.h>

#define SRSRAN_NPRACH_SRATE_HZ 1920000u
#define SRSRAN_NPRACH_N_FFT 512u            // samples per NPRACH symbol (1 / 3.75 kHz)
#define SRSRAN_NPRACH_N_SC_RA 12u           // hopping range (N_sc^RA)
#define SRSRAN_NPRACH_MAX_SC 48u            // N_sc^UL-derived maximum (nprach-NumSubcarriers)
#define SRSRAN_NPRACH_SYMS_PER_GROUP 5u
#define SRSRAN_NPRACH_GROUPS_PER_REP 4u
#define SRSRAN_NPRACH_REPS_PER_GAP 64u      // a 40 ms gap follows every 64 repetitions
#define SRSRAN_NPRACH_GAP_LEN (SRSRAN_NPRACH_SRATE_HZ / 25u) // 40 ms
#define SRSRAN_NPRACH_MAX_REP 128u
#define SRSRAN_NPRACH_MAX_GROUPS (SRSRAN_NPRACH_GROUPS_PER_REP * SRSRAN_NPRACH_MAX_REP)
#define SRSRAN_NPRACH_MAX_DET 48u

/** Default detection threshold for n_rep repetitions (false alarm rate of about 1e-6 per starting subcarrier). */
SRSRAN_API float srsran_nprach_default_threshold(uint32_t n_rep);

typedef struct SRSRAN_API {
  uint32_t cell_id;     // N_ID^Ncell, seeds the hopping sequence
  uint32_t format;      // preamble format 0 (CP 66.7 us) or 1 (CP 266.7 us)
  uint32_t n_rep;       // repetitions of the 4-symbol-group block (numRepetitionsPerPreambleAttempt)
  uint32_t n_sc_offset; // nprach-SubcarrierOffset
  uint32_t n_sc_nprach; // nprach-NumSubcarriers (12, 24, 36 or 48)
} srsran_nprach_cfg_t;

typedef struct SRSRAN_API {
  uint32_t n_init;  // starting subcarrier (the preamble index)
  float    toa;     // arrival time relative to the nominal preamble start, in samples (16 Ts)
  float    cfo_hz;  // residual carrier frequency offset
  float    metric;  // correlation peak relative to the expected noise-only level (dimensionless, grows with SNR)
  float    peak;    // correlation peak power (used to reject weak partial matches of a stronger preamble)
  bool     detected;
} srsran_nprach_det_t;

/* Helpers (all in samples at 1.92 MS/s) */
SRSRAN_API uint32_t srsran_nprach_cp_len(uint32_t format);
SRSRAN_API uint32_t srsran_nprach_group_len(uint32_t format);
SRSRAN_API uint32_t srsran_nprach_group_start(const srsran_nprach_cfg_t* cfg, uint32_t group);
SRSRAN_API uint32_t srsran_nprach_nof_samples(const srsran_nprach_cfg_t* cfg);
SRSRAN_API int      srsran_nprach_check_cfg(const srsran_nprach_cfg_t* cfg);

/** Frequency hopping of clause 10.1.6.1: n~_sc^RA(i) for i = 0 .. nof_groups-1 (each 0..11). */
SRSRAN_API int srsran_nprach_hops(uint32_t cell_id, uint32_t n_init, uint32_t nof_groups, uint8_t* hop);

/**
 * Generates the baseband preamble for starting subcarrier n_init (clause 10.1.6.2) at unit
 * amplitude. 'out' must hold srsran_nprach_nof_samples(cfg) samples.
 */
SRSRAN_API int srsran_nprach_gen(const srsran_nprach_cfg_t* cfg, uint32_t n_init, cf_t* out);

typedef struct SRSRAN_API {
  srsran_nprach_cfg_t cfg;
  srsran_dft_plan_t   fft;

  // Search grid
  uint32_t n_tau;
  float    tau_min;      // first ToA hypothesis, in samples
  uint32_t n_eps;
  float    eps_step_hz;  // CFO hypotheses are (i - n_eps/2) * eps_step_hz
  float    threshold;    // detection threshold on 'metric' (calibrated per n_rep, see nprach_test -E)
  float    ghost_ratio;  // a detection weaker than this fraction of the strongest one is a partial match
  float    noise_var;    // noise power per FFT bin used by the search; <= 0: fall back to the surface median

  // Tables
  cf_t*  shift;          // exp(-j pi i / 512), i < 2560
  cf_t*  tw_tau;         // [n_tau][12]  exp(+j 2 pi k tau / 512)
  cf_t*  tw_eps;         // [n_eps][20]  exp(-j 2 pi eps t)
  double tw_eps_period;  // group period (in symbols) the CFO table was built for

  // Work buffers
  cf_t*    bins;         // [groups][5][48] per-symbol FFT bins of the NPRACH subcarriers
  cf_t*    win;          // [512]
  cf_t*    spec;         // [512]
  cf_t*    z;            // [4][n_eps]
  float*   surf;         // [n_tau][n_eps]
  float*   sel;          // [n_tau][n_eps] scratch for the median
  uint8_t* hop;          // [max groups]
  cf_t*    y;            // [max groups][5]
} srsran_nprach_t;

SRSRAN_API int  srsran_nprach_init(srsran_nprach_t* q, const srsran_nprach_cfg_t* cfg);
SRSRAN_API void srsran_nprach_free(srsran_nprach_t* q);

/**
 * Detects preambles in 'x' (srsran_nprach_nof_samples() samples at 1.92 MS/s, starting at the nominal
 * preamble start). Every starting subcarrier is tested; each entry of 'det' (one per n_init,
 * n_sc_nprach entries) reports the best ToA/CFO and whether it exceeds the threshold.
 * Returns the number of detections.
 */
SRSRAN_API int srsran_nprach_detect(srsran_nprach_t* q, const cf_t* x, srsran_nprach_det_t* det);

/**
 * Core search on already-demodulated symbols. y holds nof_groups * 5 values (the FFT output of every
 * symbol at the hopping subcarrier), hop the per-group hopping index 0..11. group_period_syms is the time
 * from the start of one group's FFT window to the next, in NPRACH symbols (5.25 for format 0, 6 for
 * format 1, 5 if the caller has removed the cyclic prefixes). Exposed so the search can be verified
 * against independent test data.
 */
SRSRAN_API void srsran_nprach_search(srsran_nprach_t* q,
                                     const cf_t*      y,
                                     const uint8_t*   hop,
                                     uint32_t         nof_groups,
                                     double           group_period_syms,
                                     srsran_nprach_det_t* out);

#endif // SRSRAN_NPRACH_H
