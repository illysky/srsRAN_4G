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
 *  File:         npusch.h
 *
 *  Description:  NB-IoT narrowband physical uplink shared channel (NPUSCH)
 *                format 1 receiver (UL-SCH, single transport block, no HARQ
 *                combining across transmissions).
 *
 *  Reference:    3GPP TS 36.211 version 14.2.0 clauses 10.1.3 - 10.1.5
 *                3GPP TS 36.212 version 14.2.0 clauses 5.1, 5.2.2 and 6.3.2
 *
 *  The input is the baseband signal at 1.92 MS/s (one sample = 16 Ts) centred on the
 *  NB-IoT uplink carrier (for in-band operation the centre of the anchor PRB), starting at the first
 *  slot of the transmission. The samples must be contiguous: the 40 ms gap after 256 ms and postponement
 *  around NPRACH resources are not handled here, so the caller has to cut them out (and take them into
 *  account for the frame/slot of every codeword repetition, which this receiver derives assuming no gaps).
 *  Timing advance must have been compensated to within a few samples (see NPUSCH_WIN_SHIFT).
 *****************************************************************************/

#ifndef SRSRAN_NPUSCH_H
#define SRSRAN_NPUSCH_H

#include "srsran/config.h"
#include "srsran/phy/common/phy_common.h"
#include "srsran/phy/common/sequence.h"
#include "srsran/phy/dft/dft.h"
#include "srsran/phy/fec/crc.h"
#include "srsran/phy/fec/turbo/turbodecoder.h"
#include <stdbool.h>
#include <stdint.h>

#define SRSRAN_NPUSCH_SRATE_HZ 1920000u
#define SRSRAN_NPUSCH_MAX_RU 10u
#define SRSRAN_NPUSCH_MAX_REP 128u
#define SRSRAN_NPUSCH_MAX_TBS 2536u
#define SRSRAN_NPUSCH_MAX_TONES 12u
/** Slots x tones that fit the work buffers (equal for 1, 3, 6 and 12 tones at the maximum RU count and repetitions) */
#define SRSRAN_NPUSCH_MAX_TONE_SLOTS 30720u
#define SRSRAN_NPUSCH_MAX_CODED_BITS (SRSRAN_NPUSCH_MAX_RU * 144u * 2u)

typedef struct SRSRAN_API {
  uint32_t n_sc;        // 1, 3, 6 or 12 tones
  uint32_t spacing_hz;  // 3750 or 15000 (only 15000 for more than one tone)
  uint32_t sc;          // first sub-carrier index, 0..47 (3.75 kHz) or 0..11 (15 kHz)
  uint32_t n_ru;        // resource units of one repetition
  uint32_t n_rep;       // repetitions (a power of two, 1..128)
  uint32_t rv;          // redundancy version, 0 or 2
  uint32_t qm;          // single tone: 1 (pi/2 BPSK) or 2 (pi/4 QPSK); more tones: 2
  uint32_t tbs;         // transport block size in bits
  uint32_t rnti;        // n_RNTI
  uint32_t cell_id;     // N_ID^Ncell
  uint32_t frame;       // n_f of the first slot
  uint32_t slot;        // n_s of the first slot (0..19 at 15 kHz, 0..4 at 3.75 kHz)
  bool     group_hopping;   // single tone only
  uint32_t delta_ss;        // groupAssignmentNPUSCH
  int32_t  base_seq;        // multi tone base sequence index; negative: derived from the cell id
  uint32_t cyclic_shift;    // threeTone-/sixTone-CyclicShift
  uint32_t chest_window;    // slots averaged by the channel estimator (0: default of 32)
  float    noise_var;       // noise variance per tone (linear, unit power reference); <= 0: estimated from the DMRS
  uint32_t max_iterations;  // turbo decoder iterations (0: default of 10)
} srsran_npusch_cfg_t;

typedef struct SRSRAN_API {
  bool     crc_ok;
  float    snr_db;       // DMRS SNR before combining repetitions (NaN when it could not be estimated)
  float    cfo_hz;       // frequency offset seen by the channel estimator
  float    noise_var;    // noise variance used
  uint32_t nof_iterations;
} srsran_npusch_res_t;

typedef struct SRSRAN_API {
  srsran_dft_plan_t fft_128;   // 15 kHz spacing at 1.92 MS/s
  srsran_dft_plan_t fft_512;   // 3.75 kHz spacing at 1.92 MS/s
  srsran_tdec_t     tdec;
  srsran_crc_t      crc24a;
  srsran_sequence_t seq_dmrs;
  srsran_sequence_t seq_gh;
  srsran_sequence_t seq_scr;

  cf_t*    win;         // FFT input
  cf_t*    spec;        // FFT output
  cf_t*    rot;         // half sub-carrier de-rotation of the FFT window
  cf_t*    y;           // received tones, [slot][symbol][tone]
  cf_t*    h_raw;       // DMRS channel estimates, [slot][tone]
  cf_t*    h_sm;        // smoothed channel estimates
  float*   llr_f;       // accumulated soft bits of one codeword, before the channel de-interleaver
  int16_t* llr_e;
  int16_t* llr_d;       // rate de-matched, 3 * K + 12
  uint8_t* data;        // decoder output, packed bits
} srsran_npusch_t;

SRSRAN_API int  srsran_npusch_init(srsran_npusch_t* q);
SRSRAN_API void srsran_npusch_free(srsran_npusch_t* q);

/** SRSRAN_SUCCESS when the configuration can be received by this implementation */
SRSRAN_API int srsran_npusch_check_cfg(const srsran_npusch_cfg_t* cfg);

/** Slots of the transmission (repetitions included) and samples of one slot / of the whole transmission at 1.92 MS/s */
SRSRAN_API uint32_t srsran_npusch_nof_slots(const srsran_npusch_cfg_t* cfg);
SRSRAN_API uint32_t srsran_npusch_slot_len(const srsran_npusch_cfg_t* cfg);
SRSRAN_API uint32_t srsran_npusch_nof_samples(const srsran_npusch_cfg_t* cfg);

/** Coded bits of one transmission of the codeword (G) */
SRSRAN_API uint32_t srsran_npusch_nof_coded_bits(const srsran_npusch_cfg_t* cfg);

/**
 * Demodulates and decodes one NPUSCH format 1 transmission.
 *
 * @param samples  srsran_npusch_nof_samples(cfg) samples
 * @param tb       receives cfg->tbs bits (one per byte) when the CRC is correct; unspecified otherwise
 * @return SRSRAN_SUCCESS when the receiver ran (see res->crc_ok for the outcome), an error code otherwise
 */
SRSRAN_API int srsran_npusch_decode(srsran_npusch_t*           q,
                                    const srsran_npusch_cfg_t* cfg,
                                    const cf_t*                samples,
                                    uint8_t*                   tb,
                                    srsran_npusch_res_t*       res);

#endif // SRSRAN_NPUSCH_H
