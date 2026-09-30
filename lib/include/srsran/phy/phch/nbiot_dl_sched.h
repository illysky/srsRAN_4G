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
 *  File:         nbiot_dl_sched.h
 *
 *  Description:  When an NB-IoT downlink transmission (NPDCCH, NPDSCH addressed to an RNTI) occupies which subframes,
 *                and the table through which the MAC hands such transmissions to the composer.
 *
 *                Timing is done in absolute subframes t = (H-SFN * 1024 + SFN) * 10 + subframe. The functions that
 *                decide where things go (valid downlink subframes, search-space starts, NPDCCH and NPDSCH plans) are
 *                pure; the table is a fixed-size store with a mutex, written by the MAC thread and read by the DL
 *                workers, which build consecutive subframes on different threads.
 *
 *  Reference:    3GPP TS 36.211 v14.2.0 10.2.3.4, 10.2.5
 *                3GPP TS 36.213 v14.2.0 16.4.1 (k0, "NB-IoT DL subframe"), 16.6 (search spaces, Tables 16.6-1, 16.6-3)
 *                3GPP TS 36.321 v14.2.0 5.1.4 (random access response window)
 *****************************************************************************/

#ifndef SRSRAN_NBIOT_DL_SCHED_H
#define SRSRAN_NBIOT_DL_SCHED_H

#include <stdbool.h>
#include <stdint.h>

#include "srsran/config.h"
#include "srsran/phy/phch/nbiot_dlch.h"
#include "srsran/phy/phch/nbiot_sched.h"
#include "srsran/phy/phch/npbch.h"

#define SRSRAN_NBIOT_SF_PER_HFN 10240u

#define SRSRAN_NBIOT_MAX_SI 8

/// Absolute subframe of (hfn, sfn, sf)
static inline uint64_t srsran_nbiot_abs_sf(uint32_t hfn, uint32_t sfn, uint32_t sf)
{
  return ((uint64_t)hfn * 1024 + sfn) * 10 + sf;
}

/* --------------------------------------------------------------------------------- what the cell broadcasts where */

typedef struct SRSRAN_API {
  uint32_t                 cell_id;
  srsran_mib_nb_t          mib;
  bool                     sib1_set;
  srsran_nbiot_si_params_t si[SRSRAN_NBIOT_MAX_SI];
  bool                     si_set[SRSRAN_NBIOT_MAX_SI];
} srsran_nbiot_layout_t;

/**
 * True if the absolute subframe t is an NB-IoT downlink subframe on which NPDCCH or NPDSCH may be counted: not NPSS
 * (subframe 5), NSSS (subframe 9 of even frames), NPBCH (subframe 0), SIB1-NB, and not one used for an SI message (TS
 * 36.213 16.4, 16.6). The cell has no downlinkBitmap, so nothing else is excluded.
 */
SRSRAN_API bool srsran_nbiot_layout_is_dl_sf(const srsran_nbiot_layout_t* l, uint64_t t);

/// First NB-IoT downlink subframe at or after t
SRSRAN_API uint64_t srsran_nbiot_layout_next_dl_sf(const srsran_nbiot_layout_t* l, uint64_t t);

/* -------------------------------------------------------------------------------------------------------- plans */

#define SRSRAN_NBIOT_PLAN_MAX_SF 64

/// The subframes of one transmission, in the order they are sent, and what each carries
typedef struct SRSRAN_API {
  uint32_t nof_sf;
  uint64_t t[SRSRAN_NBIOT_PLAN_MAX_SF];
  // NPDCCH: subframe number that (re)initialised the scrambler and the position in that group of four
  uint8_t reinit_sf[SRSRAN_NBIOT_PLAN_MAX_SF];
  uint8_t pos_in_group[SRSRAN_NBIOT_PLAN_MAX_SF];
  // NPDSCH: part of the codeword, and the radio frame / subframe number that started the pass it belongs to
  uint8_t  sf_in_cw[SRSRAN_NBIOT_PLAN_MAX_SF];
  uint8_t  pass_sf[SRSRAN_NBIOT_PLAN_MAX_SF];
  uint16_t pass_sfn[SRSRAN_NBIOT_PLAN_MAX_SF];
} srsran_nbiot_plan_t;

/// DCI subframe repetition number field (Tables 16.6-1, 16.6-3) of the candidate with R = r_max, or -1
SRSRAN_API int srsran_nbiot_dci_rep_for_rmax(uint32_t r_max);

/**
 * First subframe k0 >= t_min of a search space with period T = r_max * g / 2 (g = npdcch-StartSF in halves: 3 is 1.5,
 * 4 is 2, 8 is 4 ...) and offset alpha = offset_eighths / 8: (10 n_f + floor(n_s / 2)) mod T = floor(alpha T)
 * (TS 36.213 16.6). Returns 0 with *period_sf = 0 if the parameters make no search space (T < 4).
 */
SRSRAN_API uint64_t srsran_nbiot_search_space_start(uint32_t r_max,
                                                    uint32_t g_halves,
                                                    uint32_t offset_eighths,
                                                    uint64_t t_min,
                                                    uint32_t* period_sf);

/**
 * NPDCCH candidate of R = r consecutive NB-IoT downlink subframes beginning with the first one at or after the search
 * space start k0 (b = 0). Only candidates that begin at a multiple of four subframes into the search space are planned
 * (r >= 4 or r = 1 at b = 0), so the "after every fourth NPDCCH subframe" reinitialisation of the scrambler is
 * unambiguous. Fails if r is 0 or larger than the plan holds.
 */
SRSRAN_API int
srsran_nbiot_plan_npdcch(const srsran_nbiot_layout_t* l, uint64_t k0, uint32_t r, srsran_nbiot_plan_t* plan);

/**
 * NPDSCH described by a DCI N1 that ended in subframe n_last: starts k0 NB-IoT downlink subframes after the first one at
 * or after n_last + 5 (TS 36.213 16.4.1), takes n_sf * n_rep of them in a row; each subframe of the codeword is sent
 * min(n_rep, 4) times before the next one and the whole pass is repeated n_rep / min(n_rep, 4) times (36.211
 * 10.2.3.4). Fails if it does not fit the plan (at most SRSRAN_NBIOT_PLAN_MAX_SF subframes).
 */
SRSRAN_API int srsran_nbiot_plan_npdsch(const srsran_nbiot_layout_t* l,
                                        uint64_t                     n_last,
                                        uint32_t                     k0,
                                        uint32_t                     n_sf,
                                        uint32_t                     n_rep,
                                        srsran_nbiot_plan_t*         plan);

/* ---------------------------------------------------------------------------------------- the table for the composer */

#define SRSRAN_NBIOT_DL_SCHED_SLOTS 4096
#define SRSRAN_NBIOT_DL_SCHED_OBJECTS 32
#define SRSRAN_NBIOT_DL_SCHED_MAX_E (2 * SRSRAN_NBIOT_DLCH_MAX_RE * SRSRAN_NBIOT_DLCH_MAX_NSF)

typedef enum { SRSRAN_NBIOT_TX_NONE = 0, SRSRAN_NBIOT_TX_NPDCCH, SRSRAN_NBIOT_TX_NPDSCH } srsran_nbiot_tx_kind_t;

/// What the composer has to put into one subframe
typedef struct SRSRAN_API {
  srsran_nbiot_tx_kind_t kind;
  uint8_t                reinit_sf, pos_in_group;           // NPDCCH
  uint16_t               rnti, pass_sfn;                     // NPDSCH
  uint8_t                nof_sf, sf_in_cw, pass_sf;          // NPDSCH
  uint32_t               e_len;
  uint8_t                e[SRSRAN_NBIOT_DL_SCHED_MAX_E];
} srsran_nbiot_dl_tx_t;

typedef struct srsran_nbiot_dl_sched_st srsran_nbiot_dl_sched_t;

SRSRAN_API srsran_nbiot_dl_sched_t* srsran_nbiot_dl_sched_new(void);
SRSRAN_API void                     srsran_nbiot_dl_sched_free(srsran_nbiot_dl_sched_t* s);

/**
 * Stores an NPDCCH transmission: the coded bits (E per subframe) and the subframes to send them in. Fails, storing
 * nothing, if one of the subframes is taken or in the past, or no object slot is free. 'now' is the latest subframe
 * the composer has been asked for; everything before now + min_lead is refused (it may already be on the air).
 */
SRSRAN_API int srsran_nbiot_dl_sched_add_npdcch(srsran_nbiot_dl_sched_t* s,
                                                const uint8_t*           e,
                                                uint32_t                 e_len,
                                                const srsran_nbiot_plan_t* plan,
                                                uint64_t                 min_t);

/// Same for an NPDSCH: e holds the whole codeword (n_sf * E bits)
SRSRAN_API int srsran_nbiot_dl_sched_add_npdsch(srsran_nbiot_dl_sched_t* s,
                                                const uint8_t*           e,
                                                uint32_t                 e_len,
                                                uint16_t                 rnti,
                                                uint32_t                 n_sf,
                                                const srsran_nbiot_plan_t* plan,
                                                uint64_t                 min_t);

/// True if a transmission is scheduled for subframe t; fills tx. Thread safe. Marks t as the latest subframe seen.
SRSRAN_API bool srsran_nbiot_dl_sched_get(srsran_nbiot_dl_sched_t* s, uint64_t t, srsran_nbiot_dl_tx_t* tx);

/// The latest subframe the composer has asked about (0 until it did)
SRSRAN_API uint64_t srsran_nbiot_dl_sched_now(srsran_nbiot_dl_sched_t* s);

/// True if subframe t already carries a transmission
SRSRAN_API bool srsran_nbiot_dl_sched_busy(srsran_nbiot_dl_sched_t* s, uint64_t t);

#endif // SRSRAN_NBIOT_DL_SCHED_H
