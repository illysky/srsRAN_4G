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
 * \brief NB-IoT downlink of an in-band (same PCI) eNB: writes one NB-IoT anchor PRB into an LTE resource grid.
 *
 * The LTE downlink is built as usual (srsran_enb_dl_t: CRS, PCFICH, PHICH, PDCCH, PDSCH, PSS/SSS, PBCH). This module
 * then takes over one PRB -- the NB-IoT anchor -- for the symbols after the LTE control region and fills it with what
 * an NB-IoT UE looks for: NPSS (subframe 5), NSSS (subframe 9 of even frames), NRS, NPBCH (subframe 0), SIB1-NB (subframe
 * 4) and the SI messages. The LTE CRS stay where they are, because LTE UEs depend on them.
 *
 * It is stateless with respect to time: srsran_enb_dl_nbiot_put_sf() is a function of (H-SFN, SFN, subframe) and of the
 * configuration only, so subframes can be produced in any order. That matters because srsenb builds consecutive
 * subframes on different worker threads. One instance per worker; an instance is not reentrant (it owns scratch
 * buffers), but it needs no synchronisation with other instances.
 *
 * Scope: in-band same-PCI, ONE antenna port (LTE and NB-IoT), normal CP, eutraControlRegionSize 3, Rel-14 conventions
 * (rotated NPBCH, SIB1/SI scrambling of TS 36.211 10.2.3.1). Anything else is refused by init(): two-port transmit
 * diversity exists in the library but has not been checked against the specification, so it is not offered.
 *
 * The LTE scheduler must keep LTE PDSCH off the anchor PRB. If it does not, the LTE data there is overwritten and
 * counted in lte_collisions rather than silently lost.
 */

#ifndef SRSRAN_ENB_DL_NBIOT_H
#define SRSRAN_ENB_DL_NBIOT_H

#include <stdbool.h>
#include <stdint.h>

#include "srsran/config.h"
#include "srsran/phy/ch_estimation/refsignal_dl_nbiot.h"
#include "srsran/phy/common/phy_common.h"
#include "srsran/phy/fec/softbuffer.h"
#include "srsran/phy/phch/nbiot_dl_sched.h"
#include "srsran/phy/phch/nbiot_dlch.h"
#include "srsran/phy/phch/nbiot_sched.h"
#include "srsran/phy/phch/npbch.h"
#include "srsran/phy/phch/npdsch.h"
#include "srsran/phy/phch/ra_nbiot.h"
#include "srsran/phy/sync/npss.h"
#include "srsran/phy/sync/nsss.h"

#define SRSRAN_ENB_DL_NBIOT_MAX_SI 8
#define SRSRAN_ENB_DL_NBIOT_MAX_BCCH_BYTES (SRSRAN_NPDSCH_MAX_TBS / 8)

/// First OFDM symbol of the anchor PRB that belongs to NB-IoT (eutraControlRegionSize, fixed at 3)
#define SRSRAN_ENB_DL_NBIOT_L_START 3

/// What srsran_enb_dl_nbiot_put_sf() placed in the subframe (bit mask)
#define SRSRAN_ENB_DL_NBIOT_HAS_NPSS (1u << 0)
#define SRSRAN_ENB_DL_NBIOT_HAS_NSSS (1u << 1)
#define SRSRAN_ENB_DL_NBIOT_HAS_NRS (1u << 2)
#define SRSRAN_ENB_DL_NBIOT_HAS_NPBCH (1u << 3)
#define SRSRAN_ENB_DL_NBIOT_HAS_SIB1 (1u << 4)
#define SRSRAN_ENB_DL_NBIOT_HAS_SI (1u << 5)
#define SRSRAN_ENB_DL_NBIOT_HAS_NPDCCH (1u << 6)
#define SRSRAN_ENB_DL_NBIOT_HAS_NPDSCH (1u << 7)

// CRS REs of one PRB after the LTE control region: ports 0/1 use symbols 4, 7, 11; ports 2/3 use symbol 8; two each
#define SRSRAN_ENB_DL_NBIOT_MAX_CRS_RE (2 * (2 * 3 + 2 * 1))

typedef struct SRSRAN_API {
  srsran_nbiot_cell_t cell;
  uint32_t            lte_nof_ports;

  srsran_mib_nb_t mib;
  bool            mib_set;

  uint8_t  sib1[SRSRAN_ENB_DL_NBIOT_MAX_BCCH_BYTES];
  uint32_t sib1_bytes;
  bool     sib1_set;

  srsran_nbiot_si_params_t si_params[SRSRAN_ENB_DL_NBIOT_MAX_SI];
  uint8_t                  si[SRSRAN_ENB_DL_NBIOT_MAX_SI][SRSRAN_ENB_DL_NBIOT_MAX_BCCH_BYTES];
  uint32_t                 si_bytes[SRSRAN_ENB_DL_NBIOT_MAX_SI];
  bool                     si_set[SRSRAN_ENB_DL_NBIOT_MAX_SI];

  cf_t                        npss[SRSRAN_NPSS_TOT_LEN];
  cf_t                        nsss[SRSRAN_NSSS_TOT_LEN];
  srsran_refsignal_dl_nbiot_t nrs;
  srsran_npbch_t              npbch;
  srsran_npdsch_t             npdsch;
  srsran_softbuffer_tx_t      softbuffer;

  // Addressed traffic (NPDCCH, NPDSCH with an RNTI): the MAC puts it into a table shared by all composers, which looks
  // the subframe up. NULL: none.
  srsran_nbiot_dlch_t      dlch;
  srsran_nbiot_dl_sched_t* dl_sched;
  srsran_nbiot_dl_tx_t     dyn; // scratch for the entry found
  // Entries of the table that fell into a subframe the composer needed for something else, and were dropped
  uint64_t dyn_conflicts;

  // LTE CRS REs inside the anchor PRB after the control region: grid index, and the LTE port that owns each
  uint32_t nof_crs_re;
  uint32_t crs_idx[SRSRAN_ENB_DL_NBIOT_MAX_CRS_RE];
  uint32_t crs_port[SRSRAN_ENB_DL_NBIOT_MAX_CRS_RE];

  // Number of subframes in which LTE had put something other than CRS into the anchor PRB (and lost it)
  uint64_t lte_collisions;
} srsran_enb_dl_nbiot_t;

/// Returns SRSRAN_SUCCESS, or an error if the cell is outside the scope above (a message is printed)
SRSRAN_API int srsran_enb_dl_nbiot_init(srsran_enb_dl_nbiot_t* q, const srsran_nbiot_cell_t* cell);

SRSRAN_API void srsran_enb_dl_nbiot_free(srsran_enb_dl_nbiot_t* q);

/// MIB-NB content (the H-SFN and SFN fields are filled in per subframe)
SRSRAN_API int srsran_enb_dl_nbiot_set_mib(srsran_enb_dl_nbiot_t* q, const srsran_mib_nb_t* mib);

/// SIB1-NB as a transport block: exactly TBS(schedulingInfoSIB1)/8 bytes. The hyperSFN-MSB field it contains is the
/// caller's business: call again when the H-SFN's upper eight bits change. Needs the MIB first.
SRSRAN_API int srsran_enb_dl_nbiot_set_sib1(srsran_enb_dl_nbiot_t* q, const uint8_t* payload, uint32_t bytes);

/// SI message idx (0-based, the order in schedulingInfoList) with its scheduling; payload is si_tb/8 bytes.
/// p->n is set to idx + 1 here.
SRSRAN_API int srsran_enb_dl_nbiot_set_si(srsran_enb_dl_nbiot_t*       q,
                                          uint32_t                     idx,
                                          const srsran_nbiot_si_params_t* p,
                                          const uint8_t*               payload,
                                          uint32_t                     bytes);

/// Lets the composer send what the MAC has put into 'sched' (which must outlive it). Call before the first subframe.
SRSRAN_API void srsran_enb_dl_nbiot_set_sched(srsran_enb_dl_nbiot_t* q, srsran_nbiot_dl_sched_t* sched);

/// What the cell broadcasts where, for planning transmissions that must stay clear of it. Needs the MIB.
SRSRAN_API int srsran_enb_dl_nbiot_get_layout(const srsran_enb_dl_nbiot_t* q, srsran_nbiot_layout_t* layout);

/// Writes the NB-IoT anchor PRB of subframe sf_idx of radio frame sfn (hyperframe hfn, which may exceed 1023: the
/// broadcast uses its 10 LSBs and the scheduling table the absolute subframe) into sf_symbols, one grid per
/// LTE port. The grids hold the LTE signal on entry. Returns a mask of SRSRAN_ENB_DL_NBIOT_HAS_*, or a negative error.
SRSRAN_API int srsran_enb_dl_nbiot_put_sf(srsran_enb_dl_nbiot_t* q,
                                          uint32_t               hfn,
                                          uint32_t               sfn,
                                          uint32_t               sf_idx,
                                          cf_t*                  sf_symbols[SRSRAN_MAX_PORTS]);

#endif // SRSRAN_ENB_DL_NBIOT_H
