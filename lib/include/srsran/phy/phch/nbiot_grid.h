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
 * \brief Which resource elements of the NB-IoT PRB are taken by reference signals.
 *
 * NPDSCH and NPDCCH are mapped onto the resource elements of the NB-IoT PRB that carry neither NRS nor (in-band) the
 * LTE cell's CRS (TS 36.211 10.2.3.4, 10.2.5.3). Both positions follow one formula, TS 36.211 6.10.1.2 and 10.2.6.2:
 *
 *   k = 6m + (v + v_shift) mod 6, m = 0, 1, v_shift = N_ID mod 6
 *
 * with v depending on the antenna port and symbol. Deriving the mask from the formula once, and using it both to map and
 * to count resource elements, avoids the two ever disagreeing.
 */

#ifndef SRSRAN_NBIOT_GRID_H
#define SRSRAN_NBIOT_GRID_H

#include <stdbool.h>
#include <stdint.h>

#include "srsran/config.h"
#include "srsran/phy/common/phy_common.h"

/// reserved[l][k] is true for the resource elements of symbol l (0..13), subcarrier k (0..11) of the NB-IoT PRB that hold
/// NRS (ports 2000.. for cell->nof_ports 1 or 2) or, in-band, CRS of the LTE cell (cell->base.nof_ports 1, 2 or 4 ports,
/// shifted by cell->base.id). Symbols before the start of NPDSCH/NPDCCH are included; callers ignore them.
SRSRAN_API void srsran_nbiot_reserved_res(const srsran_nbiot_cell_t* cell,
                                          bool                       reserved[SRSRAN_CP_NORM_SF_NSYMB][SRSRAN_NRE]);

/// Number of resource elements per subframe available for NPDSCH/NPDCCH from symbol l_start on
SRSRAN_API uint32_t srsran_nbiot_grid_nof_data_re(const srsran_nbiot_cell_t* cell, uint32_t l_start);

#endif // SRSRAN_NBIOT_GRID_H
