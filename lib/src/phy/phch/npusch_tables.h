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

/*
 * NPUSCH demodulation reference signal tables, 3GPP TS 36.211 v14.2.0.
 *
 *   npusch_w16     Table 10.1.4.1.1-1  w(n) for the single tone reference signal, [u][n mod 16]
 *   npusch_phi_3   Table 10.1.4.1.2-1  phi(n) for 3 tones,  [u = 0..11][n]
 *   npusch_phi_6   Table 10.1.4.1.2-2  phi(n) for 6 tones,  [u = 0..13][n]
 *   npusch_phi_12  Table 10.1.4.1.2-3  phi(n) for 12 tones, [u = 0..29][n]
 *
 * Extracted from the specification text by script and verified value for value against an independently typed copy
 * (OpenAirInterface lte_ul_ref_NB_IoT.c), which was only read for the comparison.
 */

#ifndef SRSRAN_NPUSCH_TABLES_H
#define SRSRAN_NPUSCH_TABLES_H

#include <stdint.h>

static const int8_t npusch_w16[16][16] = {
  {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
  {1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1},
  {1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, -1},
  {1, -1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 1},
  {1, 1, 1, 1, -1, -1, -1, -1, 1, 1, 1, 1, -1, -1, -1, -1},
  {1, -1, 1, -1, -1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, 1},
  {1, 1, -1, -1, -1, -1, 1, 1, 1, 1, -1, -1, -1, -1, 1, 1},
  {1, -1, -1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1},
  {1, 1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1, -1},
  {1, -1, 1, -1, 1, -1, 1, -1, -1, 1, -1, 1, -1, 1, -1, 1},
  {1, 1, -1, -1, 1, 1, -1, -1, -1, -1, 1, 1, -1, -1, 1, 1},
  {1, -1, -1, 1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, 1, -1},
  {1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1, -1, 1, 1, 1, 1},
  {1, -1, 1, -1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, 1, -1},
  {1, 1, -1, -1, -1, -1, 1, 1, -1, -1, 1, 1, 1, 1, -1, -1},
  {1, -1, -1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, -1, 1}};

static const int8_t npusch_phi_3[12][3] = {
  {1, -3, -3},
  {1, -3, -1},
  {1, -3, 3},
  {1, -1, -1},
  {1, -1, 1},
  {1, -1, 3},
  {1, 1, -3},
  {1, 1, -1},
  {1, 1, 3},
  {1, 3, -1},
  {1, 3, 1},
  {1, 3, 3}};

static const int8_t npusch_phi_6[14][6] = {
  {1, 1, 1, 1, 3, -3},
  {1, 1, 3, 1, -3, 3},
  {1, -1, -1, -1, 1, -3},
  {1, -1, 3, -3, -1, -1},
  {1, 3, 1, -1, -1, 3},
  {1, -3, -3, 1, 3, 1},
  {-1, -1, 1, -3, -3, -1},
  {-1, -1, -1, 3, -3, -1},
  {3, -1, 1, -3, -3, 3},
  {3, -1, 3, -3, -1, 1},
  {3, -3, 3, -1, 3, 3},
  {-3, 1, 3, 1, -3, -1},
  {-3, 1, -3, 3, -3, -1},
  {-3, 3, -3, 1, 1, -3}};

static const int8_t npusch_phi_12[30][12] = {
  {-1, 1, 3, -3, 3, 3, 1, 1, 3, 1, -3, 3},
  {1, 1, 3, 3, 3, -1, 1, -3, -3, 1, -3, 3},
  {1, 1, -3, -3, -3, -1, -3, -3, 1, -3, 1, -1},
  {-1, 1, 1, 1, 1, -1, -3, -3, 1, -3, 3, -1},
  {-1, 3, 1, -1, 1, -1, -3, -1, 1, -1, 1, 3},
  {1, -3, 3, -1, -1, 1, 1, -1, -1, 3, -3, 1},
  {-1, 3, -3, -3, -3, 3, 1, -1, 3, 3, -3, 1},
  {-3, -1, -1, -1, 1, -3, 3, -1, 1, -3, 3, 1},
  {1, -3, 3, 1, -1, -1, -1, 1, 1, 3, -1, 1},
  {1, -3, -1, 3, 3, -1, -3, 1, 1, 1, 1, 1},
  {-1, 3, -1, 1, 1, -3, -3, -1, -3, -3, 3, -1},
  {3, 1, -1, -1, 3, 3, -3, 1, 3, 1, 3, 3},
  {1, -3, 1, 1, -3, 1, 1, 1, -3, -3, -3, 1},
  {3, 3, -3, 3, -3, 1, 1, 3, -1, -3, 3, 3},
  {-3, 1, -1, -3, -1, 3, 1, 3, 3, 3, -1, 1},
  {3, -1, 1, -3, -1, -1, 1, 1, 3, 1, -1, -3},
  {1, 3, 1, -1, 1, 3, 3, 3, -1, -1, 3, -1},
  {-3, 1, 1, 3, -3, 3, -3, -3, 3, 1, 3, -1},
  {-3, 3, 1, 1, -3, 1, -3, -3, -1, -1, 1, -3},
  {-1, 3, 1, 3, 1, -1, -1, 3, -3, -1, -3, -1},
  {-1, -3, 1, 1, 1, 1, 3, 1, -1, 1, -3, -1},
  {-1, 3, -1, 1, -3, -3, -3, -3, -3, 1, -1, -3},
  {1, 1, -3, -3, -3, -3, -1, 3, -3, 1, -3, 3},
  {1, 1, -1, -3, -1, -3, 1, -1, 1, 3, -1, 1},
  {1, 1, 3, 1, 3, 3, -1, 1, -1, -3, -3, 1},
  {1, -3, 3, 3, 1, 3, 3, 1, -3, -1, -1, 3},
  {1, 3, -3, -3, 3, -3, 1, -1, -1, 3, -1, -3},
  {-3, -1, -3, -1, -3, 3, 1, -1, 1, 3, -3, -3},
  {-1, 3, -3, 3, -1, 3, 3, -3, 3, 3, -1, -1},
  {3, -3, -3, -1, -1, -3, -1, 3, -3, 3, 1, -1}};

#endif // SRSRAN_NPUSCH_TABLES_H
