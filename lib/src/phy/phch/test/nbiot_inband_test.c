/*
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
 * Checks the in-band NB-IoT signalling helpers against the specifications:
 *  - eutra-CRS-SequenceInfo-r13 <-> anchor PRB (TS 36.213 Table 16.8-1)
 *  - MIB-NB operationModeInfo-r13 bit layout (TS 36.331 MasterInformationBlock-NB)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "srsran/phy/common/phy_common.h"
#include "srsran/phy/phch/npbch.h"
#include "srsran/phy/utils/bit.h"

#define CHECK(cond, ...)                                                                                               \
  do {                                                                                                                 \
    if (!(cond)) {                                                                                                     \
      fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__);                                                             \
      fprintf(stderr, __VA_ARGS__);                                                                                    \
      fprintf(stderr, "\n");                                                                                           \
      return SRSRAN_ERROR;                                                                                             \
    }                                                                                                                  \
  } while (0)

// Legal in-band SamePCI anchor PRBs per LTE bandwidth. Taken from the per-bandwidth lists published for TS 36.213
// Table 16.8-1 (a different source from the table itself), so agreement with the code is a genuine cross-check.
struct bw_case {
  uint32_t       nof_prb;
  uint32_t       nof_expected;
  const uint32_t prbs[18];
};

static const struct bw_case cases[] = {
    {6, 0, {0}},
    {15, 2, {2, 12}},
    {25, 4, {2, 7, 17, 22}},
    {50, 8, {4, 9, 14, 19, 30, 35, 40, 45}},
    {75, 14, {2, 7, 12, 17, 22, 27, 32, 42, 47, 52, 57, 62, 67, 72}},
    {100, 18, {4, 9, 14, 19, 24, 29, 34, 39, 44, 55, 60, 65, 70, 75, 80, 85, 90, 95}},
};

static int test_table(void)
{
  for (size_t c = 0; c < sizeof(cases) / sizeof(cases[0]); c++) {
    const struct bw_case* bw = &cases[c];

    // every PRB that maps to a value must be in the expected set, and vice versa
    uint32_t found = 0;
    for (uint32_t prb = 0; prb < bw->nof_prb; prb++) {
      uint8_t info;
      int     ret      = srsran_nbiot_prb_to_crs_seq_info(bw->nof_prb, prb, &info, NULL);
      bool    expected = false;
      for (uint32_t i = 0; i < bw->nof_expected; i++) {
        expected |= (bw->prbs[i] == prb);
      }
      CHECK((ret == SRSRAN_SUCCESS) == expected,
            "N_RB=%d PRB %d: code says %s, expected %s",
            bw->nof_prb,
            prb,
            ret == SRSRAN_SUCCESS ? "legal" : "illegal",
            expected ? "legal" : "illegal");
      if (ret == SRSRAN_SUCCESS) {
        found++;
        // and the inverse must return the same PRB
        uint32_t back;
        CHECK(srsran_nbiot_crs_seq_info_to_prb(bw->nof_prb, info, &back, NULL) == SRSRAN_SUCCESS && back == prb,
              "N_RB=%d PRB %d: round trip via info %d gave %d",
              bw->nof_prb,
              prb,
              info,
              back);
      }
    }
    CHECK(found == bw->nof_expected, "N_RB=%d: found %d legal PRBs, expected %d", bw->nof_prb, found, bw->nof_expected);
  }
  return SRSRAN_SUCCESS;
}

static int test_worked_examples(void)
{
  uint32_t                     prb;
  uint8_t                      info;
  srsran_nbiot_raster_offset_t raster;

  // Worked example from the spec table: 5 MHz carrier (odd N_RB, floor(25/2)=12), info 7 -> n'=5 -> PRB 17, +7.5 kHz
  CHECK(srsran_nbiot_crs_seq_info_to_prb(25, 7, &prb, &raster) == SRSRAN_SUCCESS, "info 7 rejected for 25 PRB");
  CHECK(prb == 17, "info 7 @25 PRB -> PRB %d, expected 17", prb);
  CHECK(raster == SRSRAN_NBIOT_RASTER_OFFSET_P7DOT5_KHZ, "info 7 raster offset %d, expected +7.5 kHz", raster);

  // The four legal 5 MHz anchors and their infos
  const uint32_t prbs[4]  = {2, 7, 17, 22};
  const uint8_t  infos[4] = {5, 6, 7, 8};
  for (int i = 0; i < 4; i++) {
    CHECK(srsran_nbiot_prb_to_crs_seq_info(25, prbs[i], &info, &raster) == SRSRAN_SUCCESS, "PRB %d rejected", prbs[i]);
    CHECK(info == infos[i], "PRB %d -> info %d, expected %d", prbs[i], info, infos[i]);
  }

  // Raster offsets: PRB 2 and 7 are -7.5 kHz, PRB 17 and 22 are +7.5 kHz (odd table)
  srsran_nbiot_prb_to_crs_seq_info(25, 7, &info, &raster);
  CHECK(raster == SRSRAN_NBIOT_RASTER_OFFSET_M7DOT5_KHZ, "PRB 7 raster %d, expected -7.5 kHz", raster);
  srsran_nbiot_prb_to_crs_seq_info(25, 22, &info, &raster);
  CHECK(raster == SRSRAN_NBIOT_RASTER_OFFSET_P7DOT5_KHZ, "PRB 22 raster %d, expected +7.5 kHz", raster);

  // Even table: 10 MHz, info 19 -> n'=-21 -> PRB 4 (raster +2.5 kHz); info 23 -> n'=5 -> PRB 30 (-2.5 kHz)
  CHECK(srsran_nbiot_crs_seq_info_to_prb(50, 19, &prb, &raster) == SRSRAN_SUCCESS && prb == 4, "10 MHz info 19");
  CHECK(raster == SRSRAN_NBIOT_RASTER_OFFSET_P2DOT5_KHZ, "10 MHz info 19 raster");
  CHECK(srsran_nbiot_crs_seq_info_to_prb(50, 23, &prb, &raster) == SRSRAN_SUCCESS && prb == 30, "10 MHz info 23");
  CHECK(raster == SRSRAN_NBIOT_RASTER_OFFSET_M2DOT5_KHZ, "10 MHz info 23 raster");

  // Values defined for the other parity, and out-of-range values, must be rejected
  CHECK(srsran_nbiot_crs_seq_info_to_prb(25, 20, &prb, NULL) != SRSRAN_SUCCESS, "even-table info accepted for odd N_RB");
  CHECK(srsran_nbiot_crs_seq_info_to_prb(50, 7, &prb, NULL) != SRSRAN_SUCCESS, "odd-table info accepted for even N_RB");
  CHECK(srsran_nbiot_crs_seq_info_to_prb(25, 0, &prb, NULL) != SRSRAN_SUCCESS, "info 0 (n'=-35) must fall outside 25 PRB");
  CHECK(srsran_nbiot_crs_seq_info_to_prb(25, 32, &prb, NULL) != SRSRAN_SUCCESS, "info 32 accepted");

  // PRB 0 is not an anchor in a 25 PRB carrier, and PRB 25 is out of range altogether
  CHECK(srsran_nbiot_prb_to_crs_seq_info(25, 0, &info, NULL) != SRSRAN_SUCCESS, "PRB 0 accepted");
  CHECK(srsran_nbiot_prb_to_crs_seq_info(25, 25, &info, NULL) != SRSRAN_SUCCESS, "PRB 25 accepted");
  return SRSRAN_SUCCESS;
}

static int test_prb_isvalid(void)
{
  // Regression: nbiot_prb == nof_prb used to be accepted (off-by-one)
  srsran_nbiot_cell_t cell = {};
  cell.base.nof_prb        = 25;
  cell.nbiot_prb           = 24;
  CHECK(srsran_nbiot_prb_isvalid(&cell), "PRB 24 of 25 must be valid");
  cell.nbiot_prb = 25;
  CHECK(!srsran_nbiot_prb_isvalid(&cell), "PRB 25 of 25 must be invalid");
  return SRSRAN_SUCCESS;
}

static int test_mib_layout(void)
{
  uint8_t          payload[SRSRAN_MIB_NB_LEN];
  srsran_mib_nb_t  mib = {};
  srsran_mib_nb_t  out = {};

  mib.sched_info_sib1 = 2;
  mib.sys_info_tag    = 9;
  mib.ac_barring      = true;

  // ---- in-band, same PCI: bits 16..17 = choice index 0, bits 18..22 = eutra-CRS-SequenceInfo
  mib.mode               = SRSRAN_NBIOT_MODE_INBAND_SAME_PCI;
  mib.eutra_crs_seq_info = 7;
  srsran_npbch_mib_pack(0, 0, mib, payload);
  const uint8_t* p = payload; // one bit per byte
  // 4 SFN MSB + 2 HFN LSB + 4 schedInfo + 5 valueTag + 1 ab = 16 bits precede operationModeInfo
  uint8_t choice = (p[16] << 1) | p[17];
  CHECK(choice == 0, "in-band same-PCI choice index %d, expected 0", choice);
  uint8_t crs = 0;
  for (int i = 0; i < 5; i++) {
    crs = (crs << 1) | p[18 + i];
  }
  CHECK(crs == 7, "eutra-CRS-SequenceInfo packed as %d, expected 7", crs);
  // everything after the 23 defined bits must be spare = 0 (r15+ fields left at defaults)
  for (int i = 23; i < SRSRAN_MIB_NB_LEN; i++) {
    CHECK(p[i] == 0, "bit %d should be spare/zero", i);
  }

  srsran_npbch_mib_unpack(payload, &out);
  CHECK(out.mode == SRSRAN_NBIOT_MODE_INBAND_SAME_PCI, "unpacked mode %d", out.mode);
  CHECK(out.eutra_crs_seq_info == 7, "unpacked eutra-CRS-SequenceInfo %d", out.eutra_crs_seq_info);
  CHECK(out.sched_info_sib1 == 2 && out.sys_info_tag == 9 && out.ac_barring, "common MIB fields not preserved");

  // ---- in-band, different PCI: NumCRS-Ports(1) rasterOffset(2)
  memset(&mib, 0, sizeof(mib));
  mib.mode                     = SRSRAN_NBIOT_MODE_INBAND_DIFFERENT_PCI;
  mib.eutra_num_crs_ports_four = true;
  mib.raster_offset            = SRSRAN_NBIOT_RASTER_OFFSET_P2DOT5_KHZ;
  srsran_npbch_mib_pack(0, 0, mib, payload);
  memset(&out, 0, sizeof(out));
  srsran_npbch_mib_unpack(payload, &out);
  CHECK(out.mode == SRSRAN_NBIOT_MODE_INBAND_DIFFERENT_PCI, "different-PCI mode %d", out.mode);
  CHECK(out.eutra_num_crs_ports_four, "eutra-NumCRS-Ports lost");
  CHECK(out.raster_offset == SRSRAN_NBIOT_RASTER_OFFSET_P2DOT5_KHZ, "raster offset %d", out.raster_offset);

  // ---- guard band: rasterOffset(2)
  memset(&mib, 0, sizeof(mib));
  mib.mode          = SRSRAN_NBIOT_MODE_GUARDBAND;
  mib.raster_offset = SRSRAN_NBIOT_RASTER_OFFSET_M2DOT5_KHZ;
  srsran_npbch_mib_pack(0, 0, mib, payload);
  memset(&out, 0, sizeof(out));
  srsran_npbch_mib_unpack(payload, &out);
  CHECK(out.mode == SRSRAN_NBIOT_MODE_GUARDBAND && out.raster_offset == SRSRAN_NBIOT_RASTER_OFFSET_M2DOT5_KHZ,
        "guard-band round trip");

  // ---- standalone: config bits are all spare, and stale fields in the struct must not leak onto the air
  memset(&mib, 0, sizeof(mib));
  mib.mode               = SRSRAN_NBIOT_MODE_STANDALONE;
  mib.eutra_crs_seq_info = 31; // must be ignored
  srsran_npbch_mib_pack(0, 0, mib, payload);
  for (int i = 18; i < SRSRAN_MIB_NB_LEN; i++) {
    CHECK(payload[i] == 0, "standalone bit %d should be zero", i);
  }
  return SRSRAN_SUCCESS;
}

int main(int argc, char** argv)
{
  if (test_table() || test_worked_examples() || test_prb_isvalid() || test_mib_layout()) {
    printf("nbiot_inband_test: FAILED\n");
    return SRSRAN_ERROR;
  }
  printf("nbiot_inband_test: OK\n");
  return SRSRAN_SUCCESS;
}
