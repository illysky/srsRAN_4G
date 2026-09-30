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
 * Runs the NB-IoT downlink composer on top of a real LTE downlink (srsran_enb_dl_t), subframe after subframe, and
 * dumps the anchor PRB before and after, for enb_dl_nbiot_check.py to compare with a transmitter written from the
 * specification.
 *
 *   enb_dl_nbiot_dump out.bin nof_prb anchor_prb pci sched_info_sib1 hfn nframes junk
 *
 * out.bin: per subframe (frames 0..nframes-1, subframes 0..9), two blocks of 14 symbols x 12 subcarriers of
 * complex float (cf_t, symbol-major): the LTE-only anchor PRB, then the anchor PRB after the composer.
 *
 * stdout:
 *   MIB <sfn_base> <34 bits>            the MIB-NB block the composer was given per 64 frame period
 *   SIB1 <hex>                          the SIB1-NB transport block
 *   SI <n> <T> <offset> <rep> <tb> <window> <hex>
 *   FLAGS <sfn> <sf> <mask>             what the composer says it put there (only when non-zero)
 *   COLL <n>                            subframes in which LTE data was found in the anchor PRB
 *   OUTSIDE_DIFF <n>                    resource elements outside the anchor PRB data region that changed (must be 0)
 *
 * junk != 0 fills the anchor PRB data region with garbage before the composer runs, as an LTE PDSCH allocated on the
 * anchor PRB by mistake would; the result must be the same as without.
 */

#include <complex.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "srsran/phy/enb/enb_dl.h"
#include "srsran/phy/enb/enb_dl_nbiot.h"
#include "srsran/phy/utils/vector.h"

static uint32_t lcg_state = 12345;
static uint8_t  lcg_byte(void)
{
  lcg_state = lcg_state * 1664525u + 1013904223u;
  return (uint8_t)(lcg_state >> 24);
}

static void print_hex(const uint8_t* d, uint32_t n)
{
  for (uint32_t i = 0; i < n; i++) {
    printf("%02x", d[i]);
  }
  printf("\n");
}

int main(int argc, char** argv)
{
  if (argc != 9) {
    fprintf(stderr, "usage: %s out.bin nof_prb anchor_prb pci sched_info_sib1 hfn nframes junk\n", argv[0]);
    return 1;
  }
  const char*    out_file = argv[1];
  const uint32_t nof_prb  = (uint32_t)atoi(argv[2]);
  const uint32_t anchor   = (uint32_t)atoi(argv[3]);
  const uint32_t pci      = (uint32_t)atoi(argv[4]);
  const uint32_t sched    = (uint32_t)atoi(argv[5]);
  const uint32_t hfn      = (uint32_t)atoi(argv[6]);
  const uint32_t nframes  = (uint32_t)atoi(argv[7]);
  const int      junk     = atoi(argv[8]);

  srsran_cell_t lte;
  memset(&lte, 0, sizeof(lte));
  lte.nof_prb        = nof_prb;
  lte.nof_ports      = 1;
  lte.id             = pci;
  lte.cp             = SRSRAN_CP_NORM;
  lte.phich_length   = SRSRAN_PHICH_NORM;
  lte.phich_resources = SRSRAN_PHICH_R_1;
  lte.frame_type     = SRSRAN_FDD;

  srsran_nbiot_cell_t nb;
  memset(&nb, 0, sizeof(nb));
  nb.base       = lte;
  nb.nbiot_prb  = anchor;
  nb.n_id_ncell = pci;
  nb.nof_ports  = 1;
  nb.is_r14     = true;
  nb.mode       = SRSRAN_NBIOT_MODE_INBAND_SAME_PCI;

  srsran_mib_nb_t mib;
  memset(&mib, 0, sizeof(mib));
  mib.sched_info_sib1    = (uint8_t)sched;
  mib.sys_info_tag       = 7;
  mib.ac_barring         = true;
  mib.mode               = SRSRAN_NBIOT_MODE_INBAND_SAME_PCI;
  mib.eutra_crs_seq_info = 3;

  srsran_enb_dl_nbiot_t comp;
  if (srsran_enb_dl_nbiot_init(&comp, &nb) || srsran_enb_dl_nbiot_set_mib(&comp, &mib)) {
    fprintf(stderr, "composer init failed\n");
    return 2;
  }

  uint8_t sib1[SRSRAN_ENB_DL_NBIOT_MAX_BCCH_BYTES];
  uint32_t sib1_bytes = (uint32_t)srsran_ra_nbiot_get_sib1_tbs(&mib) / 8;
  for (uint32_t i = 0; i < sib1_bytes; i++) {
    sib1[i] = lcg_byte();
  }
  if (srsran_enb_dl_nbiot_set_sib1(&comp, sib1, sib1_bytes)) {
    return 2;
  }
  printf("SIB1 ");
  print_hex(sib1, sib1_bytes);

  // Three SI messages with different periodicities, patterns and sizes (one of them only present in even hyperframes)
  const srsran_nbiot_si_params_t si_cfg[3] = {
      {.n = 1, .si_periodicity = 64, .si_radio_frame_offset = 1, .si_repetition_pattern = 4, .si_tb = 208, .si_window_length = 160},
      {.n = 2, .si_periodicity = 128, .si_radio_frame_offset = 1, .si_repetition_pattern = 8, .si_tb = 120, .si_window_length = 160},
      {.n = 3, .si_periodicity = 2048, .si_radio_frame_offset = 1, .si_repetition_pattern = 2, .si_tb = 328, .si_window_length = 160},
  };
  for (int i = 0; i < 3; i++) {
    uint8_t  si[SRSRAN_ENB_DL_NBIOT_MAX_BCCH_BYTES];
    uint32_t n = si_cfg[i].si_tb / 8;
    for (uint32_t j = 0; j < n; j++) {
      si[j] = lcg_byte();
    }
    if (srsran_enb_dl_nbiot_set_si(&comp, (uint32_t)i, &si_cfg[i], si, n)) {
      return 2;
    }
    printf("SI %u %u %u %u %u %u ", si_cfg[i].n, si_cfg[i].si_periodicity, si_cfg[i].si_radio_frame_offset,
           si_cfg[i].si_repetition_pattern, si_cfg[i].si_tb, si_cfg[i].si_window_length);
    print_hex(si, n);
  }

  for (uint32_t sfn = 0; sfn < nframes; sfn += 64) {
    uint8_t payload[SRSRAN_MIB_NB_LEN];
    srsran_npbch_mib_pack(hfn, sfn, mib, payload);
    printf("MIB %u ", sfn);
    for (int i = 0; i < SRSRAN_MIB_NB_LEN; i++) {
      printf("%d", payload[i]);
    }
    printf("\n");
  }

  cf_t* out[SRSRAN_MAX_PORTS] = {NULL};
  out[0]                      = srsran_vec_cf_malloc(SRSRAN_SF_LEN_PRB(nof_prb));
  srsran_enb_dl_t enb;
  if (srsran_enb_dl_init(&enb, out, nof_prb) || srsran_enb_dl_set_cell(&enb, lte)) {
    fprintf(stderr, "enb_dl init failed\n");
    return 2;
  }

  const uint32_t w      = nof_prb * SRSRAN_NRE;
  const uint32_t nre    = 14 * w;
  cf_t*          before = srsran_vec_cf_malloc(nre);
  FILE*          fo     = fopen(out_file, "wb");
  if (!fo) {
    perror("fopen");
    return 2;
  }

  uint64_t outside_diff = 0;
  uint32_t jstate       = 99;
  for (uint32_t sfn = 0; sfn < nframes; sfn++) {
    for (uint32_t sf = 0; sf < 10; sf++) {
      srsran_dl_sf_cfg_t dl_sf;
      memset(&dl_sf, 0, sizeof(dl_sf));
      dl_sf.tti     = sfn * 10 + sf;
      dl_sf.cfi     = 3;
      dl_sf.sf_type = SRSRAN_SF_NORM;
      srsran_enb_dl_put_base(&enb, &dl_sf);

      cf_t* grid[SRSRAN_MAX_PORTS] = {enb.sf_symbols[0], NULL};

      // LTE-only anchor PRB
      cf_t pre[14 * SRSRAN_NRE];
      for (uint32_t l = 0; l < 14; l++) {
        memcpy(&pre[l * SRSRAN_NRE], &grid[0][l * w + anchor * SRSRAN_NRE], SRSRAN_NRE * sizeof(cf_t));
      }
      fwrite(pre, sizeof(cf_t), 14 * SRSRAN_NRE, fo);

      if (junk) {
        for (uint32_t l = 3; l < 14; l++) {
          for (uint32_t k = 0; k < SRSRAN_NRE; k++) {
            cf_t* re = &grid[0][l * w + anchor * SRSRAN_NRE + k];
            if (*re == 0) { // leave CRS alone: LTE data never lands on them
              jstate = jstate * 1664525u + 1013904223u;
              *re    = ((float)(jstate >> 16) / 32768.0f - 1.0f) + I * ((float)(jstate & 0xffff) / 32768.0f - 1.0f);
            }
          }
        }
      }
      memcpy(before, grid[0], nre * sizeof(cf_t));

      int mask = srsran_enb_dl_nbiot_put_sf(&comp, hfn, sfn, sf, grid);
      if (mask < 0) {
        fprintf(stderr, "put_sf failed at %u.%u\n", sfn, sf);
        return 3;
      }
      if (mask) {
        printf("FLAGS %u %u %d\n", sfn, sf, mask);
      }

      for (uint32_t l = 0; l < 14; l++) {
        for (uint32_t k = 0; k < w; k++) {
          bool in_anchor = (l >= 3 && k >= anchor * SRSRAN_NRE && k < (anchor + 1) * SRSRAN_NRE);
          if (!in_anchor && grid[0][l * w + k] != before[l * w + k]) {
            if (outside_diff < 8) {
              printf("OUT sfn=%u sf=%u l=%u k=%u (anchor columns %u..%u)\n", sfn, sf, l, k, anchor * SRSRAN_NRE,
                     (anchor + 1) * SRSRAN_NRE - 1);
            }
            outside_diff++;
          }
        }
      }

      cf_t post[14 * SRSRAN_NRE];
      for (uint32_t l = 0; l < 14; l++) {
        memcpy(&post[l * SRSRAN_NRE], &grid[0][l * w + anchor * SRSRAN_NRE], SRSRAN_NRE * sizeof(cf_t));
      }
      fwrite(post, sizeof(cf_t), 14 * SRSRAN_NRE, fo);
    }
  }
  fclose(fo);
  printf("COLL %llu\n", (unsigned long long)comp.lte_collisions);
  printf("OUTSIDE_DIFF %llu\n", (unsigned long long)outside_diff);

  srsran_enb_dl_nbiot_free(&comp);
  srsran_enb_dl_free(&enb);
  free(out[0]);
  free(before);
  return 0;
}
