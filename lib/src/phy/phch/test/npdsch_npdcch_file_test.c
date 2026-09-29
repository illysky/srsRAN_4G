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

#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <unistd.h>

#include "srsran/phy/channel/ch_awgn.h"
#include "srsran/phy/io/filesource.h"
#include "srsran/phy/phch/npdsch.h"
#include "srsran/phy/ue/ue_dl_nbiot.h"
#include "srsran/phy/utils/debug.h"

#define DUMP_SIGNAL 0

char* input_file_name = NULL;

static srsran_nbiot_cell_t cell = {.base       = {.nof_prb = 1, .nof_ports = 1, .cp = SRSRAN_CP_NORM, .id = 0},
                                   .nbiot_prb  = 0,
                                   .n_id_ncell = 0,
                                   .nof_ports  = 1,
                                   .mode       = SRSRAN_NBIOT_MODE_STANDALONE,
                                   .is_r14     = true};

int flen;

uint16_t rnti = SRSRAN_SIRNTI;

int             sfn            = 0;
int             nof_frames     = 0;
int             max_frames     = 1;
uint32_t        sf_idx         = 0;
uint32_t        sched_info_tag = 0; // Value of schedulingInfoSIB1-NB-r13
srsran_mib_nb_t mib;
bool            decode_sib1 = false;
float           snr         = -1.0;
const char*     sib1_dump_name = NULL; // -D: write the last decoded SIB1-NB transport block here
uint32_t        sib1_dumped    = 0;

srsran_dci_format_t  dci_format = SRSRAN_DCI_FORMAT1;
srsran_filesource_t  fsrc;
srsran_nbiot_ue_dl_t ue_dl;
cf_t*                buff_ptrs[SRSRAN_MAX_PORTS] = {NULL, NULL, NULL, NULL};

void usage(char* prog)
{
  printf("Usage: %s [rovcnwmpstRxPMISD] -i input_file\n", prog);
  printf("\t-o DCI format [Default %s]\n", srsran_dci_format_string(dci_format));
  printf("\t-c n_id_ncell [Default %d]\n", cell.n_id_ncell);
  printf("\t-s Start subframe_idx [Default %d]\n", sf_idx);
  printf("\t-w Start SFN [Default %d]\n", nof_frames);
  printf("\t-t Value of schedulingInfoSIB1-NB-r13 [Default %d]\n", sched_info_tag);
  printf("\t-k Decode SIB1 [Default %s]\n", decode_sib1 ? "Yes" : "No");
  printf("\t-r rnti [Default 0x%x]\n", rnti);
  printf("\t-p cell.nof_ports [Default %d]\n", cell.base.nof_ports);
  printf("\t-n cell.nof_prb [Default %d]\n", cell.base.nof_prb);
  printf("\t-m max_frames [Default %d]\n", max_frames);
  printf("\t-R Is R14 cell [Default %s]\n", cell.is_r14 ? "Yes" : "No");
  printf("\t-x SNR-10 (apply noise to input file) [Default %f]\n", snr);
  printf("\t-P NB-IoT PRB index within the LTE carrier [Default %d]\n", cell.nbiot_prb);
  printf("\t-M operation mode: 0=inband-samePCI 1=inband-differentPCI 2=guardband 3=standalone [Default %d]\n",
         cell.mode);
  printf("\t-I LTE cell id (PCI) of the carrier NB-IoT is deployed in [Default %d]\n", cell.base.id);
  printf("\t-S input uses standard LTE sample rates (7.68 MS/s for 25 PRB), as srsenb does with lte_sample_rates\n");
  printf("\t-D FILE write the last decoded SIB1-NB transport block to FILE (use with -k)\n");
  printf("\t-v [set srsran_verbose to debug, default none]\n");
}

void parse_args(int argc, char** argv)
{
  int opt;
  bool n_id_ncell_given = false;
  while ((opt = getopt(argc, argv, "irovcnmwpkstxPMISD")) != -1) {
    switch (opt) {
      case 'i':
        input_file_name = argv[optind];
        break;
      case 'c':
        cell.n_id_ncell  = (uint32_t)strtol(argv[optind], NULL, 10);
        n_id_ncell_given = true;
        break;
      case 'P':
        cell.nbiot_prb = (uint32_t)strtol(argv[optind], NULL, 10);
        break;
      case 'M': {
        long m = strtol(argv[optind], NULL, 10);
        if (m < 0 || m >= SRSRAN_NBIOT_MODE_N_ITEMS) {
          fprintf(stderr, "Error: invalid operation mode %ld\n", m);
          exit(-1);
        }
        cell.mode = (srsran_nbiot_mode_t)m;
        break;
      }
      case 'I':
        cell.base.id = (uint32_t)strtol(argv[optind], NULL, 10);
        break;
      case 'S':
        srsran_use_standard_symbol_size(true);
        break;
      case 'R':
        cell.is_r14 = !cell.is_r14;
        break;
      case 's':
        sf_idx = (uint32_t)strtol(argv[optind], NULL, 10);
        break;
      case 't':
        sched_info_tag = (uint32_t)strtol(argv[optind], NULL, 10);
        break;
      case 'r':
        rnti = strtoul(argv[optind], NULL, 0);
        break;
      case 'w':
        sfn = strtoul(argv[optind], NULL, 0);
        break;
      case 'm':
        max_frames = strtoul(argv[optind], NULL, 0);
        break;
      case 'n':
        cell.base.nof_prb = (uint32_t)strtol(argv[optind], NULL, 10);
        break;
      case 'k':
        decode_sib1 = true;
        break;
      case 'D':
        sib1_dump_name = argv[optind];
        break;
      case 'p':
        cell.base.nof_ports = (uint32_t)strtol(argv[optind], NULL, 10);
        break;
      case 'o':
        dci_format = srsran_dci_format_from_string(argv[optind]);
        if (dci_format == SRSRAN_DCI_NOF_FORMATS) {
          fprintf(stderr, "Error unsupported format %s\n", argv[optind]);
          exit(-1);
        }
        break;
      case 'x':
        snr = strtof(argv[optind], NULL);
        break;
      case 'v':
        increase_srsran_verbose_level();
        break;
      default:
        usage(argv[0]);
        exit(-1);
    }
  }
  if (!input_file_name) {
    usage(argv[0]);
    exit(-1);
  }

  if (cell.mode == SRSRAN_NBIOT_MODE_INBAND_SAME_PCI) {
    if (n_id_ncell_given && cell.n_id_ncell != cell.base.id) {
      fprintf(stderr,
              "Error: inband-samePCI needs n_id_ncell (%d) == LTE cell id (%d); use -I and drop -c\n",
              cell.n_id_ncell,
              cell.base.id);
      exit(-1);
    }
    cell.n_id_ncell = cell.base.id;
  }
}

int base_init()
{
  if (srsran_filesource_init(&fsrc, input_file_name, SRSRAN_COMPLEX_FLOAT_BIN)) {
    fprintf(stderr, "Error opening file %s\n", input_file_name);
    exit(-1);
  }

  flen         = 2 * (SRSRAN_SLOT_LEN(srsran_symbol_sz(cell.base.nof_prb)));
  buff_ptrs[0] = srsran_vec_cf_malloc(flen);
  if (!buff_ptrs[0]) {
    perror("malloc");
    exit(-1);
  }

  // For in-band the whole LTE carrier is demodulated, so the grid has to be sized for it
  if (srsran_nbiot_ue_dl_init(&ue_dl, buff_ptrs, cell.base.nof_prb, SRSRAN_NBIOT_NUM_RX_ANTENNAS)) {
    fprintf(stderr, "Error initializing UE DL\n");
    return -1;
  }

  if (srsran_nbiot_ue_dl_set_cell(&ue_dl, cell)) {
    fprintf(stderr, "Configuring cell in UE DL\n");
    return -1;
  }

  mib.sched_info_sib1 = sched_info_tag;
  srsran_nbiot_ue_dl_set_mib(&ue_dl, mib);
  srsran_nbiot_ue_dl_set_rnti(&ue_dl, rnti);
  // srsran_mib_nb_printf(stdout, 0, &mib);

  DEBUG("Memory init OK");
  return 0;
}

void base_free()
{
  srsran_filesource_free(&fsrc);
  srsran_nbiot_ue_dl_free(&ue_dl);

  for (int i = 0; i < SRSRAN_MAX_PORTS; i++) {
    if (buff_ptrs[i] != NULL) {
      free(buff_ptrs[i]);
    }
  }
}

int main(int argc, char** argv)
{
  int                 dci_formatN0_detected = 0;
  int                 dci_formatN1_detected = 0;
  srsran_dci_format_t last_dci_format       = SRSRAN_DCI_FORMAT0;

  if (argc < 3) {
    usage(argv[0]);
    exit(-1);
  }
  parse_args(argc, argv);

  if (base_init()) {
    fprintf(stderr, "Error initializing memory\n");
    exit(-1);
  }

  // adjust SNR input to allow for negative values
  if (snr != -1.0) {
    snr -= 10.0;
    printf("Target SNR: %.2fdB\n", snr);
  }

  uint8_t* data = malloc(100000);

  int nread = 0;

  if (decode_sib1) {
    srsran_nbiot_ue_dl_decode_sib1(&ue_dl, sfn);
  }

  do {
    nread = srsran_filesource_read(&fsrc, buff_ptrs[0], flen);
    if (nread > 0) {
      DEBUG("%d.%d: Reading %d samples.", sfn, sf_idx, nread);

      // add some noise to the signal
      if (snr != -1.0) {
        float var = srsran_convert_dB_to_power(-snr);
        srsran_ch_awgn_c(buff_ptrs[0], buff_ptrs[0], var, nread);
      }

      // try to decode
      if (srsran_ra_nbiot_is_valid_dl_sf(sfn * 10 + sf_idx)) {
        int n;
        if (srsran_nbiot_ue_dl_has_grant(&ue_dl)) {
          // attempt to decode NPDSCH
          n = srsran_nbiot_ue_dl_decode_npdsch(&ue_dl, buff_ptrs[0], data, sfn, sf_idx, rnti);
          if (n == SRSRAN_SUCCESS) {
            INFO("NPDSCH decoded ok.");

            if (sib1_dump_name && ue_dl.npdsch_cfg.has_bcch) {
              // data holds the decoded transport block, packed into bytes
              FILE* df = fopen(sib1_dump_name, "wb");
              if (df) {
                fwrite(data, 1, ue_dl.npdsch_cfg.grant.mcs[0].tbs / 8, df);
                fclose(df);
                sib1_dumped++;
              }
            }

            if (decode_sib1) {
              srsran_nbiot_ue_dl_decode_sib1(&ue_dl, sfn);
            }
          } else if (n == SRSRAN_ERROR && decode_sib1) {
            // reactivate SIB1 reception
            srsran_nbiot_ue_dl_decode_sib1(&ue_dl, sfn);
          }
        } else {
          // decode NPDCCH
          srsran_dci_msg_t dci_msg;
          n = srsran_nbiot_ue_dl_decode_npdcch(&ue_dl, buff_ptrs[0], sfn, sf_idx, rnti, &dci_msg);
          if (n == SRSRAN_NBIOT_UE_DL_FOUND_DCI) {
            INFO("Found %s DCI for RNTI=0x%x", srsran_dci_format_string(dci_msg.format), rnti);
            last_dci_format = dci_msg.format;

            if (dci_msg.format == SRSRAN_DCI_FORMATN0) {
              // try to convert to UL grant
              srsran_ra_nbiot_ul_dci_t   dci_unpacked;
              srsran_ra_nbiot_ul_grant_t grant;
              // Creates the UL NPUSCH resource allocation grant from a DCI format N0 message
              if (srsran_nbiot_dci_msg_to_ul_grant(
                      &dci_msg, &dci_unpacked, &grant, sfn * 10 + sf_idx, SRSRAN_NPUSCH_SC_SPACING_15000)) {
                fprintf(stderr, "Error unpacking DCI\n");
                return SRSRAN_ERROR;
              }
              dci_formatN0_detected++;

              // we don't use UL grants any further
            } else if (dci_msg.format == SRSRAN_DCI_FORMATN1) {
              // try to convert DCI to DL grant
              srsran_ra_nbiot_dl_dci_t   dci_unpacked;
              srsran_ra_nbiot_dl_grant_t grant;
              if (srsran_nbiot_dci_msg_to_dl_grant(
                      &dci_msg, rnti, &dci_unpacked, &grant, sfn, sf_idx, 64 /* fixme: remove */, cell.mode)) {
                fprintf(stderr, "Error unpacking DCI\n");
                return SRSRAN_ERROR;
              }

              dci_formatN1_detected++;

              // activate grant
              srsran_nbiot_ue_dl_set_grant(&ue_dl, &grant);
            } else {
              fprintf(stderr, "Unsupported DCI format.\n");
              return SRSRAN_ERROR;
            }
          }
        }
      }

      sf_idx++;
      if (sf_idx == 10) {
        sfn = (sfn + 1) % 1024;
        nof_frames++;
        sf_idx = 0;
      }
    }
  } while (nread > 0 && nof_frames <= max_frames);

  uint32_t pkts_ok = ue_dl.pkts_ok;
  printf("pkt_total=%d\n", ue_dl.pkts_total);
  printf("pkt_ok=%d\n", pkts_ok);
  if (sib1_dump_name) {
    printf("sib1_dumped=%u\n", sib1_dumped);
  }
  printf("pkt_errors=%d\n", ue_dl.pkt_errors);
  printf("bler=%.2f\n", ue_dl.pkts_total ? (float)100 * ue_dl.pkt_errors / ue_dl.pkts_total : 0);
  printf("rate=%.2f\n", ((ue_dl.bits_total / ((nof_frames * 10 + sf_idx) / 1000.0)) / 1000.0));
  printf("dci_detected=%d\n", ue_dl.nof_detected);
  printf("dci_formatN0_detected=%d\n", dci_formatN0_detected);
  printf("dci_formatN1_detected=%d\n", dci_formatN1_detected);
  printf("num_frames=%d\n", nof_frames);

#if DUMP_SIGNAL
  printf("Saving signal...\n");
  srsran_nb_ue_dl_save_signal(&ue_dl, &ue_dl.softbuffer, sf_idx);
#endif

  base_free();
  if (data != NULL) {
    free(data);
  }

  // exit value depends on actual test

  // check if we are testing for correct DCI deconding
  if (dci_format == SRSRAN_DCI_FORMATN0 || dci_format == SRSRAN_DCI_FORMATN1 || dci_format == SRSRAN_DCI_FORMATN2) {
    // this is a DCI test, check if last DCI matches requested type
    if (last_dci_format == dci_format) {
      return SRSRAN_SUCCESS;
    } else {
      return SRSRAN_ERROR;
    }
  }

  // success if at least one NPDSCH could be decoded
  if (pkts_ok > 0)
    return SRSRAN_SUCCESS;

  // return error by default
  return SRSRAN_ERROR;
}
