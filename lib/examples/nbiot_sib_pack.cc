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

/**
 * nbiot_sib_pack: build the MIB-NB / SIB1-NB broadcast for an NB-IoT cell from enb_nbiot.conf.
 *
 *   nbiot_sib_pack -c enb_nbiot.conf [-H hyper_sfn_msb] [-o sib1.bin] [-j]
 *
 * -o writes the SIB1-NB transport block (already padded to the size implied by schedulingInfoSIB1) for
 * npdsch_enodeb -B. -j prints the decoded ASN.1 as JSON, which is what a UE will see.
 */

#include <cstdio>
#include <cstdlib>
#include <string>
#include <unistd.h>
#include <vector>

#include "nbiot_sib_builder.h"
#include "srsran/asn1/rrc_nbiot.h"

static void usage(const char* prog)
{
  printf("Usage: %s -c enb_nbiot.conf [-H hyper_sfn_msb] [-o sib1.bin] [-s sib2.bin] [-j]\n", prog);
  printf("       %s -d sib1.bin        (decode a SIB1-NB transport block to JSON)\n", prog);
  printf("  -c FILE  NB-IoT carrier config (libconfig)\n");
  printf("  -H N     hyper-SFN 8 MSBs to place in SIB1-NB (default 0)\n");
  printf("  -o FILE  write the padded SIB1-NB transport block (binary)\n");
  printf("  -s FILE  write the padded SIB2-NB (first SI message) transport block (binary)\n");
  printf("  -j       print the decoded SIB1-NB and SI message (SIB2-NB) as JSON\n");
}

/// Decode a SIB1-NB transport block file and print it as JSON (no config needed).
static int decode_file(const char* path)
{
  FILE* f = fopen(path, "rb");
  if (!f) {
    fprintf(stderr, "cannot read %s\n", path);
    return 1;
  }
  std::vector<uint8_t> buf(512);
  size_t               n = fread(buf.data(), 1, buf.size(), f);
  fclose(f);

  asn1::rrc::bcch_dl_sch_msg_nb_s dl;
  asn1::cbit_ref                  bref(buf.data(), n);
  if (dl.unpack(bref) != asn1::SRSASN_SUCCESS) {
    fprintf(stderr, "%s does not decode as BCCH-DL-SCH-Message-NB\n", path);
    return 1;
  }
  asn1::json_writer j;
  dl.to_json(j);
  printf("%s\n", j.to_string().c_str());
  return 0;
}

int main(int argc, char** argv)
{
  std::string conf, out_file, sib2_file;
  unsigned    hfn_msb = 0;
  bool        json    = false;

  int opt;
  while ((opt = getopt(argc, argv, "c:H:o:s:d:jh")) != -1) {
    switch (opt) {
      case 'd':
        return decode_file(optarg);
      case 'c':
        conf = optarg;
        break;
      case 'H':
        hfn_msb = (unsigned)strtoul(optarg, nullptr, 0);
        break;
      case 'o':
        out_file = optarg;
        break;
      case 's':
        sib2_file = optarg;
        break;
      case 'j':
        json = true;
        break;
      default:
        usage(argv[0]);
        return opt == 'h' ? 0 : 1;
    }
  }
  if (conf.empty() || hfn_msb > 255) {
    usage(argv[0]);
    return 1;
  }

  std::string       err;
  nbiot::cell_config c;
  if (!nbiot::load_config_file(conf, c, err)) {
    fprintf(stderr, "config error: %s\n", err.c_str());
    return 1;
  }

  printf("NB-IoT cell: PLMN %s/%s TAC 0x%04x cell-id 0x%07x band %u\n",
         c.mcc.c_str(),
         c.mnc.c_str(),
         c.tac,
         c.cell_id,
         c.band);
  printf("  in-band same-PCI, LTE %u PRB, NB-IoT anchor PRB %u, PCI %u\n", c.lte_nof_prb, c.nbiot_prb, c.n_id_ncell);
  printf("  eutra-CRS-SequenceInfo %u, raster offset %s kHz\n",
         c.crs_seq_info,
         c.raster_offset == SRSRAN_NBIOT_RASTER_OFFSET_M7DOT5_KHZ   ? "-7.5"
         : c.raster_offset == SRSRAN_NBIOT_RASTER_OFFSET_M2DOT5_KHZ ? "-2.5"
         : c.raster_offset == SRSRAN_NBIOT_RASTER_OFFSET_P2DOT5_KHZ ? "+2.5"
                                                                    : "+7.5");

  std::vector<uint8_t> mib_bits;
  if (!nbiot::pack_mib_bits(c, 0, 0, mib_bits, err)) {
    fprintf(stderr, "MIB-NB error: %s\n", err.c_str());
    return 1;
  }
  printf("  MIB-NB: %zu bits, schedulingInfoSIB1=%u (%u repetitions, TBS %u bits)\n",
         mib_bits.size(),
         c.sched_info_sib1,
         nbiot::sib1_repetitions(c),
         nbiot::sib1_tbs_bits(c));

  std::vector<uint8_t> sib1;
  size_t               unpadded = 0;
  if (!nbiot::pack_sib1(c, (uint8_t)hfn_msb, sib1, unpadded, err)) {
    fprintf(stderr, "SIB1-NB error: %s\n", err.c_str());
    return 1;
  }
  printf("  SIB1-NB: %zu bytes (%zu bits) of %zu bytes available\n", unpadded, unpadded * 8, sib1.size());

  std::vector<uint8_t> sib2;
  size_t               unpadded2 = 0;
  if (!nbiot::pack_sib2(c, sib2, unpadded2, err)) {
    fprintf(stderr, "SIB2-NB error: %s\n", err.c_str());
    return 1;
  }
  printf("  SIB2-NB: %zu bytes (%zu bits) of %zu bytes available (SI message 0: periodicity %u rf, every %u rf)\n",
         unpadded2,
         unpadded2 * 8,
         sib2.size(),
         c.si_sched[0].periodicity_rf,
         c.si_sched[0].repetition_pattern);

  if (json) {
    asn1::rrc::bcch_dl_sch_msg_nb_s dl;
    asn1::cbit_ref                  bref(sib1.data(), sib1.size());
    if (dl.unpack(bref) != asn1::SRSASN_SUCCESS) {
      fprintf(stderr, "internal error: packed SIB1-NB does not unpack\n");
      return 1;
    }
    asn1::json_writer j;
    dl.to_json(j);
    printf("%s\n", j.to_string().c_str());

    asn1::rrc::bcch_dl_sch_msg_nb_s si;
    asn1::cbit_ref                  bref2(sib2.data(), sib2.size());
    if (si.unpack(bref2) != asn1::SRSASN_SUCCESS) {
      fprintf(stderr, "internal error: packed SIB2-NB does not unpack\n");
      return 1;
    }
    asn1::json_writer j2;
    si.to_json(j2);
    printf("%s\n", j2.to_string().c_str());
  }

  if (!sib2_file.empty()) {
    FILE* f = fopen(sib2_file.c_str(), "wb");
    if (!f || fwrite(sib2.data(), 1, sib2.size(), f) != sib2.size()) {
      fprintf(stderr, "cannot write %s\n", sib2_file.c_str());
      if (f) {
        fclose(f);
      }
      return 1;
    }
    fclose(f);
    printf("  wrote %zu bytes to %s\n", sib2.size(), sib2_file.c_str());
  }

  if (!out_file.empty()) {
    FILE* f = fopen(out_file.c_str(), "wb");
    if (!f || fwrite(sib1.data(), 1, sib1.size(), f) != sib1.size()) {
      fprintf(stderr, "cannot write %s\n", out_file.c_str());
      if (f) {
        fclose(f);
      }
      return 1;
    }
    fclose(f);
    printf("  wrote %zu bytes to %s\n", sib1.size(), out_file.c_str());
  }
  return 0;
}
