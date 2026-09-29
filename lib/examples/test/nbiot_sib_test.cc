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
 * Tests for the NB-IoT system information builder (nbiot_sib_builder).
 *
 * What is (and is not) independent evidence here:
 *  - The ASN.1 codec is checked against a SIB1-NB captured from a real operator network: it must decode to sensible
 *    values and re-encode byte for byte. That is evidence the codec matches real-world encoders, not just itself.
 *  - The ASN.1 MIB-NB encoder is checked against the separate, hand-written C packer used by the NPBCH path.
 *  - The raster offset implied by the anchor PRB is re-derived from LTE subcarrier geometry and compared with the
 *    TS 36.213 Table 16.8-1 lookup.
 *  - Every invalid configuration must be rejected, and for the right reason (the error must name the offending key).
 */

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "../nbiot_sib_builder.h"
#include "srsran/asn1/rrc_nbiot.h"

static int g_fail = 0;
static int g_checks = 0;

#define CHECK(cond, ...)                                                                                               \
  do {                                                                                                                 \
    ++g_checks;                                                                                                        \
    if (!(cond)) {                                                                                                     \
      ++g_fail;                                                                                                        \
      printf("FAIL %s:%d: ", __FILE__, __LINE__);                                                                      \
      printf(__VA_ARGS__);                                                                                             \
      printf("\n");                                                                                                    \
    }                                                                                                                  \
  } while (0)

// A SIB1-NB captured from a real in-band NB-IoT operator network (the payload the generator has always broadcast).
static const uint8_t real_network_sib1[] = {0x43, 0x4d, 0xd0, 0x92, 0x22, 0x06, 0x04, 0x30, 0x28,
                                            0x6e, 0x87, 0xd0, 0x4b, 0x13, 0x90, 0xb4, 0x12, 0xa1,
                                            0x02, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static const char* BASE_CONF = R"(
nbiot = {
  lte     = { n_prb = 25; pci = 1; nof_ports = 1; cfi = 3; };
  carrier = { operation_mode = "inband_same_pci"; nbiot_prb = 17; n_id_ncell = 1; nof_ports = 1; };
  cell    = { mcc = "234"; mnc = "01"; tac = 0x0001; cell_id = 0x019B01; band = 20;
              cell_barred = false; intra_freq_reselection = true; q_rx_lev_min = -70; };
  mib     = { sched_info_sib1 = 0; sys_info_tag = 0; ac_barring = false; };
  sib1    = { si_window_length = 160; si_radio_frame_offset = 1; nrs_crs_power_offset = 0;
              eutra_control_region_size = 3;
              sched_info = ( { si_periodicity = 1024; si_repetition_pattern = 4; si_tb = 208; sib_mapping = [ ]; } ); };
};
)";

static bool load_string(const std::string& text, nbiot::cell_config& c, std::string& err)
{
  libconfig::Config cfg;
  try {
    cfg.readString(text);
  } catch (const libconfig::ParseException& e) {
    err = std::string("test config does not parse: ") + e.getError();
    return false;
  }
  return nbiot::load_config(cfg, c, err);
}

static std::string replace(std::string s, const std::string& from, const std::string& to)
{
  size_t pos = s.find(from);
  if (pos == std::string::npos) {
    printf("TEST BUG: '%s' not found in base config\n", from.c_str());
    ++g_fail;
    return s;
  }
  return s.replace(pos, from.size(), to);
}

/*************************************************************************************************/
static void test_real_network_vector()
{
  asn1::rrc::bcch_dl_sch_msg_nb_s dl;
  asn1::cbit_ref                  bref(real_network_sib1, sizeof(real_network_sib1));
  CHECK(dl.unpack(bref) == asn1::SRSASN_SUCCESS, "real-network SIB1-NB does not decode");
  CHECK(dl.msg.type() == asn1::rrc::bcch_dl_sch_msg_type_nb_c::types::c1, "not a c1 message");
  CHECK(dl.msg.c1().type() == asn1::rrc::bcch_dl_sch_msg_type_nb_c::c1_c_::types::sib_type1_r13, "not SIB1-NB");

  const asn1::rrc::sib_type1_nb_s& s = dl.msg.c1().sib_type1_r13();
  CHECK(s.freq_band_ind_r13 == 20, "band %u != 20", s.freq_band_ind_r13);
  CHECK(s.cell_access_related_info_r13.plmn_id_list_r13.size() == 1, "expected one PLMN");
  const auto& plmn = s.cell_access_related_info_r13.plmn_id_list_r13[0].plmn_id_r13;
  CHECK(plmn.mcc[0] == 2 && plmn.mcc[1] == 4 && plmn.mcc[2] == 4, "MCC digits");
  CHECK(plmn.mnc.size() == 2 && plmn.mnc[0] == 8 && plmn.mnc[1] == 1, "MNC digits");
  CHECK(s.eutra_ctrl_region_size_r13_present && s.eutra_ctrl_region_size_r13.to_number() == 3, "control region");
  CHECK(s.sched_info_list_r13.size() == 2, "expected two SI messages, got %u", (unsigned)s.sched_info_list_r13.size());
  // TS 36.331: SIB2-NB is implicit in the first SI message, so its sib-MappingInfo lists nothing.
  CHECK(s.sched_info_list_r13[0].sib_map_info_r13.size() == 0, "first SI message must not list SIB2 explicitly");

  // Re-encode and compare with the original bits, byte for byte.
  uint8_t       out[64] = {};
  asn1::bit_ref oref(out, sizeof(out));
  CHECK(dl.pack(oref) == asn1::SRSASN_SUCCESS, "re-pack failed");
  oref.align_bytes_zero();
  size_t len = oref.distance_bytes();
  CHECK(len <= sizeof(real_network_sib1), "re-packed length %zu too long", len);
  CHECK(memcmp(out, real_network_sib1, len) == 0, "re-encoding of the real-network SIB1-NB is not byte-exact");
  for (size_t i = len; i < sizeof(real_network_sib1); ++i) {
    CHECK(real_network_sib1[i] == 0, "non-zero padding at byte %zu", i);
  }
}

/*************************************************************************************************/
static void test_config_load()
{
  nbiot::cell_config c;
  std::string        err;
  CHECK(load_string(BASE_CONF, c, err), "base config rejected: %s", err.c_str());
  CHECK(c.crs_seq_info == 7, "PRB 17 of 25 must give eutra-CRS-SequenceInfo 7, got %u", c.crs_seq_info);
  CHECK(c.raster_offset == SRSRAN_NBIOT_RASTER_OFFSET_P7DOT5_KHZ, "PRB 17 must give +7.5 kHz raster offset");
  CHECK(c.mcc == "234" && c.mnc == "01" && c.tac == 1 && c.cell_id == 0x019B01, "identity fields");
  CHECK(nbiot::sib1_tbs_bits(c) == 208 && nbiot::sib1_repetitions(c) == 4, "schedulingInfoSIB1=0 -> 208 bits x4");
}

/// The anchor PRB -> (eutra-CRS-SequenceInfo, raster offset) mapping, checked against expectations worked out from
/// the subcarrier geometry rather than from the library's own table.
static void test_anchor_geometry()
{
  struct row {
    uint32_t                     prb;
    uint8_t                      info;
    srsran_nbiot_raster_offset_t off;
  };
  const row rows[] = {{2, 5, SRSRAN_NBIOT_RASTER_OFFSET_M7DOT5_KHZ},
                      {7, 6, SRSRAN_NBIOT_RASTER_OFFSET_M7DOT5_KHZ},
                      {17, 7, SRSRAN_NBIOT_RASTER_OFFSET_P7DOT5_KHZ},
                      {22, 8, SRSRAN_NBIOT_RASTER_OFFSET_P7DOT5_KHZ}};
  for (const row& r : rows) {
    // Independent derivation. 25 PRB = 300 used subcarriers at k = -150..-1, +1..+150 (DC unused). PRB 12 straddles
    // DC and is centred on it; PRBs below DC are shifted down by half a subcarrier, PRBs above are shifted up.
    double centre_khz = ((double)r.prb - 12.0) * 180.0 + (r.prb < 12 ? -7.5 : 7.5);
    double raster     = std::fmod(centre_khz, 100.0);
    if (raster > 50.0) {
      raster -= 100.0;
    }
    if (raster <= -50.0) {
      raster += 100.0;
    }
    const srsran_nbiot_raster_offset_t geo = std::fabs(raster - 7.5) < 1e-6    ? SRSRAN_NBIOT_RASTER_OFFSET_P7DOT5_KHZ
                                             : std::fabs(raster + 7.5) < 1e-6 ? SRSRAN_NBIOT_RASTER_OFFSET_M7DOT5_KHZ
                                                                              : (srsran_nbiot_raster_offset_t)99;
    CHECK(geo == r.off, "PRB %u: geometry gives raster offset %.1f kHz, expected table entry differs", r.prb, raster);

    std::string        err;
    nbiot::cell_config c;
    CHECK(load_string(replace(BASE_CONF, "nbiot_prb = 17", "nbiot_prb = " + std::to_string(r.prb)), c, err),
          "PRB %u rejected: %s",
          r.prb,
          err.c_str());
    CHECK(c.crs_seq_info == r.info, "PRB %u: crs seq info %u != %u", r.prb, c.crs_seq_info, r.info);
    CHECK(c.raster_offset == r.off, "PRB %u: raster offset mismatch", r.prb);
    CHECK(c.raster_offset == geo, "PRB %u: library raster offset disagrees with geometry", r.prb);
  }
}

/*************************************************************************************************/
/// ASN.1 MIB-NB encoder vs the hand-written C packer used by the NPBCH path, bit for bit.
static void test_mib_cross_check()
{
  const uint32_t prbs[] = {2, 7, 17, 22};
  uint32_t       n      = 0;
  for (uint32_t prb : prbs) {
    nbiot::cell_config c;
    std::string        err;
    CHECK(load_string(replace(BASE_CONF, "nbiot_prb = 17", "nbiot_prb = " + std::to_string(prb)), c, err), "%s",
          err.c_str());
    for (uint32_t sched = 0; sched <= 11; ++sched) {
      for (uint32_t tag : {0u, 1u, 17u, 31u}) {
        for (bool ab : {false, true}) {
          c.sched_info_sib1 = sched;
          c.sys_info_tag    = tag;
          c.ac_barring      = ab;
          for (uint32_t hfn : {0u, 1u, 2u, 3u}) {
            for (uint32_t sfn : {0u, 64u, 128u, 512u, 960u, 1023u}) {
              std::vector<uint8_t> asn1_bits;
              CHECK(nbiot::pack_mib_bits(c, hfn, sfn, asn1_bits, err), "pack: %s", err.c_str());

              uint8_t c_bits[SRSRAN_MIB_NB_LEN] = {};
              srsran_npbch_mib_pack(hfn, sfn, nbiot::to_c_mib(c), c_bits);

              CHECK(asn1_bits.size() == SRSRAN_MIB_NB_LEN, "ASN.1 MIB is %zu bits", asn1_bits.size());
              bool same = asn1_bits.size() == SRSRAN_MIB_NB_LEN && memcmp(asn1_bits.data(), c_bits, SRSRAN_MIB_NB_LEN) == 0;
              CHECK(same, "MIB-NB mismatch prb=%u sched=%u tag=%u ab=%d hfn=%u sfn=%u", prb, sched, tag, ab, hfn, sfn);
              ++n;
            }
          }
        }
      }
    }
  }
  printf("  MIB-NB: %u parameter combinations compared, ASN.1 vs C packer\n", n);

  // And the C unpacker recovers what the ASN.1 packer wrote.
  nbiot::cell_config c;
  std::string        err;
  CHECK(load_string(BASE_CONF, c, err), "%s", err.c_str());
  c.sched_info_sib1 = 5;
  c.sys_info_tag    = 9;
  std::vector<uint8_t> bits;
  CHECK(nbiot::pack_mib_bits(c, 2, 640, bits, err), "%s", err.c_str());
  srsran_mib_nb_t out;
  memset(&out, 0, sizeof(out));
  srsran_npbch_mib_unpack(bits.data(), &out);
  CHECK(out.sched_info_sib1 == 5 && out.sys_info_tag == 9 && out.hfn == 2 && out.sfn == 640, "unpack of ASN.1 MIB");
  CHECK(out.mode == SRSRAN_NBIOT_MODE_INBAND_SAME_PCI && out.eutra_crs_seq_info == 7, "operation mode / CRS info");
  // (For same-PCI the raster offset is not broadcast in MIB-NB; it is implied by eutra-CRS-SequenceInfo, which
  //  test_anchor_geometry() checks against the subcarrier geometry.)
}

/*************************************************************************************************/
static void test_sib1_roundtrip()
{
  nbiot::cell_config c;
  std::string        err;
  CHECK(load_string(BASE_CONF, c, err), "%s", err.c_str());

  std::vector<uint8_t> tb;
  size_t               len = 0;
  CHECK(nbiot::pack_sib1(c, 0xA5, tb, len, err), "pack_sib1: %s", err.c_str());
  CHECK(tb.size() == 26, "transport block is %zu bytes, want 26 (208 bits)", tb.size());
  CHECK(len > 0 && len <= 26, "unpadded length %zu", len);
  for (size_t i = len; i < tb.size(); ++i) {
    CHECK(tb[i] == 0, "padding byte %zu is not zero", i);
  }

  // hyper-SFN MSB sits at bit offset 12 (msg class 1 + c1 choice 1 + 10 optional-field bits). The generator overwrites
  // exactly those bits at run time, so this pins the offset it relies on.
  uint32_t hs = 0;
  for (int i = 12; i < 20; ++i) {
    hs = (hs << 1) | ((tb[i / 8] >> (7 - (i % 8))) & 1);
  }
  CHECK(hs == 0xA5, "hyperSFN-MSB not at bit 12: read 0x%02x", hs);

  asn1::rrc::bcch_dl_sch_msg_nb_s dl;
  asn1::cbit_ref                  bref(tb.data(), tb.size());
  CHECK(dl.unpack(bref) == asn1::SRSASN_SUCCESS, "own SIB1-NB does not unpack");
  const asn1::rrc::sib_type1_nb_s& s = dl.msg.c1().sib_type1_r13();
  CHECK(s.hyper_sfn_msb_r13.to_number() == 0xA5, "hyperSFN");
  const auto& car = s.cell_access_related_info_r13;
  CHECK(car.plmn_id_list_r13.size() == 1, "one PLMN");
  const auto& plmn = car.plmn_id_list_r13[0].plmn_id_r13;
  CHECK(plmn.mcc[0] == 2 && plmn.mcc[1] == 3 && plmn.mcc[2] == 4, "MCC");
  CHECK(plmn.mnc.size() == 2 && plmn.mnc[0] == 0 && plmn.mnc[1] == 1, "MNC keeps its leading zero");
  CHECK(car.tac_r13.to_number() == 1, "TAC");
  CHECK(car.cell_id_r13.to_number() == 0x019B01, "cell id 0x%llx", (unsigned long long)car.cell_id_r13.to_number());
  CHECK(std::string(car.cell_barred_r13.to_string()) == "notBarred", "cellBarred");
  CHECK(std::string(car.intra_freq_resel_r13.to_string()) == "allowed", "intraFreqReselection");
  CHECK(s.freq_band_ind_r13 == 20, "band");
  CHECK(s.cell_sel_info_r13.q_rx_lev_min_r13 == -70, "qRxLevMin");
  CHECK(s.eutra_ctrl_region_size_r13_present && s.eutra_ctrl_region_size_r13.to_number() == 3, "control region");
  CHECK(s.nrs_crs_pwr_offset_r13_present && s.nrs_crs_pwr_offset_r13.to_number() == 0.0f, "NRS/CRS power offset");
  CHECK(s.si_win_len_r13.to_number() == 160 && s.si_radio_frame_offset_r13 == 1, "SI window / offset");
  CHECK(s.sched_info_list_r13.size() == 1, "one SI message");
  CHECK(s.sched_info_list_r13[0].si_periodicity_r13.to_number() == 1024, "SI periodicity");
  CHECK(s.sched_info_list_r13[0].si_repeat_pattern_r13.to_number() == 4, "SI repetition pattern");
  CHECK(s.sched_info_list_r13[0].si_tb_r13.to_number() == 208, "SI TB");
  CHECK(s.sched_info_list_r13[0].sib_map_info_r13.size() == 0, "SIB2 is implicit; mapping list must be empty");

  // Re-encode what we decoded: must reproduce the transmitted bits (guards against lossy fields).
  uint8_t       out[64] = {};
  asn1::bit_ref oref(out, sizeof(out));
  CHECK(dl.pack(oref) == asn1::SRSASN_SUCCESS, "re-pack");
  oref.align_bytes_zero();
  CHECK((size_t)oref.distance_bytes() == len && memcmp(out, tb.data(), len) == 0, "SIB1-NB not stable under re-encode");

  // A different cell must produce different bits (the builder is not returning a canned message).
  nbiot::cell_config c2;
  CHECK(load_string(replace(replace(BASE_CONF, "mnc = \"01\"", "mnc = \"99\""), "tac = 0x0001", "tac = 0x1234"), c2, err),
        "%s", err.c_str());
  std::vector<uint8_t> tb2;
  size_t               len2 = 0;
  CHECK(nbiot::pack_sib1(c2, 0xA5, tb2, len2, err), "%s", err.c_str());
  CHECK(tb2 != tb, "changing PLMN and TAC did not change the SIB1-NB");
}

/*************************************************************************************************/
static void expect_reject(const char* what, const std::string& text, const char* must_mention)
{
  nbiot::cell_config c;
  std::string        err;
  bool               ok = load_string(text, c, err);
  CHECK(!ok, "'%s' was accepted but must be rejected", what);
  if (!ok) {
    CHECK(err.find(must_mention) != std::string::npos,
          "'%s' rejected for the wrong reason: '%s' (expected it to mention '%s')",
          what,
          err.c_str(),
          must_mention);
  }
}

static void test_rejections()
{
  expect_reject("anchor PRB not on the raster", replace(BASE_CONF, "nbiot_prb = 17", "nbiot_prb = 3"), "nbiot_prb");
  expect_reject("anchor PRB out of range", replace(BASE_CONF, "nbiot_prb = 17", "nbiot_prb = 40"), "nbiot_prb");
  expect_reject("PCI mismatch in same-PCI mode", replace(BASE_CONF, "n_id_ncell = 1", "n_id_ncell = 2"), "n_id_ncell");
  expect_reject("unsupported operation mode",
                replace(BASE_CONF, "\"inband_same_pci\"", "\"standalone\""),
                "operation_mode");
  expect_reject("MNC with one digit", replace(BASE_CONF, "mnc = \"01\"", "mnc = \"1\""), "mnc");
  expect_reject("MCC with letters", replace(BASE_CONF, "mcc = \"234\"", "mcc = \"2x4\""), "mcc");
  expect_reject("qRxLevMin out of range", replace(BASE_CONF, "q_rx_lev_min = -70", "q_rx_lev_min = -10"), "q_rx_lev_min");
  expect_reject("TAC over 16 bits", replace(BASE_CONF, "tac = 0x0001", "tac = 0x10000"), "tac");
  expect_reject("reserved schedulingInfoSIB1", replace(BASE_CONF, "sched_info_sib1 = 0", "sched_info_sib1 = 12"),
                "sched_info_sib1");
  expect_reject("missing key", replace(BASE_CONF, "tac = 0x0001;", ""), "tac");
  expect_reject("wrong key type", replace(BASE_CONF, "cell_barred = false", "cell_barred = \"no\""), "cell_barred");
  expect_reject("SIB2 listed explicitly", replace(BASE_CONF, "sib_mapping = [ ]", "sib_mapping = [ 2 ]"), "SIB2");

  // Control region: 2 is what I originally (wrongly) wrote. It must not slip through in either form.
  expect_reject("cfi and SIB1 region disagree",
                replace(BASE_CONF, "eutra_control_region_size = 3", "eutra_control_region_size = 2"),
                "eutra_control_region_size");
  expect_reject("PHY cannot honour region size 2",
                replace(replace(BASE_CONF, "eutra_control_region_size = 3", "eutra_control_region_size = 2"), "cfi = 3", "cfi = 2"),
                "PHY");
}

/*************************************************************************************************/
static void test_overflow()
{
  nbiot::cell_config c;
  std::string        err;
  CHECK(load_string(BASE_CONF, c, err), "%s", err.c_str());

  // Cram SIB mapping entries into the first SI message until SIB1-NB no longer fits 208 bits.
  for (int i = 0; i < 30; ++i) {
    c.si_sched[0].sib_mapping.push_back(3);
  }
  std::vector<uint8_t> tb;
  size_t               len = 0;
  bool                 ok  = nbiot::pack_sib1(c, 0, tb, len, err);
  CHECK(!ok, "an oversized SIB1-NB must be rejected");
  CHECK(err.find("bits") != std::string::npos && err.find("schedulingInfoSIB1") != std::string::npos,
        "overflow error should explain the TBS limit: '%s'", err.c_str());

  // ... and fits once a larger schedulingInfoSIB1 (328 bits) is chosen.
  c.sched_info_sib1 = 3;
  ok                = nbiot::pack_sib1(c, 0, tb, len, err);
  CHECK(ok, "should fit 328 bits: %s", err.c_str());
  CHECK(tb.size() == 41, "328 bits = 41 bytes, got %zu", tb.size());
}

int main()
{
  printf("nbiot_sib_test\n");
  test_real_network_vector();
  test_config_load();
  test_anchor_geometry();
  test_mib_cross_check();
  test_sib1_roundtrip();
  test_rejections();
  test_overflow();
  printf("%d checks, %d failed\n", g_checks, g_fail);
  return g_fail == 0 ? 0 : 1;
}
