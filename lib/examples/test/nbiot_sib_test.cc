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

/// Like CHECK, but abandons the current test function: later checks would dereference what this one just proved absent.
#define REQUIRE(cond, ...)                                                                                             \
  do {                                                                                                                 \
    ++g_checks;                                                                                                        \
    if (!(cond)) {                                                                                                     \
      ++g_fail;                                                                                                        \
      printf("FAIL %s:%d: ", __FILE__, __LINE__);                                                                      \
      printf(__VA_ARGS__);                                                                                             \
      printf("\n");                                                                                                    \
      return;                                                                                                          \
    }                                                                                                                  \
  } while (0)

// A SIB1-NB captured from a real in-band NB-IoT operator network (the payload the generator has always broadcast).
static const uint8_t real_network_sib1[] = {0x43, 0x4d, 0xd0, 0x92, 0x22, 0x06, 0x04, 0x30, 0x28,
                                            0x6e, 0x87, 0xd0, 0x4b, 0x13, 0x90, 0xb4, 0x12, 0xa1,
                                            0x02, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// NOTE: every SIB2 field deliberately has a value different from every other field of the same kind, so that a
// swapped or mis-wired field cannot go unnoticed. (The deployment config uses realistic, partly equal values.)
static const char* BASE_CONF = R"(
nbiot = {
  lte     = { n_prb = 25; pci = 1; nof_ports = 1; cfi = 3; dl_earfcn = 6300; };
  carrier = { operation_mode = "inband_same_pci"; nbiot_prb = 17; n_id_ncell = 1; nof_ports = 1; };
  cell    = { mcc = "234"; mnc = "01"; tac = 0x0001; cell_id = 0x019B01; band = 20;
              cell_barred = false; intra_freq_reselection = true; q_rx_lev_min = -70; };
  mib     = { sched_info_sib1 = 0; sys_info_tag = 0; ac_barring = false; };
  sib1    = { si_window_length = 160; si_radio_frame_offset = 1; nrs_crs_power_offset = 0;
              eutra_control_region_size = 3;
              sched_info = ( { si_periodicity = 1024; si_repetition_pattern = 4; si_tb = 208; sib_mapping = [ ]; } ); };

  sib2 = {
    rach = {
      preamble_trans_max_ce         = 20;
      preamble_init_rx_target_power = -104;  // dBm; same as the live LTE cell (sib.conf preamble_init_rx_target_pwr)
      power_ramping_step            = 6;     // dB
      ra_response_window            = 7;     // ra-ResponseWindowSize, in NPDCCH search-space periods
      mac_contention_timer          = 32;    // in NPDCCH search-space periods
    };

    // NPRACH: what the UE transmits Msg1 on. The Phase 2 detector must be configured from exactly these values.
    nprach = {
      cp_length_us                 = 66.7;   // 66.7 = preamble format 0, 266.7 = format 1
      periodicity_ms               = 640;
      start_time_ms                = 64;
      subcarrier_offset            = 18;     // first 3.75 kHz subcarrier of the NPRACH region
      nof_subcarriers              = 24;     // 12, 24, 36 or 48
      nof_ce_levels                = 1;      // only 1 is implemented
      msg3_subcarrier_range_start  = "oneThird"; // zero | oneThird | twoThird | one
      max_preamble_attempts        = 8;      // maxNumPreambleAttemptCE
      num_repetitions_per_preamble = 16;
      npdcch_num_repetitions_ra    = 128;      // Rmax of the Type-2 common search space
      npdcch_start_sf_css_ra       = 48;      // search-space period = this x Rmax subframes
      npdcch_offset_ra             = "oneEighth"; // zero | oneEighth | oneFourth | threeEighth
    };

    pcch = {
      default_paging_cycle_rf       = 256;   // RADIO FRAMES (128/256/512/1024)
      nB                            = "halfT";
      npdcch_num_repetitions_paging = 512;
    };

    npdsch = {
      // Reference power of one NRS RE. The UE computes path loss as (this - measured NRSRP) and sets its NPRACH
      // power from it, so a wrong value makes every UE transmit that many dB too loud or too quiet.
      // PLACEHOLDER: measure the NRS EPRE at the antenna port (in-band: LTE CRS EPRE + sib1.nrs_crs_power_offset).
      nrs_power_dbm = -20;
    };

    npusch = {
      ack_nack_num_repetitions_msg4 = 4;
      group_hopping_enabled         = true;
      group_assignment_npusch       = 11;
      // DMRS base sequence / cyclic shift are not signalled: the UE derives them from the cell id.
    };

    ul_power_control = {
      p0_nominal_npusch   = -85;   // dBm
      alpha               = 0.7;
      delta_preamble_msg3 = 2;     // dB
    };

    ue_timers = {
      t300 = 10000;   // ms
      t301 = 15000;
      t310 = 8000;
      n310 = 10;
      t311 = 20000;
      n311 = 5;
    };

    time_alignment_timer = "sf5120";
  };
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
  // Band 20, DL EARFCN 6300 (806.0 MHz) -> LTE UL centre 847.0 MHz; PRB 17 lies 5 PRB (900 kHz) above it: 847.9 MHz,
  // which is UL EARFCN 24309 (24150 + 159 raster steps above 832 MHz) with no offset.
  CHECK(c.ul_freq_khz == 847900 && c.ul_earfcn == 24309 && c.ul_offset_m == 0,
        "UL carrier of PRB 17: %u kHz, EARFCN %u, M_UL %d (want 847900 / 24309 / 0)",
        c.ul_freq_khz,
        c.ul_earfcn,
        c.ul_offset_m);
}

/// The forward relation of TS 36.101 5.7.3F, written out separately from the builder's inverse.
static double ul_freq_khz_of(uint32_t f_ul_low_khz, uint32_t n_offs_ul, uint32_t n_ul, int m_ul)
{
  return (double)f_ul_low_khz + 0.1 * 1000.0 * ((double)n_ul - (double)n_offs_ul) + 0.0025 * 1000.0 * 2.0 * m_ul;
}

static void test_ul_carrier()
{
  struct row {
    const char* what;
    uint32_t    band, dl_earfcn, lte_prb, prb;
    double      anchor_khz;   // worked out by hand from the PRB geometry (see comments)
    uint32_t    n_ul;
    int         m_ul;
  };
  // Band 20: F_UL_low 832 MHz, N_Offs-UL 24150; DL EARFCN 6300 -> LTE UL centre 832 + (6300-6150) * 0.1 = 847.0 MHz.
  // On a 25 PRB carrier PRB p is centred at (p - 12) * 180 kHz from the carrier centre (PRB 12 straddles it).
  const row rows[] = {
      {"25 PRB, PRB 2", 20, 6300, 25, 2, 847000 - 10 * 180, 24282, 0},
      {"25 PRB, PRB 7", 20, 6300, 25, 7, 847000 - 5 * 180, 24291, 0},
      {"25 PRB, PRB 17", 20, 6300, 25, 17, 847000 + 5 * 180, 24309, 0},
      {"25 PRB, PRB 22", 20, 6300, 25, 22, 847000 + 10 * 180, 24318, 0},
      // Even-PRB carriers put the centre between PRBs, so the anchor is 90 kHz off the raster and needs M_UL != 0.
      // 6 PRB, PRB 2: 2 PRB above the low edge => (2 + 0.5 - 3) * 180 = -90 kHz from the centre: 846.910 MHz
      //   = 846.9 + 0.010 -> 149 steps + 2 * 5 kHz.
      {"6 PRB, PRB 2 (positive offset)", 20, 6300, 6, 2, 847000 - 90, 24299, 2},
      // 50 PRB, PRB 5: (5 + 0.5 - 25) * 180 = -3510 kHz from the centre: 843.490 MHz = 843.5 - 0.010.
      {"50 PRB, PRB 5 (negative offset)", 20, 6300, 50, 5, 847000 - 3510, 24265, -2},
      // A different band, so the table is not only exercised on one row: band 3 (F_UL_low 1710, N_Offs-UL 19200),
      // DL EARFCN 1575 (N_Offs-DL 1200) -> UL centre 1710 + 37.5 = 1747.5 MHz.
      {"band 3, 25 PRB, PRB 17", 3, 1575, 25, 17, 1747500 + 5 * 180, 19584, 0}   // 1748.4 MHz: 384 steps above 1710 MHz,
  };
  for (const row& r : rows) {
    uint32_t    n_ul = 0, f_khz = 0;
    int         m_ul = 0;
    std::string err;
    CHECK(nbiot::derive_ul_carrier(r.band, r.dl_earfcn, r.lte_prb, r.prb, n_ul, m_ul, f_khz, err),
          "%s: %s",
          r.what,
          err.c_str());
    CHECK(n_ul == r.n_ul && m_ul == r.m_ul, "%s: got EARFCN %u / M_UL %d, want %u / %d", r.what, n_ul, m_ul, r.n_ul, r.m_ul);
    CHECK(f_khz == (uint32_t)r.anchor_khz, "%s: got %u kHz, want %.0f", r.what, f_khz, r.anchor_khz);
    // the pair must reproduce the anchor centre under the specification's forward formula
    const uint32_t f_ul_low   = r.band == 20 ? 832000 : 1710000;
    const uint32_t n_offs_ul  = r.band == 20 ? 24150 : 19200;
    CHECK(std::fabs(ul_freq_khz_of(f_ul_low, n_offs_ul, n_ul, m_ul) - r.anchor_khz) < 1e-6,
          "%s: (EARFCN, M_UL) gives %.3f kHz, not %.0f",
          r.what,
          ul_freq_khz_of(f_ul_low, n_offs_ul, n_ul, m_ul),
          r.anchor_khz);
  }

  // Not derivable: TDD band, band without UL/DL pairing, EARFCN below the band.
  uint32_t n_ul = 0, f_khz = 0;
  int      m_ul = 0;
  std::string err;
  CHECK(!nbiot::derive_ul_carrier(38, 38000, 25, 17, n_ul, m_ul, f_khz, err) && err.find("FDD") != std::string::npos,
        "TDD band accepted or wrong error: '%s'",
        err.c_str());
  CHECK(!nbiot::derive_ul_carrier(20, 6000, 25, 17, n_ul, m_ul, f_khz, err) && err.find("dl_earfcn") != std::string::npos,
        "EARFCN below the band accepted or wrong error: '%s'",
        err.c_str());
}

/// Every offset M_UL that can occur must survive the ASN.1 encoding: the enumeration is v-10..v-1, v-0dot5, v0, v1..v9,
/// so a naive index mapping is off by one on one side of zero.
static void test_ul_offset_encoding()
{
  nbiot::cell_config c;
  std::string        err;
  CHECK(load_string(BASE_CONF, c, err), "%s", err.c_str());
  for (int m = -10; m <= 9; ++m) {
    c.ul_offset_m = m;
    c.ul_earfcn   = 24000 + (uint32_t)(m + 10); // a distinct EARFCN per case, so earfcn and offset cannot be confused
    std::vector<uint8_t> tb;
    size_t               len = 0;
    REQUIRE(nbiot::pack_sib2(c, tb, len, err), "M_UL %d: %s", m, err.c_str());
    asn1::rrc::bcch_dl_sch_msg_nb_s dl;
    asn1::cbit_ref                  bref(tb.data(), tb.size());
    REQUIRE(dl.unpack(bref) == asn1::SRSASN_SUCCESS, "M_UL %d: does not unpack", m);
    const auto& s2 = dl.msg.c1().sys_info_r13().crit_exts.sys_info_r13().sib_type_and_info_r13[0].sib2_r13();
    const auto& ul = s2.freq_info_r13.ul_carrier_freq_r13;
    CHECK(ul.carrier_freq_r13 == c.ul_earfcn, "M_UL %d: EARFCN %u came back as %u", m, c.ul_earfcn, ul.carrier_freq_r13);
    CHECK(ul.carrier_freq_offset_r13_present && std::fabs(ul.carrier_freq_offset_r13.to_number() - (float)m) < 1e-6,
          "M_UL %d came back as %s",
          m,
          ul.carrier_freq_offset_r13.to_string());
  }
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
  REQUIRE(nbiot::pack_sib1(c, 0xA5, tb, len, err), "pack_sib1: %s", err.c_str());
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
  REQUIRE(dl.unpack(bref) == asn1::SRSASN_SUCCESS, "own SIB1-NB does not unpack");
  const asn1::rrc::sib_type1_nb_s& s = dl.msg.c1().sib_type1_r13();
  CHECK(s.hyper_sfn_msb_r13.to_number() == 0xA5, "hyperSFN");
  const auto& car = s.cell_access_related_info_r13;
  REQUIRE(car.plmn_id_list_r13.size() == 1, "one PLMN");
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
  REQUIRE(s.sched_info_list_r13.size() == 1, "one SI message");
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
  expect_reject("missing DL EARFCN", replace(BASE_CONF, " dl_earfcn = 6300;", ""), "dl_earfcn");
  expect_reject("TDD band (UL carrier not derivable)", replace(BASE_CONF, "band = 20", "band = 38"), "FDD");
  expect_reject("DL EARFCN outside the band", replace(BASE_CONF, "dl_earfcn = 6300", "dl_earfcn = 100"), "dl_earfcn");
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

/*************************************************************************************************/
static void test_sib2_roundtrip()
{
  nbiot::cell_config c;
  std::string        err;
  CHECK(load_string(BASE_CONF, c, err), "%s", err.c_str());

  std::vector<uint8_t> tb;
  size_t               len = 0;
  REQUIRE(nbiot::pack_sib2(c, tb, len, err), "pack_sib2: %s", err.c_str());
  CHECK(tb.size() == 26, "SI message transport block is %zu bytes, want 26 (si_tb 208)", tb.size());
  CHECK(len > 0 && len <= 26, "unpadded length %zu", len);
  for (size_t i = len; i < tb.size(); ++i) {
    CHECK(tb[i] == 0, "padding byte %zu is not zero", i);
  }

  asn1::rrc::bcch_dl_sch_msg_nb_s dl;
  asn1::cbit_ref                  bref(tb.data(), tb.size());
  REQUIRE(dl.unpack(bref) == asn1::SRSASN_SUCCESS, "own SI message does not unpack");
  CHECK(dl.msg.c1().type() == asn1::rrc::bcch_dl_sch_msg_type_nb_c::c1_c_::types::sys_info_r13, "not a SystemInformation-NB");
  const auto& ies = dl.msg.c1().sys_info_r13().crit_exts.sys_info_r13();
  REQUIRE(ies.sib_type_and_info_r13.size() == 1, "expected exactly SIB2-NB in the SI message");
  const asn1::rrc::sib_type2_nb_r13_s& s2 = ies.sib_type_and_info_r13[0].sib2_r13();
  const auto&                          rr = s2.rr_cfg_common_r13;

  CHECK(rr.rach_cfg_common_r13.preamb_trans_max_ce_r13.to_number() == 20, "preambleTransMax-CE");
  CHECK(rr.rach_cfg_common_r13.pwr_ramp_params_r13.preamb_init_rx_target_pwr.to_number() == -104, "target power");
  CHECK(rr.rach_cfg_common_r13.pwr_ramp_params_r13.pwr_ramp_step.to_number() == 6, "ramping step");
  REQUIRE(rr.rach_cfg_common_r13.rach_info_list_r13.size() == 1, "one RACH info entry");
  CHECK(rr.rach_cfg_common_r13.rach_info_list_r13[0].ra_resp_win_size_r13.to_number() == 7, "RA response window");
  CHECK(rr.rach_cfg_common_r13.rach_info_list_r13[0].mac_contention_resolution_timer_r13.to_number() == 32, "contention timer");

  CHECK(rr.pcch_cfg_r13.default_paging_cycle_r13.to_number() == 256, "paging cycle");
  CHECK(std::string(rr.pcch_cfg_r13.nb_r13.to_string()) == "halfT", "nB");
  CHECK(rr.pcch_cfg_r13.npdcch_num_repeat_paging_r13.to_number() == 512, "paging repetitions");

  CHECK(std::string(rr.nprach_cfg_r13.nprach_cp_len_r13.to_string()) == "us66dot7", "NPRACH CP length");
  REQUIRE(rr.nprach_cfg_r13.nprach_params_list_r13.size() == 1, "one NPRACH resource");
  const auto& np = rr.nprach_cfg_r13.nprach_params_list_r13[0];
  CHECK(np.nprach_periodicity_r13.to_number() == 640, "NPRACH periodicity");
  CHECK(np.nprach_start_time_r13.to_number() == 64, "NPRACH start time");
  CHECK(np.nprach_subcarrier_offset_r13.to_number() == 18, "NPRACH subcarrier offset");
  CHECK(np.nprach_num_subcarriers_r13.to_number() == 24, "NPRACH subcarriers");
  CHECK(std::string(np.nprach_subcarrier_msg3_range_start_r13.to_string()) == "oneThird", "Msg3 range start");
  CHECK(np.max_num_preamb_attempt_ce_r13.to_number() == 8, "max preamble attempts");
  CHECK(np.num_repeats_per_preamb_attempt_r13.to_number() == 16, "repetitions per preamble");
  CHECK(np.npdcch_num_repeats_ra_r13.to_number() == 128, "RA NPDCCH repetitions");
  CHECK(np.npdcch_start_sf_css_ra_r13.to_number() == 48.0f, "RA NPDCCH start SF");
  CHECK(std::string(np.npdcch_offset_ra_r13.to_string()) == "oneEighth", "RA NPDCCH offset");

  CHECK(rr.npdsch_cfg_common_r13.nrs_pwr_r13 == -20, "NRS power");
  CHECK(rr.npusch_cfg_common_r13.ack_nack_num_repeats_msg4_r13.size() == 1 &&
            rr.npusch_cfg_common_r13.ack_nack_num_repeats_msg4_r13[0].to_number() == 4,
        "ACK/NACK repetitions for Msg4");
  CHECK(!rr.npusch_cfg_common_r13.dmrs_cfg_r13_present, "DMRS config must not be signalled (UE derives from cell id)");
  CHECK(rr.npusch_cfg_common_r13.ul_ref_sigs_npusch_r13.group_hop_enabled_r13, "group hopping");
  CHECK(rr.npusch_cfg_common_r13.ul_ref_sigs_npusch_r13.group_assign_npusch_r13 == 11, "group assignment");
  CHECK(rr.ul_pwr_ctrl_common_r13.p0_nominal_npusch_r13 == -85, "P0 nominal NPUSCH");
  CHECK(std::fabs(rr.ul_pwr_ctrl_common_r13.alpha_r13.to_number() - 0.7f) < 1e-6, "alpha");
  CHECK(rr.ul_pwr_ctrl_common_r13.delta_preamb_msg3_r13 == 2, "delta preamble Msg3");
  // In-band: ul-CarrierFreq is mandatory and must name PRB 17's uplink carrier (847.9 MHz = EARFCN 24309, offset 0)
  REQUIRE(s2.freq_info_r13.ul_carrier_freq_r13_present, "ul-CarrierFreq missing (mandatory for in-band operation)");
  CHECK(s2.freq_info_r13.ul_carrier_freq_r13.carrier_freq_r13 == 24309, "ul-CarrierFreq is %u, want 24309",
        s2.freq_info_r13.ul_carrier_freq_r13.carrier_freq_r13);
  REQUIRE(s2.freq_info_r13.ul_carrier_freq_r13.carrier_freq_offset_r13_present, "ul-CarrierFreq offset missing");
  CHECK(std::string(s2.freq_info_r13.ul_carrier_freq_r13.carrier_freq_offset_r13.to_string()) == "v0",
        "ul-CarrierFreq offset is %s, want v0",
        s2.freq_info_r13.ul_carrier_freq_r13.carrier_freq_offset_r13.to_string());
  CHECK(std::string(s2.time_align_timer_common_r13.to_string()) == "sf5120", "time alignment timer");

  const auto& t = s2.ue_timers_and_consts_r13;
  CHECK(t.t300_r13.to_number() == 10000 && t.t301_r13.to_number() == 15000 && t.t310_r13.to_number() == 8000 &&
            t.n310_r13.to_number() == 10 && t.t311_r13.to_number() == 20000 && t.n311_r13.to_number() == 5,
        "UE timers and constants");

  // re-encode stability
  uint8_t       out[64] = {};
  asn1::bit_ref oref(out, sizeof(out));
  CHECK(dl.pack(oref) == asn1::SRSASN_SUCCESS, "re-pack");
  oref.align_bytes_zero();
  CHECK((size_t)oref.distance_bytes() == len && memcmp(out, tb.data(), len) == 0, "SI message not stable under re-encode");

  // changing a value that matters to the UE changes the bits
  nbiot::cell_config c2;
  CHECK(load_string(replace(BASE_CONF, "periodicity_ms               = 640", "periodicity_ms               = 1280"), c2, err), "%s", err.c_str());
  std::vector<uint8_t> tb2;
  size_t               len2 = 0;
  CHECK(nbiot::pack_sib2(c2, tb2, len2, err), "%s", err.c_str());
  CHECK(tb2 != tb, "changing the NPRACH periodicity did not change SIB2-NB");
}

/// Load may succeed while packing fails (value valid as a number but not encodable in ASN.1); either way the config
/// must be refused, and the message must point at the offending key.
static void expect_refused(const char* what, const std::string& text, const char* must_mention)
{
  nbiot::cell_config c;
  std::string        err;
  bool               ok = load_string(text, c, err);
  if (ok) {
    std::vector<uint8_t> tb;
    size_t               len = 0;
    ok = nbiot::pack_sib1(c, 0, tb, len, err) && nbiot::pack_sib2(c, tb, len, err);
  }
  CHECK(!ok, "'%s' was accepted but must be refused", what);
  if (!ok) {
    CHECK(err.find(must_mention) != std::string::npos,
          "'%s' refused for the wrong reason: '%s' (expected it to mention '%s')",
          what,
          err.c_str(),
          must_mention);
  }
}

static void test_sib2_rejections()
{
  expect_refused("NPRACH periodicity not in the ASN.1 set",
                 replace(BASE_CONF, "periodicity_ms               = 640", "periodicity_ms               = 700"),
                 "nprach.periodicity_ms");
  expect_refused("alpha not in the ASN.1 set", replace(BASE_CONF, "alpha               = 0.7", "alpha               = 0.75"), "alpha");
  expect_refused("unknown nB", replace(BASE_CONF, "nB                            = \"halfT\"", "nB                            = \"bogus\""), "nB");
  expect_refused("two CE levels", replace(BASE_CONF, "nof_ce_levels                = 1", "nof_ce_levels                = 2"), "nof_ce_levels");
  expect_refused("NPRACH subcarriers overflow the carrier",
                 replace(BASE_CONF, "subcarrier_offset            = 18", "subcarrier_offset            = 36"),
                 "48");
  expect_refused("NRS power out of range", replace(BASE_CONF, "nrs_power_dbm = -20", "nrs_power_dbm = 80"), "nrs_power_dbm");
  expect_refused("unknown NPRACH CP length", replace(BASE_CONF, "cp_length_us                 = 66.7", "cp_length_us                 = 100"), "cp_length_us");
  expect_refused("unknown Msg3 range start",
                 replace(BASE_CONF, "msg3_subcarrier_range_start  = \"oneThird\"", "msg3_subcarrier_range_start  = \"half\""),
                 "msg3_subcarrier_range_start");
  expect_refused("paging cycle not in the ASN.1 set",
                 replace(BASE_CONF, "default_paging_cycle_rf       = 256", "default_paging_cycle_rf       = 2048"),
                 "default_paging_cycle_rf");
  expect_refused("missing UE timer", replace(BASE_CONF, "t311 = 20000;", ""), "t311");
  expect_refused("P0 out of range", replace(BASE_CONF, "p0_nominal_npusch   = -85", "p0_nominal_npusch   = -200"), "p0_nominal_npusch");

  // SIB2-NB does not fit a 56-bit SI message; the error must say which si_tb would.
  expect_refused("si_tb too small for SIB2-NB", replace(BASE_CONF, "si_tb = 208", "si_tb = 56"), "smallest that fits is 208");
}

int main()
{
  printf("nbiot_sib_test\n");
  test_real_network_vector();
  test_config_load();
  test_anchor_geometry();
  test_ul_carrier();
  test_ul_offset_encoding();
  test_mib_cross_check();
  test_sib1_roundtrip();
  test_sib2_roundtrip();
  test_rejections();
  test_sib2_rejections();
  test_overflow();
  printf("%d checks, %d failed\n", g_checks, g_fail);
  return g_fail == 0 ? 0 : 1;
}
