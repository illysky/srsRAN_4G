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
 * Hand-worked vectors and structural properties of the NB-IoT random access module. The randomised comparison with an
 * independent model is in nbiot_ra_check.py; here every expected byte was derived by hand from the field layout of
 * TS 36.321 Figure 6.1.5-3b and TS 36.213 16.3.3.
 */

#include "srsran/phy/phch/nbiot_ra.h"
#include <stdio.h>
#include <string.h>

static int g_fail   = 0;
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

static void test_ra_rnti(void)
{
  CHECK(srsran_nbiot_ra_rnti(0, 0) == 1, "sfn 0");
  CHECK(srsran_nbiot_ra_rnti(3, 0) == 1, "sfn 3 shares the RNTI of sfn 0");
  CHECK(srsran_nbiot_ra_rnti(4, 0) == 2, "sfn 4");
  CHECK(srsran_nbiot_ra_rnti(1023, 0) == 256, "sfn 1023");
  CHECK(srsran_nbiot_ra_rnti(8, 1) == 259, "sfn 8 on carrier 1: 1 + 2 + 256");
}

static void test_ta(void)
{
  CHECK(srsran_nbiot_ta_from_toa(0.0f) == 0, "0");
  CHECK(srsran_nbiot_ta_from_toa(-3.0f) == 0, "negative ToA (early arrival inside the search margin)");
  CHECK(srsran_nbiot_ta_from_toa(0.49f) == 0, "rounds down");
  CHECK(srsran_nbiot_ta_from_toa(0.51f) == 1, "rounds up");
  CHECK(srsran_nbiot_ta_from_toa(58.0f) == 58, "58 samples = 58 * 16 Ts = 483.75 us round trip = 72.5 km");
  CHECK(srsran_nbiot_ta_from_toa(1282.0f) == 1282, "largest index");
  CHECK(srsran_nbiot_ta_from_toa(1290.0f) == 1282, "clipped");
}

/// Bit field of the grant, worked out by hand:
///   1 | 010001 | 10 | 101 | 010  = 101 0001 1010 1010 = 0x51AA
///   (15 kHz, I_sc 17, I_delay 2, I_rep 5, I_MCS 2)
static void test_grant_bits(void)
{
  srsran_nbiot_msg3_grant_t g = {.sc_15khz = 1, .i_sc = 17, .i_delay = 2, .i_rep = 5, .i_mcs = 2};
  uint32_t                  bits;
  CHECK(srsran_nbiot_msg3_grant_pack(&g, &bits) == 0 && bits == 0x51AA, "packed %04x, expected 51AA", bits);
  srsran_nbiot_msg3_grant_t u;
  srsran_nbiot_msg3_grant_unpack(0x51AA, &u);
  CHECK(u.sc_15khz == 1 && u.i_sc == 17 && u.i_delay == 2 && u.i_rep == 5 && u.i_mcs == 2, "unpack of 51AA");

  // the very first bit is the spacing and the very last the MCS LSB
  srsran_nbiot_msg3_grant_t a = {.sc_15khz = 1};
  CHECK(srsran_nbiot_msg3_grant_pack(&a, &bits) == 0 && bits == 0x4000, "spacing is the MSB of 15 bits");
  srsran_nbiot_msg3_grant_t z = {.i_mcs = 1};
  CHECK(srsran_nbiot_msg3_grant_pack(&z, &bits) == 0 && bits == 0x0001, "MCS is the LSB");

  // reserved values
  srsran_nbiot_msg3_grant_t r = {.i_mcs = 3};
  CHECK(srsran_nbiot_msg3_grant_pack(&r, &bits) != 0, "I_MCS 3 is reserved");
  r = (srsran_nbiot_msg3_grant_t){.sc_15khz = 1, .i_sc = 19};
  CHECK(srsran_nbiot_msg3_grant_pack(&r, &bits) != 0, "I_sc 19 at 15 kHz is reserved");
  r = (srsran_nbiot_msg3_grant_t){.sc_15khz = 0, .i_sc = 48};
  CHECK(srsran_nbiot_msg3_grant_pack(&r, &bits) != 0, "I_sc 48 at 3.75 kHz is reserved");
  r = (srsran_nbiot_msg3_grant_t){.sc_15khz = 2};
  CHECK(srsran_nbiot_msg3_grant_pack(&r, &bits) != 0, "spacing bit 2");

  // every packed value survives an unpack
  int n_ok = 0;
  for (uint32_t v = 0; v < (1u << 15); ++v) {
    srsran_nbiot_msg3_grant_t q;
    srsran_nbiot_msg3_grant_unpack(v, &q);
    uint32_t back;
    if (srsran_nbiot_msg3_grant_pack(&q, &back) == 0) {
      CHECK(back == v, "grant %04x re-packs as %04x", v, back);
      ++n_ok;
    }
  }
  // valid: 3.75 kHz has 48 * 4 * 8 * 3 = 4608, 15 kHz has 19 * 4 * 8 * 3 = 1824
  CHECK(n_ok == 4608 + 1824, "%d valid grants, expected %d", n_ok, 4608 + 1824);
}

/// Grant -> NPUSCH: the values are read off the specification tables by hand.
static void test_grant_to_npusch(void)
{
  srsran_npusch_cfg_t cfg;
  uint32_t            k0;

  // 15 kHz, I_sc 13 = 3 tones starting at 3(13-12) = 3; I_MCS 1: QPSK, 3 RU; I_rep 3: 8 repetitions; I_delay 3: k0 = 64
  srsran_nbiot_msg3_grant_t g = {.sc_15khz = 1, .i_sc = 13, .i_delay = 3, .i_rep = 3, .i_mcs = 1};
  CHECK(srsran_nbiot_msg3_grant_to_npusch(&g, 0x4321, 77, &cfg, &k0) == 0, "valid grant refused");
  CHECK(cfg.n_sc == 3 && cfg.spacing_hz == 15000 && cfg.sc == 3 && cfg.n_ru == 3 && cfg.n_rep == 8 && cfg.qm == 2 &&
            cfg.tbs == 88 && cfg.rnti == 0x4321 && cfg.cell_id == 77 && cfg.rv == 0 && k0 == 64,
        "3-tone Msg3 mapped to n_sc %u sp %u sc %u n_ru %u n_rep %u qm %u tbs %u k0 %u",
        cfg.n_sc, cfg.spacing_hz, cfg.sc, cfg.n_ru, cfg.n_rep, cfg.qm, cfg.tbs, k0);

  // 3.75 kHz, I_sc 47, I_MCS 0: pi/2 BPSK, 4 RU, one repetition, k0 = 12
  g = (srsran_nbiot_msg3_grant_t){.sc_15khz = 0, .i_sc = 47, .i_delay = 0, .i_rep = 0, .i_mcs = 0};
  CHECK(srsran_nbiot_msg3_grant_to_npusch(&g, 1, 2, &cfg, &k0) == 0, "valid grant refused");
  CHECK(cfg.n_sc == 1 && cfg.spacing_hz == 3750 && cfg.sc == 47 && cfg.n_ru == 4 && cfg.n_rep == 1 && cfg.qm == 1 && k0 == 12,
        "BPSK Msg3 mapped to n_sc %u sp %u sc %u n_ru %u n_rep %u qm %u k0 %u",
        cfg.n_sc, cfg.spacing_hz, cfg.sc, cfg.n_ru, cfg.n_rep, cfg.qm, k0);

  // 15 kHz, I_sc 18 = all 12 tones; I_MCS 0: with more than one tone the modulation is QPSK for every entry, 4 RU
  g = (srsran_nbiot_msg3_grant_t){.sc_15khz = 1, .i_sc = 18, .i_delay = 1, .i_rep = 7, .i_mcs = 0};
  CHECK(srsran_nbiot_msg3_grant_to_npusch(&g, 1, 2, &cfg, &k0) == 0, "valid grant refused");
  CHECK(cfg.n_sc == 12 && cfg.sc == 0 && cfg.n_ru == 4 && cfg.n_rep == 128 && cfg.qm == 2 && k0 == 16,
        "12-tone Msg3 mapped to n_sc %u sc %u n_ru %u n_rep %u qm %u k0 %u", cfg.n_sc, cfg.sc, cfg.n_ru, cfg.n_rep, cfg.qm, k0);

  // 15 kHz single tone (I_sc 0..11), I_MCS 0: BPSK as well
  g = (srsran_nbiot_msg3_grant_t){.sc_15khz = 1, .i_sc = 11, .i_mcs = 0};
  srsran_nbiot_msg3_grant_to_npusch(&g, 1, 2, &cfg, &k0);
  CHECK(cfg.n_sc == 1 && cfg.sc == 11 && cfg.qm == 1, "15 kHz single tone with I_MCS 0 must be pi/2 BPSK");

  // 6 tones at I_sc 17 start at 6
  g = (srsran_nbiot_msg3_grant_t){.sc_15khz = 1, .i_sc = 17, .i_mcs = 2};
  srsran_nbiot_msg3_grant_to_npusch(&g, 1, 2, &cfg, &k0);
  CHECK(cfg.n_sc == 6 && cfg.sc == 6 && cfg.n_ru == 1, "6-tone Msg3");

  // reserved grants are refused and leave nothing half-filled that could be used
  g = (srsran_nbiot_msg3_grant_t){.sc_15khz = 1, .i_sc = 40};
  CHECK(srsran_nbiot_msg3_grant_to_npusch(&g, 1, 2, &cfg, &k0) != 0, "reserved I_sc accepted");
  CHECK(srsran_nbiot_msg3_grant_to_npusch(NULL, 1, 2, &cfg, &k0) != 0, "NULL grant accepted");
}

/// PDU with one RAR, worked out by hand.
///   subheader  E=0 T=1 RAPID=13          0100 1101 = 4D
///   TA 997 = 011 1110 0101, grant 0x51AA = 101 0001 1010 1010, TC-RNTI 0x1234
///   oct 1: R + TA[10:4] = 0 0111110      3E
///   oct 2: TA[3:0] 0101 + grant[14:11] 1010   5A
///   oct 3: grant[10:3] 0011 0101         35
///   oct 4: grant[2:0] 010 + 5 x R        0100 0000 = 40
///   oct 5,6: 12 34
static void test_rar_pdu(void)
{
  srsran_nbiot_rar_t rar = {.rapid = 13,
                            .ta    = 997,
                            .grant = {.sc_15khz = 1, .i_sc = 17, .i_delay = 2, .i_rep = 5, .i_mcs = 2},
                            .tc_rnti = 0x1234};
  static const uint8_t want[] = {0x4D, 0x3E, 0x5A, 0x35, 0x40, 0x12, 0x34};
  uint8_t              pdu[64];
  int                  n = srsran_nbiot_rar_pdu_pack(-1, &rar, 1, pdu, sizeof(pdu));
  CHECK(n == 7 && memcmp(pdu, want, 7) == 0, "single RAR PDU: length %d, bytes %02x %02x %02x %02x %02x %02x %02x", n, pdu[0],
        pdu[1], pdu[2], pdu[3], pdu[4], pdu[5], pdu[6]);

  // with a Backoff Indicator of 5 in front: E=1 T=0 R R BI = 1000 0101 = 85, and the RAPID subheader keeps E=0
  static const uint8_t want_bi[] = {0x85, 0x4D, 0x3E, 0x5A, 0x35, 0x40, 0x12, 0x34};
  n = srsran_nbiot_rar_pdu_pack(5, &rar, 1, pdu, sizeof(pdu));
  CHECK(n == 8 && memcmp(pdu, want_bi, 8) == 0, "PDU with Backoff Indicator");

  // two RARs: subheaders 1100 0000|13 = CD, 0100 0000|5 = 45, then both payloads
  srsran_nbiot_rar_t two[2] = {rar, rar};
  two[1].rapid              = 5;
  two[1].tc_rnti            = 0xABCD;
  n                         = srsran_nbiot_rar_pdu_pack(-1, two, 2, pdu, sizeof(pdu));
  CHECK(n == 14 && pdu[0] == 0xCD && pdu[1] == 0x45 && pdu[2] == 0x3E && pdu[8] == 0x3E && pdu[12] == 0xAB && pdu[13] == 0xCD,
        "two RARs: length %d, subheaders %02x %02x", n, pdu[0], pdu[1]);

  // Backoff Indicator alone is a legal RAR PDU (nothing to answer, only telling the UEs to back off)
  n = srsran_nbiot_rar_pdu_pack(9, NULL, 0, pdu, sizeof(pdu));
  CHECK(n == 1 && pdu[0] == 0x09, "BI-only PDU: %d bytes, %02x", n, pdu[0]);

  // parsing
  int                bi;
  srsran_nbiot_rar_t out[4];
  CHECK(srsran_nbiot_rar_pdu_unpack(want_bi, sizeof(want_bi), &bi, out, 4) == 1, "parse of the BI PDU");
  CHECK(bi == 5 && out[0].rapid == 13 && out[0].ta == 997 && out[0].grant.sc_15khz == 1 && out[0].grant.i_sc == 17 &&
            out[0].grant.i_delay == 2 && out[0].grant.i_rep == 5 && out[0].grant.i_mcs == 2 && out[0].tc_rnti == 0x1234,
        "parsed fields");
  CHECK(srsran_nbiot_rar_pdu_unpack(want, sizeof(want), &bi, out, 4) == 1 && bi == -1, "no BI reported for a PDU without one");

  // reserved bits are ignored on reception, trailing octets are padding
  uint8_t noisy[16];
  memcpy(noisy, want, sizeof(want));
  noisy[1] |= 0x80;          // R before the timing advance
  noisy[4] |= 0x1f;          // R x5 after the grant
  noisy[7] = 0xAA;           // padding
  noisy[8] = 0x55;
  CHECK(srsran_nbiot_rar_pdu_unpack(noisy, 9, &bi, out, 4) == 1 && out[0].ta == 997 && out[0].grant.i_mcs == 2 &&
            out[0].tc_rnti == 0x1234,
        "reserved bits / padding disturbed parsing");

  // malformed
  CHECK(srsran_nbiot_rar_pdu_unpack(want, 6, &bi, out, 4) < 0, "truncated payload accepted");
  CHECK(srsran_nbiot_rar_pdu_unpack(want, 0, &bi, out, 4) < 0, "empty PDU accepted");
  static const uint8_t header_only[] = {0xCD};
  CHECK(srsran_nbiot_rar_pdu_unpack(header_only, 1, &bi, out, 4) < 0, "E=1 with nothing following accepted");
  // more RARs than the caller has room for
  CHECK(srsran_nbiot_rar_pdu_unpack(pdu, 1, &bi, out, 0) >= 0, "BI-only PDU refused when room for zero RARs");
  static const uint8_t two_rar_pdu[] = {0xCD, 0x45, 0x3E, 0x5A, 0x35, 0x40, 0x12, 0x34, 0x3E, 0x5A, 0x35, 0x40, 0xAB, 0xCD};
  CHECK(srsran_nbiot_rar_pdu_unpack(two_rar_pdu, sizeof(two_rar_pdu), &bi, out, 1) < 0, "two RARs accepted with room for one");
  CHECK(srsran_nbiot_rar_pdu_unpack(two_rar_pdu, sizeof(two_rar_pdu), &bi, out, 4) == 2 && out[1].rapid == 5 &&
            out[1].tc_rnti == 0xABCD,
        "two RARs");
}

int main(void)
{
  printf("nbiot_ra_test\n");
  test_ra_rnti();
  test_ta();
  test_grant_bits();
  test_grant_to_npusch();
  test_rar_pdu();
  printf("%d checks, %d failed\n", g_checks, g_fail);
  return g_fail == 0 ? 0 : 1;
}
