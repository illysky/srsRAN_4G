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
 * NB-IoT NPUSCH format 1 receiver, TS 36.211 v14.2.0 clause 10.1.3 - 10.1.5, TS 36.212 v14.2.0 clause 5.1 / 6.3.2.
 *
 * Signal flow
 *   1. every SC-FDMA symbol is transformed with an FFT over the body after the cyclic prefix. The window starts a few
 *      samples inside the prefix so that small timing errors only turn into a phase slope, which the channel estimate
 *      absorbs. The uplink is shifted by half a sub-carrier: the body is multiplied by exp(-j pi n / N) first.
 *   2. single tone: the known phase rotation rho * (l mod 2) + phi_hat(l) of clause 10.1.5 is removed from every symbol
 *   3. the reference symbol of every slot gives a least squares channel estimate per tone. The phase progression of
 *      consecutive estimates gives the frequency offset, the estimates are averaged over a sliding window after
 *      being rotated to the slot they are used in, and the residual gives the noise variance.
 *   4. data symbols are equalised (multi tone: also transform de-precoded), turned into soft bits and descrambled per
 *      codeword pass. Every repetition adds its soft bits to the same coded bit positions.
 *   5. per resource unit channel de-interleaver, rate de-matching, turbo decoding with CRC24A early stop
 */

#include "srsran/phy/phch/npusch.h"
#include "npusch_tables.h"
#include "srsran/phy/fec/cbsegm.h"
#include "srsran/phy/fec/turbo/rm_turbo.h"
#include "srsran/phy/utils/bit.h"
#include "srsran/phy/utils/debug.h"
#include "srsran/phy/utils/vector.h"
#include <complex.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define NPUSCH_TS_HZ 30720000.0 // 1 / Ts
#define NPUSCH_DEFAULT_WINDOW 32
#define NPUSCH_DEFAULT_ITERATIONS 10
#define NPUSCH_LLR_TARGET 48.0f // mean |soft bit| after scaling to 16 bit
#define NPUSCH_LLR_CLIP 127.0f  // keeps the sum of the repeated bits of the rate de-matcher inside 16 bit
#define NPUSCH_SEQ_MAX_LEN 32768u

// ---------------------------------------------------------------- geometry

static bool is_single_tone(const srsran_npusch_cfg_t* c)
{
  return c->n_sc == 1;
}

static uint32_t slots_per_ru(const srsran_npusch_cfg_t* c)
{
  if (c->spacing_hz == 3750) {
    return 16;
  }
  switch (c->n_sc) {
    case 1:
      return 16;
    case 3:
      return 8;
    case 6:
      return 4;
    default:
      return 2;
  }
}

static uint32_t data_syms_ru(const srsran_npusch_cfg_t* c)
{
  return 6 * slots_per_ru(c) * c->n_sc;
}

static uint32_t qm_of(const srsran_npusch_cfg_t* c)
{
  return is_single_tone(c) ? c->qm : 2;
}

static uint32_t m_identical(const srsran_npusch_cfg_t* c)
{
  if (is_single_tone(c)) {
    return 1;
  }
  uint32_t m = (c->n_rep + 1) / 2;
  return m > 4 ? 4 : m;
}

static uint32_t slots_unit(const srsran_npusch_cfg_t* c) // N_slots of clause 10.1.3.6
{
  return c->spacing_hz == 3750 ? 1 : 2;
}

static uint32_t slots_per_frame(const srsran_npusch_cfg_t* c)
{
  return c->spacing_hz == 3750 ? 5 : 20;
}

static uint32_t dmrs_symbol(const srsran_npusch_cfg_t* c)
{
  return c->spacing_hz == 3750 ? 4 : 3;
}

static uint32_t fft_size(const srsran_npusch_cfg_t* c) // samples of the symbol body at 1.92 MS/s
{
  return c->spacing_hz == 3750 ? 512 : 128;
}

// cyclic prefix in samples (16 Ts each)
static uint32_t cp_samples(const srsran_npusch_cfg_t* c, uint32_t l)
{
  if (c->spacing_hz == 3750) {
    return 16;
  }
  return l == 0 ? 10 : 9;
}

// FFT window starts this many samples before the end of the cyclic prefix
static uint32_t win_shift(const srsran_npusch_cfg_t* c)
{
  return c->spacing_hz == 3750 ? 8 : 4;
}

uint32_t srsran_npusch_slot_len(const srsran_npusch_cfg_t* c)
{
  return c->spacing_hz == 3750 ? 3840 : 960;
}

uint32_t srsran_npusch_nof_slots(const srsran_npusch_cfg_t* c)
{
  return c->n_rep * c->n_ru * slots_per_ru(c);
}

uint32_t srsran_npusch_nof_samples(const srsran_npusch_cfg_t* c)
{
  return srsran_npusch_nof_slots(c) * srsran_npusch_slot_len(c);
}

uint32_t srsran_npusch_nof_coded_bits(const srsran_npusch_cfg_t* c)
{
  return c->n_ru * data_syms_ru(c) * qm_of(c);
}

int srsran_npusch_check_cfg(const srsran_npusch_cfg_t* c)
{
  if (c == NULL) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  if (c->n_sc != 1 && c->n_sc != 3 && c->n_sc != 6 && c->n_sc != 12) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  if (c->spacing_hz != 3750 && c->spacing_hz != 15000) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  if (c->spacing_hz == 3750 && c->n_sc != 1) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  uint32_t n_carriers = c->spacing_hz == 3750 ? 48 : 12;
  if (c->sc + c->n_sc > n_carriers || (c->n_sc > 1 && c->sc % c->n_sc != 0)) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  if (c->n_ru < 1 || c->n_ru > SRSRAN_NPUSCH_MAX_RU) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  if (c->n_rep < 1 || c->n_rep > SRSRAN_NPUSCH_MAX_REP || (c->n_rep & (c->n_rep - 1)) != 0) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  if (c->rv != 0 && c->rv != 2) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  if (is_single_tone(c) ? (c->qm != 1 && c->qm != 2) : c->qm != 2) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  if (c->tbs == 0 || c->tbs > SRSRAN_NPUSCH_MAX_TBS || !srsran_cbsegm_cbsize_isvalid(c->tbs + 24)) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  if (c->cell_id >= SRSRAN_NUM_PCI || c->slot >= slots_per_frame(c) || c->rnti > 0xffff) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  if (c->group_hopping && !is_single_tone(c)) {
    return SRSRAN_ERROR_INVALID_INPUTS; // sequence group hopping of multi tone reference signals is not implemented
  }
  if (srsran_npusch_nof_slots(c) * c->n_sc > SRSRAN_NPUSCH_MAX_TONE_SLOTS) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  if (srsran_npusch_nof_coded_bits(c) > SRSRAN_NPUSCH_MAX_CODED_BITS) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  if (c->delta_ss > 29) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  if ((c->n_sc == 3 && c->cyclic_shift > 2) || (c->n_sc == 6 && c->cyclic_shift > 3)) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  return SRSRAN_SUCCESS;
}

// ---------------------------------------------------------------- init / free

void srsran_npusch_free(srsran_npusch_t* q)
{
  if (q == NULL) {
    return;
  }
  srsran_dft_plan_free(&q->fft_128);
  srsran_dft_plan_free(&q->fft_512);
  srsran_tdec_free(&q->tdec);
  srsran_sequence_free(&q->seq_dmrs);
  srsran_sequence_free(&q->seq_gh);
  srsran_sequence_free(&q->seq_scr);
  free(q->win);
  free(q->spec);
  free(q->rot);
  free(q->y);
  free(q->h_raw);
  free(q->h_sm);
  free(q->llr_f);
  free(q->llr_e);
  free(q->llr_d);
  free(q->data);
  memset(q, 0, sizeof(srsran_npusch_t));
}

static int init_fft(srsran_dft_plan_t* p, uint32_t n)
{
  if (srsran_dft_plan_c(p, n, SRSRAN_DFT_FORWARD)) {
    return SRSRAN_ERROR;
  }
  srsran_dft_plan_set_mirror(p, false);
  srsran_dft_plan_set_dc(p, false);
  srsran_dft_plan_set_norm(p, false);
  return SRSRAN_SUCCESS;
}

int srsran_npusch_init(srsran_npusch_t* q)
{
  if (q == NULL) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  memset(q, 0, sizeof(srsran_npusch_t));

  if (init_fft(&q->fft_128, 128) || init_fft(&q->fft_512, 512)) {
    srsran_npusch_free(q);
    return SRSRAN_ERROR;
  }
  if (srsran_tdec_init(&q->tdec, SRSRAN_TCOD_MAX_LEN_CB) ||
      srsran_crc_init(&q->crc24a, SRSRAN_LTE_CRC24A, 24) ||
      srsran_sequence_init(&q->seq_dmrs, NPUSCH_SEQ_MAX_LEN) || srsran_sequence_init(&q->seq_gh, 256) ||
      srsran_sequence_init(&q->seq_scr, 4096)) {
    srsran_npusch_free(q);
    return SRSRAN_ERROR;
  }
  srsran_rm_turbo_gentables();

  q->win   = srsran_vec_cf_malloc(512);
  q->spec  = srsran_vec_cf_malloc(512);
  q->rot   = srsran_vec_cf_malloc(128 + 512);
  q->y     = srsran_vec_cf_malloc(7 * SRSRAN_NPUSCH_MAX_TONE_SLOTS);
  q->h_raw = srsran_vec_cf_malloc(SRSRAN_NPUSCH_MAX_TONE_SLOTS);
  q->h_sm  = srsran_vec_cf_malloc(SRSRAN_NPUSCH_MAX_TONE_SLOTS);
  q->llr_f = srsran_vec_f_malloc(SRSRAN_NPUSCH_MAX_CODED_BITS);
  q->llr_e = srsran_vec_i16_malloc(SRSRAN_NPUSCH_MAX_CODED_BITS + 64);
  q->llr_d = srsran_vec_i16_malloc(3 * SRSRAN_TCOD_MAX_LEN_CB + 12 + 64);
  q->data  = srsran_vec_u8_malloc(SRSRAN_TCOD_MAX_LEN_CB / 8 + 64);
  if (!q->win || !q->spec || !q->rot || !q->y || !q->h_raw || !q->h_sm || !q->llr_f || !q->llr_e || !q->llr_d ||
      !q->data) {
    srsran_npusch_free(q);
    return SRSRAN_ERROR;
  }
  // exp(-j pi n / N): moves the half sub-carrier offset of the uplink grid onto the FFT bins
  for (uint32_t n = 0; n < 128; n++) {
    q->rot[n] = cexpf(-I * (float)M_PI * (float)n / 128.0f);
  }
  for (uint32_t n = 0; n < 512; n++) {
    q->rot[128 + n] = cexpf(-I * (float)M_PI * (float)n / 512.0f);
  }
  return SRSRAN_SUCCESS;
}

// ---------------------------------------------------------------- front end

/* FFT of one symbol; `sym` points at the first sample of its cyclic prefix. Output: one value per tone, scaled such
 * that a tone of unit amplitude and a flat channel gives the channel gain. */
static void demod_symbol(srsran_npusch_t*           q,
                         const srsran_npusch_cfg_t* c,
                         const cf_t*                sym,
                         uint32_t                   l,
                         cf_t*                      tones)
{
  uint32_t n     = fft_size(c);
  const cf_t* w  = sym + cp_samples(c, l) - win_shift(c);
  const cf_t* r  = q->rot + (n == 128 ? 0 : 128);
  for (uint32_t m = 0; m < n; m++) {
    q->win[m] = w[m] * r[m];
  }
  srsran_dft_run_c(n == 128 ? &q->fft_128 : &q->fft_512, q->win, q->spec);
  for (uint32_t i = 0; i < c->n_sc; i++) {
    int k;
    if (is_single_tone(c)) {
      k = (int)c->sc - (c->spacing_hz == 3750 ? 24 : 6);
    } else {
      k = (int)(c->sc + i) - 6;
    }
    tones[i] = q->spec[(k + (int)n) % (int)n] / (float)n;
  }
}

/* Reference signal value of one tone; `slot` is the index of the slot in the transmission, `u` the base sequence
 * (single tone) */
static cf_t dmrs_single(const srsran_npusch_t* q, uint32_t slot, uint32_t u)
{
  int w = npusch_w16[u][slot % 16];
  return (1.0f + I) * (float)M_SQRT1_2 * (float)(1 - 2 * (q->seq_dmrs.c[slot] & 1)) * (float)w;
}

static const int8_t* phi_table(uint32_t n_sc, uint32_t u)
{
  switch (n_sc) {
    case 3:
      return npusch_phi_3[u];
    case 6:
      return npusch_phi_6[u];
    default:
      return npusch_phi_12[u];
  }
}

static cf_t dmrs_multi(const srsran_npusch_cfg_t* c, uint32_t u, uint32_t n)
{
  float alpha = 0.0f;
  if (c->n_sc == 3) {
    alpha = 2.0f * (float)M_PI * (float)c->cyclic_shift / 3.0f; // 0, 2pi/3, 4pi/3
  } else if (c->n_sc == 6) {
    static const float mult[4] = {0.0f, 1.0f, 2.0f, 4.0f}; // in units of pi / 3: 0, 2pi/6, 4pi/6, 8pi/6
    alpha                      = mult[c->cyclic_shift & 3] * (float)M_PI / 3.0f;
  }
  return cexpf(I * alpha * (float)n) * cexpf(I * (float)phi_table(c->n_sc, u)[n] * (float)M_PI / 4.0f);
}

// ---------------------------------------------------------------- decoder

static float clampf(float v, float lo, float hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

/* Maximum likelihood estimate of the phase advance per slot of the channel estimates h[slot][tone]: the peak of the
 * periodogram over all slots (power summed over the tones), refined by parabolic interpolation. Unlike the phase of
 * adjacent-slot products this keeps working when the SNR of a single slot is far below 0 dB, because the processing
 * gain grows with the number of slots. */
static int estimate_psi(const cf_t* h, uint32_t n_slots, uint32_t n_sc, float* psi)
{
  *psi = 0.0f;
  if (n_slots < 2) {
    return SRSRAN_SUCCESS;
  }
  uint32_t m = 64;
  while (m < 8 * n_slots) {
    m <<= 1;
  }
  srsran_dft_plan_t plan;
  cf_t*             in  = srsran_vec_cf_malloc(m);
  cf_t*             out = srsran_vec_cf_malloc(m);
  float*            pw  = srsran_vec_f_malloc(m);
  if (!in || !out || !pw || srsran_dft_plan_c(&plan, m, SRSRAN_DFT_FORWARD)) {
    free(in);
    free(out);
    free(pw);
    return SRSRAN_ERROR;
  }
  srsran_dft_plan_set_mirror(&plan, false);
  srsran_dft_plan_set_dc(&plan, false);
  srsran_dft_plan_set_norm(&plan, false);
  memset(pw, 0, sizeof(float) * m);
  for (uint32_t i = 0; i < n_sc; i++) {
    memset(in, 0, sizeof(cf_t) * m);
    for (uint32_t s = 0; s < n_slots; s++) {
      in[s] = h[s * n_sc + i];
    }
    srsran_dft_run_c(&plan, in, out);
    for (uint32_t k = 0; k < m; k++) {
      pw[k] += crealf(out[k]) * crealf(out[k]) + cimagf(out[k]) * cimagf(out[k]);
    }
  }
  uint32_t kmax = 0;
  for (uint32_t k = 1; k < m; k++) {
    if (pw[k] > pw[kmax]) {
      kmax = k;
    }
  }
  float a = sqrtf(pw[(kmax + m - 1) % m]), b = sqrtf(pw[kmax]), d = sqrtf(pw[(kmax + 1) % m]);
  float den   = a - 2.0f * b + d;
  float delta = den < -1e-20f ? 0.5f * (a - d) / den : 0.0f;
  delta       = clampf(delta, -0.5f, 0.5f);
  float bin   = (float)kmax + delta;
  if (bin >= (float)m / 2.0f) {
    bin -= (float)m;
  }
  *psi = 2.0f * (float)M_PI * bin / (float)m;
  srsran_dft_plan_free(&plan);
  free(in);
  free(out);
  free(pw);
  return SRSRAN_SUCCESS;
}

int srsran_npusch_decode(srsran_npusch_t*           q,
                         const srsran_npusch_cfg_t* c,
                         const cf_t*                samples,
                         uint8_t*                   tb,
                         srsran_npusch_res_t*       res)
{
  if (q == NULL || c == NULL || samples == NULL || tb == NULL || res == NULL ||
      srsran_npusch_check_cfg(c) != SRSRAN_SUCCESS) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  memset(res, 0, sizeof(*res));
  res->snr_db = NAN;

  const uint32_t n_sc      = c->n_sc;
  const uint32_t n_slots   = srsran_npusch_nof_slots(c);
  const uint32_t slot_len  = srsran_npusch_slot_len(c);
  const uint32_t l_dmrs    = dmrs_symbol(c);
  const uint32_t qm        = qm_of(c);
  const uint32_t n_fft     = fft_size(c);
  const uint32_t spf       = slots_per_frame(c);
  const uint32_t first_abs = c->frame * spf + c->slot;

  // ---- 1. front end: FFT of every symbol, rotation removal for a single tone
  uint32_t sym_start[7];
  uint32_t off = 0;
  for (uint32_t l = 0; l < 7; l++) {
    sym_start[l] = off;
    off += cp_samples(c, l) + n_fft;
  }
  double   phi_hat = 0.0;
  uint32_t lt      = 0;
  double   df      = (double)c->spacing_hz;
  int      k_single = (int)c->sc - (c->spacing_hz == 3750 ? 24 : 6);
  double   rho     = c->qm == 1 ? M_PI / 2.0 : M_PI / 4.0;
  uint32_t n_ts    = c->spacing_hz == 3750 ? 8192 : 2048;
  for (uint32_t s = 0; s < n_slots; s++) {
    for (uint32_t l = 0; l < 7; l++) {
      cf_t* t = q->y + ((size_t)s * 7 + l) * n_sc;
      demod_symbol(q, c, samples + (size_t)s * slot_len + sym_start[l], l, t);
      if (is_single_tone(c)) {
        uint32_t cp_ts = (c->spacing_hz == 3750) ? 256 : (l == 0 ? 160 : 144);
        if (lt > 0) {
          phi_hat += 2.0 * M_PI * df * ((double)k_single + 0.5) * (double)(n_ts + cp_ts) / NPUSCH_TS_HZ;
          phi_hat = fmod(phi_hat, 2.0 * M_PI);
        }
        double phi = rho * (double)(lt % 2) + phi_hat;
        t[0] *= cexpf(-I * (float)phi);
        lt++;
      }
    }
  }

  // ---- 2. reference signal, least squares channel estimate per slot and tone
  if (is_single_tone(c)) {
    if (srsran_sequence_set_LTE_pr(&q->seq_dmrs, n_slots, 35)) {
      return SRSRAN_ERROR;
    }
    if (c->group_hopping) {
      if (srsran_sequence_set_LTE_pr(&q->seq_gh, 8 * spf + 8, c->cell_id / 16)) {
        return SRSRAN_ERROR;
      }
    }
  }
  const uint32_t f_ss = (c->cell_id + c->delta_ss) % 16;
  for (uint32_t s = 0; s < n_slots; s++) {
    const cf_t* yd = q->y + ((size_t)s * 7 + l_dmrs) * n_sc;
    if (is_single_tone(c)) {
      uint32_t u = c->cell_id % 16;
      if (c->group_hopping) {
        uint32_t ru       = s / slots_per_ru(c);
        uint32_t ns_first = (first_abs + ru * slots_per_ru(c)) % spf;
        uint32_t f_gh     = 0;
        for (uint32_t i = 0; i < 8; i++) {
          f_gh += (uint32_t)(q->seq_gh.c[8 * ns_first + i] & 1) << i;
        }
        u = (f_gh % 16 + f_ss) % 16;
      }
      q->h_raw[s] = yd[0] * conjf(dmrs_single(q, s, u));
    } else {
      uint32_t mod = c->n_sc == 3 ? 12 : (c->n_sc == 6 ? 14 : 30);
      uint32_t u   = c->base_seq < 0 ? c->cell_id % mod : (uint32_t)c->base_seq % mod;
      for (uint32_t i = 0; i < n_sc; i++) {
        q->h_raw[s * n_sc + i] = yd[i] * conjf(dmrs_multi(c, u, i));
      }
    }
  }

  // frequency offset: the estimates follow h(s) = a exp(j psi s), psi in radians per slot
  double slot_seconds = (double)slot_len / (double)SRSRAN_NPUSCH_SRATE_HZ;
  float  psi          = 0.0f;
  if (estimate_psi(q->h_raw, n_slots, n_sc, &psi) != SRSRAN_SUCCESS) {
    return SRSRAN_ERROR;
  }
  res->cfo_hz = psi / (float)(2.0 * M_PI * slot_seconds);

  // sliding window average, every estimate rotated to the slot it is used for
  uint32_t win = c->chest_window ? c->chest_window : NPUSCH_DEFAULT_WINDOW;
  if (win > n_slots) {
    win = n_slots;
  }
  double resid = 0.0;
  double power = 0.0;
  for (uint32_t s = 0; s < n_slots; s++) {
    int start = (int)s - (int)(win / 2);
    if (start < 0) {
      start = 0;
    }
    if (start + (int)win > (int)n_slots) {
      start = (int)n_slots - (int)win;
    }
    for (uint32_t i = 0; i < n_sc; i++) {
      cf_t sum = 0.0f;
      for (uint32_t j = (uint32_t)start; j < (uint32_t)start + win; j++) {
        sum += q->h_raw[j * n_sc + i] * cexpf(I * psi * ((float)s - (float)j));
      }
      cf_t h = sum / (float)win;
      q->h_sm[s * n_sc + i] = h;
      float d = cabsf(q->h_raw[s * n_sc + i] - h);
      resid += (double)d * d;
      power += (double)cabsf(h) * cabsf(h);
    }
  }
  power /= (double)(n_slots * n_sc);
  float noise = 1.0f;
  if (c->noise_var > 0.0f) {
    noise = c->noise_var;
  } else if (win > 1) {
    // an estimate that contains the sample itself leaves 1 - 1/win of its noise in the residual
    noise = (float)(resid / ((double)(n_slots * n_sc) * (1.0 - 1.0 / (double)win)));
    if (noise < 1e-9f * (float)power) {
      noise = 1e-9f * (float)power;
    }
  }
  res->noise_var = noise;
  if (c->noise_var > 0.0f || win > 1) {
    res->snr_db = 10.0f * log10f((float)power / noise);
  }

  // ---- 3. equalisation, soft bits and repetition combining
  uint32_t m_ident   = m_identical(c);
  uint32_t n_pass    = c->n_rep / m_ident;
  uint32_t unit      = slots_unit(c);
  uint32_t pass_len  = c->n_ru * slots_per_ru(c) * m_ident; // slots of one codeword pass
  uint32_t f_bits    = srsran_npusch_nof_coded_bits(c);

  memset(q->llr_f, 0, sizeof(float) * f_bits);

  // time of every symbol centre relative to the reference symbol, in slot units (for the frequency offset)
  float dt_slot[7];
  for (uint32_t l = 0; l < 7; l++) {
    float centre_l = (float)(sym_start[l] + cp_samples(c, l)) + (float)n_fft / 2.0f;
    float centre_d = (float)(sym_start[l_dmrs] + cp_samples(c, l_dmrs)) + (float)n_fft / 2.0f;
    dt_slot[l]     = (centre_l - centre_d) / (float)slot_len;
  }

  cf_t idft_tw[SRSRAN_NPUSCH_MAX_TONES * SRSRAN_NPUSCH_MAX_TONES];
  if (n_sc > 1) {
    for (uint32_t i = 0; i < n_sc; i++) {
      for (uint32_t m = 0; m < n_sc; m++) {
        idft_tw[i * n_sc + m] = cexpf(I * 2.0f * (float)M_PI * (float)((i * m) % n_sc) / (float)n_sc);
      }
    }
  }

  for (uint32_t p = 0; p < n_pass; p++) {
    uint32_t abs0 = first_abs + p * pass_len;
    uint32_t nf   = (abs0 / spf) % 1024;
    uint32_t ns   = abs0 % spf;
    uint32_t cinit = (c->rnti << 14) + ((nf % 2) << 13) + ((ns / 2) << 9) + c->cell_id;
    if (srsran_sequence_set_LTE_pr(&q->seq_scr, f_bits, cinit)) {
      return SRSRAN_ERROR;
    }
    const uint8_t* scr = q->seq_scr.c;

    for (uint32_t qs = 0; qs < pass_len; qs++) {
      uint32_t s    = p * pass_len + qs;
      uint32_t blk  = qs / (unit * m_ident);
      uint32_t r    = qs % (unit * m_ident);
      uint32_t cslot = blk * unit + (r % unit);
      for (uint32_t l = 0; l < 7; l++) {
        if (l == l_dmrs) {
          continue;
        }
        uint32_t    ld   = l < l_dmrs ? l : l - 1;
        uint32_t    pos  = (cslot * 6 + ld) * n_sc; // index of the first modulation symbol of this SC-FDMA symbol
        const cf_t* y    = q->y + ((size_t)s * 7 + l) * n_sc;
        cf_t        rotl = cexpf(I * psi * dt_slot[l]);

        if (n_sc == 1) {
          cf_t h    = q->h_sm[s] * rotl;
          cf_t hy   = conjf(h) * y[0];
          float* out = q->llr_f + pos * qm;
          if (qm == 2) {
            float c1 = 2.0f * (float)M_SQRT2 / noise;
            float l0 = -c1 * crealf(hy);
            float l1 = -c1 * cimagf(hy);
            out[0] += scr[pos * 2] ? -l0 : l0;
            out[1] += scr[pos * 2 + 1] ? -l1 : l1;
          } else {
            // symbols are +-(1 + j) / sqrt(2): a real BPSK along the pi/4 direction
            float stat = crealf(hy * cexpf(-I * (float)(M_PI / 4.0)));
            float l0   = -4.0f * stat / noise;
            out[0] += scr[pos] ? -l0 : l0;
          }
        } else {
          cf_t   zt[SRSRAN_NPUSCH_MAX_TONES];
          float  mu = 0.0f, var = 0.0f;
          cf_t   x[SRSRAN_NPUSCH_MAX_TONES];
          for (uint32_t i = 0; i < n_sc; i++) {
            cf_t  h  = q->h_sm[s * n_sc + i] * rotl;
            float h2 = crealf(h) * crealf(h) + cimagf(h) * cimagf(h);
            float den = h2 + noise;
            x[i]      = conjf(h) * y[i] / den;
            mu += h2 / den;
            var += noise * h2 / (den * den);
          }
          mu /= (float)n_sc;
          var /= (float)n_sc;
          for (uint32_t m = 0; m < n_sc; m++) {
            cf_t sum = 0.0f;
            for (uint32_t i = 0; i < n_sc; i++) {
              sum += x[i] * idft_tw[i * n_sc + m];
            }
            zt[m] = sum / sqrtf((float)n_sc);
          }
          float c1 = 2.0f * (float)M_SQRT2 * mu / var;
          for (uint32_t m = 0; m < n_sc; m++) {
            float l0 = -c1 * crealf(zt[m]);
            float l1 = -c1 * cimagf(zt[m]);
            uint32_t b = (pos + m) * 2;
            q->llr_f[b] += scr[b] ? -l0 : l0;
            q->llr_f[b + 1] += scr[b + 1] ? -l1 : l1;
          }
        }
      }
    }
  }

  // ---- 4. channel de-interleaver per resource unit (TS 36.212 5.2.2.8 without control information)
  uint32_t per_ru = data_syms_ru(c) * qm;
  uint32_t c_mux  = 6 * slots_per_ru(c);
  uint32_t rows   = data_syms_ru(c) / c_mux;
  float    mean_abs = 0.0f;
  float*   e_f      = (float*)malloc(sizeof(float) * f_bits);
  if (e_f == NULL) {
    return SRSRAN_ERROR;
  }
  for (uint32_t ru = 0; ru < c->n_ru; ru++) {
    for (uint32_t cc = 0; cc < c_mux; cc++) {
      for (uint32_t rr = 0; rr < rows; rr++) {
        for (uint32_t b = 0; b < qm; b++) {
          e_f[ru * per_ru + (rr * c_mux + cc) * qm + b] = q->llr_f[ru * per_ru + (cc * rows + rr) * qm + b];
        }
      }
    }
  }
  for (uint32_t i = 0; i < f_bits; i++) {
    mean_abs += fabsf(e_f[i]);
  }
  mean_abs /= (float)f_bits;
  float scale = mean_abs > 0.0f ? NPUSCH_LLR_TARGET / mean_abs : 1.0f;
  for (uint32_t i = 0; i < f_bits; i++) {
    q->llr_e[i] = (int16_t)lrintf(clampf(e_f[i] * scale, -NPUSCH_LLR_CLIP, NPUSCH_LLR_CLIP));
  }
  memset(q->llr_e + f_bits, 0, 64 * sizeof(int16_t));
  free(e_f);

  // ---- 5. rate de-matching and turbo decoding
  uint32_t k_cb   = c->tbs + 24;
  int      cb_idx = srsran_cbsegm_cbindex(k_cb);
  if (cb_idx < 0) {
    return SRSRAN_ERROR;
  }
  memset(q->llr_d, 0, sizeof(int16_t) * (3 * k_cb + 12 + 64));
  if (srsran_rm_turbo_rx_lut(q->llr_e, q->llr_d, f_bits, (uint32_t)cb_idx, c->rv)) {
    return SRSRAN_ERROR;
  }
  if (srsran_tdec_new_cb(&q->tdec, k_cb)) {
    return SRSRAN_ERROR;
  }
  uint32_t max_iter = c->max_iterations ? c->max_iterations : NPUSCH_DEFAULT_ITERATIONS;
  for (uint32_t it = 0; it < max_iter; it++) {
    srsran_tdec_iteration(&q->tdec, q->llr_d, q->data);
    res->nof_iterations++;
    if (srsran_crc_checksum_byte(&q->crc24a, q->data, (int)k_cb) == 0) {
      res->crc_ok = true;
      break;
    }
  }
  if (res->crc_ok) {
    srsran_bit_unpack_vector(q->data, tb, (int)c->tbs);
  }
  return SRSRAN_SUCCESS;
}
