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

#include "srsran/phy/phch/nbiot_dl_sched.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------------------------------------ valid subframes */

bool srsran_nbiot_layout_is_dl_sf(const srsran_nbiot_layout_t* l, uint64_t t)
{
  const uint32_t hfn = (uint32_t)(t / SRSRAN_NBIOT_SF_PER_HFN);
  const uint32_t rem = (uint32_t)(t % SRSRAN_NBIOT_SF_PER_HFN);
  const uint32_t sfn = rem / 10;
  const uint32_t sf  = rem % 10;

  if (sf == 0 || sf == 5 || (sf == 9 && (sfn % 2) == 0)) { // NPBCH, NPSS, NSSS
    return false;
  }
  srsran_nbiot_bcch_pos_t pos;
  if (l->sib1_set && srsran_nbiot_sib1_locate(l->cell_id, &l->mib, sfn, sf, &pos)) {
    return false;
  }
  for (uint32_t i = 0; i < SRSRAN_NBIOT_MAX_SI; i++) {
    if (l->si_set[i] && srsran_nbiot_si_locate(l->cell_id, &l->mib, &l->si[i], hfn, sfn, sf, &pos)) {
      return false;
    }
  }
  return true;
}

uint64_t srsran_nbiot_layout_next_dl_sf(const srsran_nbiot_layout_t* l, uint64_t t)
{
  // a frame always has valid subframes; the loop is bounded by an SI window (at most 2560 ms) to be safe
  for (uint32_t i = 0; i < 4 * SRSRAN_NBIOT_SF_PER_HFN; i++) {
    if (srsran_nbiot_layout_is_dl_sf(l, t + i)) {
      return t + i;
    }
  }
  return t;
}

/* ----------------------------------------------------------------------------------------------------- search space */

int srsran_nbiot_dci_rep_for_rmax(uint32_t r_max)
{
  switch (r_max) {
    case 1:
      return 0;
    case 2:
      return 1;
    case 4:
      return 2;
    case 8:
    case 16:
    case 32:
    case 64:
    case 128:
    case 256:
    case 512:
    case 1024:
    case 2048:
      return 3; // Rmax >= 8: R = Rmax / 8, / 4, / 2, Rmax are 00, 01, 10, 11
    default:
      return -1;
  }
}

uint64_t srsran_nbiot_search_space_start(uint32_t  r_max,
                                         uint32_t  g_halves,
                                         uint32_t  offset_eighths,
                                         uint64_t  t_min,
                                         uint32_t* period_sf)
{
  if (period_sf) {
    *period_sf = 0;
  }
  if (r_max == 0 || g_halves == 0 || offset_eighths > 3 || ((uint64_t)r_max * g_halves) % 2 != 0) {
    return 0;
  }
  const uint32_t T = r_max * g_halves / 2;
  if (T < 4) {
    return 0;
  }
  const uint32_t want = (offset_eighths * T) / 8; // floor(alpha_offset * T)
  if (period_sf) {
    *period_sf = T;
  }
  // 10 n_f + floor(n_s / 2) is the subframe count inside the H-SFN, n_f being the SFN (0..1023)
  for (uint64_t t = t_min; t < t_min + SRSRAN_NBIOT_SF_PER_HFN + T; t++) {
    if ((t % SRSRAN_NBIOT_SF_PER_HFN) % T == want) {
      return t;
    }
  }
  return 0;
}

/* ------------------------------------------------------------------------------------------------------- plans */

int srsran_nbiot_plan_npdcch(const srsran_nbiot_layout_t* l, uint64_t k0, uint32_t r, srsran_nbiot_plan_t* plan)
{
  if (l == NULL || plan == NULL || r == 0 || r > SRSRAN_NBIOT_PLAN_MAX_SF) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  memset(plan, 0, sizeof(*plan));
  uint64_t t = srsran_nbiot_layout_next_dl_sf(l, k0);
  for (uint32_t i = 0; i < r; i++) {
    plan->t[i] = t;
    t          = srsran_nbiot_layout_next_dl_sf(l, t + 1);
  }
  for (uint32_t i = 0; i < r; i++) {
    plan->pos_in_group[i] = i % 4;
    plan->reinit_sf[i]    = (uint8_t)((plan->t[i - i % 4] % SRSRAN_NBIOT_SF_PER_HFN) % 10);
  }
  plan->nof_sf = r;
  return SRSRAN_SUCCESS;
}

int srsran_nbiot_plan_npdsch(const srsran_nbiot_layout_t* l,
                             uint64_t                     n_last,
                             uint32_t                     k0,
                             uint32_t                     n_sf,
                             uint32_t                     n_rep,
                             srsran_nbiot_plan_t*         plan)
{
  if (l == NULL || plan == NULL || n_sf == 0 || n_sf > SRSRAN_NBIOT_DLCH_MAX_NSF || n_rep == 0 ||
      (uint64_t)n_sf * n_rep > SRSRAN_NBIOT_PLAN_MAX_SF) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  memset(plan, 0, sizeof(*plan));

  uint64_t t = srsran_nbiot_layout_next_dl_sf(l, n_last + 5);
  for (uint32_t i = 0; i < k0; i++) {
    t = srsran_nbiot_layout_next_dl_sf(l, t + 1);
  }
  const uint32_t n = n_sf * n_rep;
  uint64_t       times[SRSRAN_NBIOT_PLAN_MAX_SF];
  for (uint32_t i = 0; i < n; i++) {
    times[i] = t;
    t        = srsran_nbiot_layout_next_dl_sf(l, t + 1);
  }

  const uint32_t m      = n_rep < 4 ? n_rep : 4;
  const uint32_t passes = n_rep / m;
  uint32_t       idx    = 0;
  for (uint32_t p = 0; p < passes; p++) {
    const uint64_t start = times[p * n_sf * m];
    for (uint32_t cw = 0; cw < n_sf; cw++) {
      for (uint32_t r = 0; r < m; r++, idx++) {
        plan->t[idx]        = times[idx];
        plan->sf_in_cw[idx] = cw;
        plan->pass_sf[idx]  = (uint8_t)((start % SRSRAN_NBIOT_SF_PER_HFN) % 10);
        plan->pass_sfn[idx] = (uint16_t)((start % SRSRAN_NBIOT_SF_PER_HFN) / 10);
      }
    }
  }
  plan->nof_sf = n;
  return SRSRAN_SUCCESS;
}

/* ---------------------------------------------------------------------------------------------------------- table */

typedef struct {
  uint64_t t;    // subframe this slot is for; valid while t >= now
  bool     used;
  uint8_t  obj;
  uint8_t  reinit_sf, pos_in_group, nof_sf, sf_in_cw, pass_sf;
  uint16_t pass_sfn;
} slot_t;

typedef struct {
  bool                   used;
  uint64_t               last_t;
  srsran_nbiot_tx_kind_t kind;
  uint16_t               rnti;
  uint32_t               e_len;
  uint8_t                e[SRSRAN_NBIOT_DL_SCHED_MAX_E];
} object_t;

struct srsran_nbiot_dl_sched_st {
  pthread_mutex_t lock;
  uint64_t        now;
  bool                  layout_set;
  srsran_nbiot_layout_t layout;
  slot_t          slot[SRSRAN_NBIOT_DL_SCHED_SLOTS];
  object_t        obj[SRSRAN_NBIOT_DL_SCHED_OBJECTS];
};

srsran_nbiot_dl_sched_t* srsran_nbiot_dl_sched_new(void)
{
  srsran_nbiot_dl_sched_t* s = calloc(1, sizeof(*s));
  if (s) {
    pthread_mutex_init(&s->lock, NULL);
  }
  return s;
}

void srsran_nbiot_dl_sched_free(srsran_nbiot_dl_sched_t* s)
{
  if (s) {
    pthread_mutex_destroy(&s->lock);
    free(s);
  }
}

static bool slot_free_for(const srsran_nbiot_dl_sched_t* s, uint64_t t)
{
  const slot_t* sl = &s->slot[t % SRSRAN_NBIOT_DL_SCHED_SLOTS];
  // reusable if it never held anything, or held a subframe that is over
  return !sl->used || sl->t < s->now;
}

static int add(srsran_nbiot_dl_sched_t*   s,
               srsran_nbiot_tx_kind_t     kind,
               const uint8_t*             e,
               uint32_t                   e_len,
               uint16_t                   rnti,
               uint32_t                   n_sf,
               const srsran_nbiot_plan_t* plan,
               uint64_t                   min_t)
{
  if (s == NULL || e == NULL || plan == NULL || plan->nof_sf == 0 || e_len == 0 || e_len > SRSRAN_NBIOT_DL_SCHED_MAX_E) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  int ret = SRSRAN_ERROR;
  pthread_mutex_lock(&s->lock);

  uint64_t last = 0;
  bool     ok   = true;
  for (uint32_t i = 0; i < plan->nof_sf && ok; i++) {
    const uint64_t t = plan->t[i];
    ok &= t >= min_t && t >= s->now && t < s->now + SRSRAN_NBIOT_DL_SCHED_SLOTS && slot_free_for(s, t);
    // the same subframe twice in one plan would also be a conflict
    for (uint32_t j = 0; j < i && ok; j++) {
      ok &= plan->t[j] != t;
    }
    if (t > last) {
      last = t;
    }
  }
  int o = -1;
  for (int i = 0; i < SRSRAN_NBIOT_DL_SCHED_OBJECTS && ok; i++) {
    if (!s->obj[i].used || s->obj[i].last_t < s->now) {
      o = i;
      break;
    }
  }
  if (ok && o >= 0) {
    object_t* ob = &s->obj[o];
    ob->used     = true;
    ob->last_t   = last;
    ob->kind     = kind;
    ob->rnti     = rnti;
    ob->e_len    = e_len;
    memcpy(ob->e, e, e_len);
    for (uint32_t i = 0; i < plan->nof_sf; i++) {
      slot_t* sl       = &s->slot[plan->t[i] % SRSRAN_NBIOT_DL_SCHED_SLOTS];
      sl->used         = true;
      sl->t            = plan->t[i];
      sl->obj          = o;
      sl->reinit_sf    = plan->reinit_sf[i];
      sl->pos_in_group = plan->pos_in_group[i];
      sl->nof_sf       = n_sf;
      sl->sf_in_cw     = plan->sf_in_cw[i];
      sl->pass_sf      = plan->pass_sf[i];
      sl->pass_sfn     = plan->pass_sfn[i];
    }
    ret = SRSRAN_SUCCESS;
  }
  pthread_mutex_unlock(&s->lock);
  return ret;
}

int srsran_nbiot_dl_sched_add_npdcch(srsran_nbiot_dl_sched_t*   s,
                                     const uint8_t*             e,
                                     uint32_t                   e_len,
                                     const srsran_nbiot_plan_t* plan,
                                     uint64_t                   min_t)
{
  return add(s, SRSRAN_NBIOT_TX_NPDCCH, e, e_len, 0, 0, plan, min_t);
}

int srsran_nbiot_dl_sched_add_npdsch(srsran_nbiot_dl_sched_t*   s,
                                     const uint8_t*             e,
                                     uint32_t                   e_len,
                                     uint16_t                   rnti,
                                     uint32_t                   n_sf,
                                     const srsran_nbiot_plan_t* plan,
                                     uint64_t                   min_t)
{
  return add(s, SRSRAN_NBIOT_TX_NPDSCH, e, e_len, rnti, n_sf, plan, min_t);
}

bool srsran_nbiot_dl_sched_get(srsran_nbiot_dl_sched_t* s, uint64_t t, srsran_nbiot_dl_tx_t* tx)
{
  bool found = false;
  pthread_mutex_lock(&s->lock);
  if (t > s->now) {
    s->now = t;
  }
  const slot_t* sl = &s->slot[t % SRSRAN_NBIOT_DL_SCHED_SLOTS];
  if (sl->used && sl->t == t) {
    const object_t* ob = &s->obj[sl->obj];
    tx->kind           = ob->kind;
    tx->reinit_sf      = sl->reinit_sf;
    tx->pos_in_group   = sl->pos_in_group;
    tx->rnti           = ob->rnti;
    tx->pass_sfn       = sl->pass_sfn;
    tx->nof_sf         = sl->nof_sf;
    tx->sf_in_cw       = sl->sf_in_cw;
    tx->pass_sf        = sl->pass_sf;
    tx->e_len          = ob->e_len;
    memcpy(tx->e, ob->e, ob->e_len);
    found = true;
  }
  pthread_mutex_unlock(&s->lock);
  return found;
}

uint64_t srsran_nbiot_dl_sched_now(srsran_nbiot_dl_sched_t* s)
{
  pthread_mutex_lock(&s->lock);
  uint64_t n = s->now;
  pthread_mutex_unlock(&s->lock);
  return n;
}

void srsran_nbiot_dl_sched_set_layout(srsran_nbiot_dl_sched_t* s, const srsran_nbiot_layout_t* l)
{
  pthread_mutex_lock(&s->lock);
  s->layout     = *l;
  s->layout_set = true;
  pthread_mutex_unlock(&s->lock);
}

bool srsran_nbiot_dl_sched_get_layout(srsran_nbiot_dl_sched_t* s, srsran_nbiot_layout_t* l)
{
  pthread_mutex_lock(&s->lock);
  const bool set = s->layout_set;
  if (set) {
    *l = s->layout;
  }
  pthread_mutex_unlock(&s->lock);
  return set;
}

bool srsran_nbiot_dl_sched_busy(srsran_nbiot_dl_sched_t* s, uint64_t t)
{
  pthread_mutex_lock(&s->lock);
  const slot_t* sl   = &s->slot[t % SRSRAN_NBIOT_DL_SCHED_SLOTS];
  bool          busy = sl->used && sl->t == t;
  pthread_mutex_unlock(&s->lock);
  return busy;
}
