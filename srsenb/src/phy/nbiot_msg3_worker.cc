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

#include "srsenb/hdr/phy/nbiot_msg3_worker.h"
#include "srsran/phy/common/phy_common.h"
#include "srsran/phy/utils/bit.h"
#include <cstring>

namespace srsenb {

namespace {
constexpr uint32_t MAX_OUTSTANDING = 8;
// A registered transmission whose start never came (the TTI counter is modulo 10240) is dropped after this
constexpr uint32_t WAIT_LIMIT_SF = 10000;
} // namespace

nbiot_msg3_worker::nbiot_msg3_worker(srslog::basic_logger& logger_) : thread("NBIOT_NPUSCH"), logger(logger_) {}

nbiot_msg3_worker::~nbiot_msg3_worker()
{
  stop();
}

int nbiot_msg3_worker::init(uint32_t lte_nof_prb, uint32_t anchor_prb, int priority, std::string& err)
{
  if (anchor_prb >= lte_nof_prb) {
    err = "NB-IoT anchor PRB is outside the LTE carrier";
    return SRSRAN_ERROR;
  }
  sf_len            = SRSRAN_SF_LEN_PRB(lte_nof_prb);
  const uint32_t fs = sf_len * 1000;
  if (fs % SRSRAN_ANCHOR_FE_OUT_RATE_HZ != 0 or fs / SRSRAN_ANCHOR_FE_OUT_RATE_HZ > SRSRAN_ANCHOR_FE_MAX_RATIO) {
    err = "the uplink sample rate is not a multiple of 1.92 MS/s; set expert.lte_sample_rates = true";
    return SRSRAN_ERROR;
  }
  // Same place as the NPRACH receiver: (2 p + 1 - N_PRB) * 90 kHz from the centre of the LTE uplink
  const int32_t offset_hz = ((int32_t)(2 * anchor_prb + 1) - (int32_t)lte_nof_prb) * 90000;
  if (srsran_anchor_fe_init(&fe, fs, offset_hz) != SRSRAN_SUCCESS) {
    err = "cannot build the uplink front end for the anchor PRB";
    return SRSRAN_ERROR;
  }
  fe_init  = true;
  fe_delay = srsran_anchor_fe_delay(&fe);
  if (fe_delay > sf_len) {
    err = "front end delay exceeds a subframe";
    return SRSRAN_ERROR;
  }
  if (srsran_npusch_init(&npusch) != SRSRAN_SUCCESS) {
    err = "cannot initialise the NPUSCH receiver";
    return SRSRAN_ERROR;
  }
  npusch_init = true;

  running = true;
  start(priority);
  initiated = true;
  return SRSRAN_SUCCESS;
}

bool nbiot_msg3_worker::expect(const nbiot_npusch_expect& e, std::string& why)
{
  if (not initiated) {
    why = "receiver not running";
    return false;
  }
  if (srsran_npusch_check_cfg(&e.cfg) != SRSRAN_SUCCESS) {
    why = "NPUSCH configuration the receiver cannot do";
    return false;
  }
  std::unique_ptr<job> j(new job);
  j->req                = e;
  const uint32_t n_out  = srsran_npusch_nof_samples(&e.cfg);
  const uint64_t need   = (uint64_t)sf_len - fe_delay + srsran_anchor_fe_nof_input(&fe, n_out);
  j->n_capture          = (uint32_t)((need + sf_len - 1) / sf_len);
  j->raw.assign((size_t)j->n_capture * sf_len, cf_t{});

  std::lock_guard<std::mutex> l(lock);
  if (waiting.size() + capturing.size() >= MAX_OUTSTANDING) {
    why = "too many transmissions outstanding";
    return false;
  }
  j->first_tti = WAIT_LIMIT_SF; // used as the remaining wait while the job is waiting
  waiting.push_back(std::move(j));
  return true;
}

int nbiot_msg3_worker::new_tti(uint32_t tti, const cf_t* rx)
{
  if (not initiated) {
    return SRSRAN_ERROR;
  }
  std::lock_guard<std::mutex> l(lock);

  // Transmissions starting in the next subframe: this one is the pre-roll of the front end
  for (auto it = waiting.begin(); it != waiting.end();) {
    job* j = it->get();
    if ((tti + 1) % 10240 == (uint32_t)(j->req.start_sf % 10240)) {
      j->first_tti = tti;
      j->n_sf      = 0;
      capturing.push_back(std::move(*it));
      it = waiting.erase(it);
    } else if (--j->first_tti == 0) {
      logger.warning("NB-IoT NPUSCH: transmission of RNTI 0x%x at subframe %llu never started, dropped",
                     j->req.rnti,
                     (unsigned long long)j->req.start_sf);
      it = waiting.erase(it);
    } else {
      ++it;
    }
  }

  for (auto it = capturing.begin(); it != capturing.end();) {
    job* j = it->get();
    if (tti != (j->first_tti + j->n_sf) % 10240) {
      logger.warning("NB-IoT NPUSCH: subframes are not contiguous (got TTI %u), dropping RNTI 0x%x", tti, j->req.rnti);
      it = capturing.erase(it);
      continue;
    }
    std::memcpy(&j->raw[(size_t)j->n_sf * sf_len], rx, sizeof(cf_t) * sf_len);
    j->n_sf++;
    if (j->n_sf == j->n_capture) {
      pending.push(it->release());
      it = capturing.erase(it);
    } else {
      ++it;
    }
  }
  return SRSRAN_SUCCESS;
}

void nbiot_msg3_worker::process(job* j)
{
  const uint32_t n_out = srsran_npusch_nof_samples(&j->req.cfg);
  decimated.assign(n_out, cf_t{});
  // The transmission starts at input sample sf_len of the capture (the first subframe is the pre-roll)
  const uint64_t s0 = sf_len;
  const uint64_t n0 = (uint64_t)j->first_tti * sf_len + s0 - fe_delay;
  if (srsran_anchor_fe_run(&fe, &j->raw[s0 - fe_delay], n0, decimated.data(), n_out) != SRSRAN_SUCCESS) {
    logger.error("NB-IoT NPUSCH: front end failed");
    return;
  }

  nbiot_npusch_result r;
  r.req = j->req;
  std::vector<uint8_t> bits(j->req.cfg.tbs);
  if (srsran_npusch_decode(&npusch, &j->req.cfg, decimated.data(), bits.data(), &r.res) != SRSRAN_SUCCESS) {
    logger.error("NB-IoT NPUSCH: receiver failed for RNTI 0x%x", j->req.rnti);
    return;
  }
  if (r.res.crc_ok) {
    r.tb.assign(j->req.cfg.tbs / 8, 0);
    uint8_t* p = bits.data();
    srsran_bit_pack_vector(p, r.tb.data(), (int)j->req.cfg.tbs);
  }
  if (callback) {
    callback(r);
  }
}

void nbiot_msg3_worker::run_thread()
{
  while (running) {
    job* j = pending.wait_pop();
    if (j == nullptr) {
      break;
    }
    if (running) {
      process(j);
    }
    delete j;
  }
}

void nbiot_msg3_worker::stop()
{
  if (initiated) {
    initiated = false;
    running   = false;
    pending.push(nullptr);
    wait_thread_finish();
  }
  if (fe_init) {
    srsran_anchor_fe_free(&fe);
    fe_init = false;
  }
  if (npusch_init) {
    srsran_npusch_free(&npusch);
    npusch_init = false;
  }
}

} // namespace srsenb
