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

#include "srsenb/hdr/phy/nbiot_prach_worker.h"
#include "srsran/phy/common/phy_common.h"
#include <algorithm>
#include <cstring>

namespace srsenb {

namespace {
// Samples (1.92 MS/s) handed to the detector beyond the preamble itself: the detector searches arrival times up to
// the cyclic prefix, which moves its last analysis windows past the nominal end (nprach_frontend_test uses the same).
constexpr uint32_t NPRACH_SEARCH_TAIL = 600;

// Capture buffers: the one being filled, and the ones queued for or being processed by the detector thread.
constexpr uint32_t NOF_JOBS = 3;
} // namespace

bool nbiot_prach_worker::is_opportunity_start(uint32_t periodicity_ms, uint32_t start_time_ms, uint32_t tti)
{
  if (periodicity_ms < 10 or periodicity_ms % 10 != 0) {
    return false;
  }
  // Opportunities start start_time_ms after the first subframe of each radio frame n_f with n_f mod (P / 10) == 0.
  // The hyper frame is 10240 ms, so a start time always falls inside it and n_f = (tti - start) / 10.
  const uint32_t d = (tti + 10240 - (start_time_ms % 10240)) % 10240;
  return d % 10 == 0 and (d / 10) % (periodicity_ms / 10) == 0;
}

nbiot_prach_worker::nbiot_prach_worker(srslog::basic_logger& logger_) : thread("NBIOT_PRACH"), logger(logger_) {}

nbiot_prach_worker::~nbiot_prach_worker()
{
  stop();
}

int nbiot_prach_worker::init(const nbiot_nprach_params& p, int priority, std::string& err)
{
  params = p;

  if (srsran_nprach_check_cfg(&params.nprach) != SRSRAN_SUCCESS) {
    err = "NPRACH configuration is not valid (format, repetitions, subcarriers or offset)";
    return SRSRAN_ERROR;
  }
  if (params.periodicity_ms < 10 or params.periodicity_ms % 10 != 0 or params.start_time_ms >= 10240) {
    err = "nprach-Periodicity must be a multiple of 10 ms and nprach-StartTime below 10240 ms";
    return SRSRAN_ERROR;
  }
  if (params.anchor_prb >= params.lte_nof_prb) {
    err = "NB-IoT anchor PRB is outside the LTE carrier";
    return SRSRAN_ERROR;
  }

  sf_len = SRSRAN_SF_LEN_PRB(params.lte_nof_prb);
  const uint32_t fs = sf_len * 1000;
  if (fs % SRSRAN_ANCHOR_FE_OUT_RATE_HZ != 0 or fs / SRSRAN_ANCHOR_FE_OUT_RATE_HZ > SRSRAN_ANCHOR_FE_MAX_RATIO) {
    err = "the uplink sample rate of the eNB (" + std::to_string(fs) + " Hz) is not a multiple of 1.92 MS/s; set "
          "expert.lte_sample_rates = true";
    return SRSRAN_ERROR;
  }

  // The anchor carrier sits (2 p + 1 - N_PRB) * 90 kHz from the centre of the LTE uplink
  const int32_t offset_hz = ((int32_t)(2 * params.anchor_prb + 1) - (int32_t)params.lte_nof_prb) * 90000;
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

  if (srsran_nprach_init(&nprach, &params.nprach) != SRSRAN_SUCCESS) {
    err = "cannot initialise the NPRACH detector";
    return SRSRAN_ERROR;
  }
  nprach_init = true;

  n_out = srsran_nprach_nof_samples(&params.nprach) + NPRACH_SEARCH_TAIL;
  decimated.assign(n_out, cf_t{});

  // One subframe before the opportunity (the front end reads its group delay before the first output sample), then
  // enough to supply every input sample of the last output sample.
  const uint64_t need = (uint64_t)sf_len - fe_delay + srsran_anchor_fe_nof_input(&fe, n_out);
  n_capture_sf        = (uint32_t)((need + sf_len - 1) / sf_len);

  if (n_capture_sf + 1 > params.periodicity_ms) {
    err = "the NPRACH preamble (" + std::to_string(n_capture_sf) + " ms) does not fit in nprach-Periodicity";
    return SRSRAN_ERROR;
  }

  for (uint32_t i = 0; i < NOF_JOBS; i++) {
    all_jobs.emplace_back(new job);
    all_jobs.back()->raw.assign((size_t)n_capture_sf * sf_len, cf_t{});
    free_jobs.push_back(all_jobs.back().get());
  }

  running = true;
  start(priority);
  initiated = true;
  return SRSRAN_SUCCESS;
}

int nbiot_prach_worker::new_tti(uint32_t tti, const cf_t* rx)
{
  if (not initiated) {
    return SRSRAN_ERROR;
  }

  if (current == nullptr) {
    if (not is_opportunity_start(params.periodicity_ms, params.start_time_ms, (tti + 1) % 10240)) {
      return SRSRAN_SUCCESS;
    }
    // The next subframe is the nominal start: this one is the pre-roll for the filter
    {
      std::lock_guard<std::mutex> lock(pool_mutex);
      if (not free_jobs.empty()) {
        current = free_jobs.back();
        free_jobs.pop_back();
      }
    }
    if (current == nullptr) {
      skipped++;
      logger.warning("NB-IoT NPRACH: detector is behind, skipping the opportunity at TTI %u (%u skipped)",
                     (tti + 1) % 10240,
                     skipped);
      return SRSRAN_SUCCESS;
    }
    current->start_tti = (tti + 1) % 10240;
    current->n_sf      = 0;
  } else if (tti != (current->start_tti + 10240 - 1 + current->n_sf) % 10240) {
    // a subframe went missing: what was captured is misaligned, drop it
    logger.warning("NB-IoT NPRACH: subframes are not contiguous (got TTI %u), dropping the capture", tti);
    {
      std::lock_guard<std::mutex> lock(pool_mutex);
      free_jobs.push_back(current);
    }
    current = nullptr;
    return SRSRAN_SUCCESS;
  }

  std::memcpy(&current->raw[(size_t)current->n_sf * sf_len], rx, sizeof(cf_t) * sf_len);
  current->n_sf++;

  if (current->n_sf == n_capture_sf) {
    pending.push(current);
    current = nullptr;
  }
  return SRSRAN_SUCCESS;
}

void nbiot_prach_worker::process(job* j)
{
  // The output stream starts at the nominal preamble start, which is input sample sf_len of the capture
  const uint64_t s0 = sf_len;
  const uint64_t n0 = (uint64_t)((j->start_tti + 10240 - 1) % 10240) * sf_len + s0 - fe_delay;
  if (srsran_anchor_fe_run(&fe, &j->raw[s0 - fe_delay], n0, decimated.data(), n_out) != SRSRAN_SUCCESS) {
    logger.error("NB-IoT NPRACH: front end failed");
    return;
  }

  srsran_nprach_det_t det[SRSRAN_NPRACH_MAX_SC] = {};
  int                 n                          = srsran_nprach_detect(&nprach, decimated.data(), det);
  if (n < 0) {
    logger.error("NB-IoT NPRACH: detector failed");
    return;
  }
  for (uint32_t i = 0; i < params.nprach.n_sc_nprach; i++) {
    if (not det[i].detected) {
      continue;
    }
    nbiot_nprach_detection d = {};
    d.tti                    = j->start_tti;
    d.n_init                 = det[i].n_init;
    d.toa                    = det[i].toa;
    d.cfo_hz                 = det[i].cfo_hz;
    d.metric                 = det[i].metric;
    logger.info("NB-IoT NPRACH: TTI %u preamble %u toa %.2f samples cfo %.0f Hz metric %.1f",
                d.tti,
                d.n_init,
                d.toa,
                d.cfo_hz,
                d.metric);
    if (callback) {
      callback(d);
    }
  }
}

void nbiot_prach_worker::run_thread()
{
  while (running) {
    job* j = pending.wait_pop();
    if (j == nullptr) {
      break;
    }
    if (running) {
      process(j);
    }
    std::lock_guard<std::mutex> lock(pool_mutex);
    free_jobs.push_back(j);
  }
}

void nbiot_prach_worker::stop()
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
  if (nprach_init) {
    srsran_nprach_free(&nprach);
    nprach_init = false;
  }
}

} // namespace srsenb
