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

#ifndef SRSENB_NBIOT_PRACH_WORKER_H
#define SRSENB_NBIOT_PRACH_WORKER_H

#include "srsran/common/block_queue.h"
#include "srsran/common/threads.h"
#include "srsran/srslog/srslog.h"
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

extern "C" {
#include "srsran/phy/phch/nprach.h"
#include "srsran/phy/resampling/anchor_fe.h"
}

namespace srsenb {

/// What the NB-IoT random access receiver needs to know; all of it comes from the carrier file / SIB2-NB.
struct nbiot_nprach_params {
  uint32_t            lte_nof_prb    = 0; ///< the LTE host carrier the receiver sees
  uint32_t            anchor_prb     = 0; ///< uplink anchor PRB inside it
  uint32_t            periodicity_ms = 0; ///< nprach-Periodicity
  uint32_t            start_time_ms  = 0; ///< nprach-StartTime
  srsran_nprach_cfg_t nprach         = {};
};

/// A detected preamble.
struct nbiot_nprach_detection {
  uint32_t tti;     ///< TTI in which the preamble nominally starts (radio frame start + nprach-StartTime)
  uint32_t n_init;  ///< starting subcarrier = random access preamble identifier
  float    toa;     ///< arrival time after the nominal start in samples at 1.92 MS/s, which are units of 16 Ts
  float    cfo_hz;  ///< carrier frequency offset of the UE
  float    metric;  ///< detection metric
};

/**
 * NB-IoT NPRACH receiver of the eNodeB.
 *
 * The txrx thread hands over every received subframe in order (new_tti). The worker copies the subframes of each
 * NPRACH opportunity, from one subframe before it starts to the end of the search window, and a separate thread then
 * moves the anchor PRB to DC at 1.92 MS/s (srsran_anchor_fe) and runs the preamble detector. Everything but the copy
 * happens off the txrx thread.
 *
 * The nominal preamble start is the first sample of TTI "start": the radio frame fulfilling
 * n_f mod (periodicity / 10) = 0 plus nprach-StartTime milliseconds (TS 36.211 10.1.6).
 */
class nbiot_prach_worker : srsran::thread
{
public:
  using detect_callback = std::function<void(const nbiot_nprach_detection&)>;

  explicit nbiot_prach_worker(srslog::basic_logger& logger);
  ~nbiot_prach_worker();

  /// Fails (with a message) if the parameters are not something the receiver can do.
  int init(const nbiot_nprach_params& params, int priority, std::string& err);

  /// Called from the worker thread, once per detected preamble. Set before the first subframe arrives.
  void set_callback(detect_callback cb) { callback = std::move(cb); }

  /// rx: the subframe just received on the LTE carrier (sf_len samples of RF port 0), valid only during the call.
  /// Must be called for every TTI in order.
  int new_tti(uint32_t tti, const cf_t* rx);

  void stop();

  /// True if an NPRACH opportunity starts at the beginning of this TTI (0..10239).
  static bool is_opportunity_start(uint32_t periodicity_ms, uint32_t start_time_ms, uint32_t tti);

  /// Subframes that have to be captured per opportunity, and the subframe-length in samples they consist of.
  uint32_t nof_capture_sf() const { return n_capture_sf; }

private:
  struct job {
    std::vector<cf_t> raw;
    uint32_t          start_tti = 0; // TTI of the nominal preamble start
    uint32_t          n_sf      = 0; // subframes stored
  };

  srslog::basic_logger& logger;
  nbiot_nprach_params   params  = {};
  detect_callback       callback;
  bool                  initiated = false;

  uint32_t sf_len        = 0;
  uint32_t n_out         = 0; // output samples (1.92 MS/s) handed to the detector
  uint32_t n_capture_sf  = 0;
  uint32_t fe_delay      = 0;

  srsran_anchor_fe_t fe   = {};
  srsran_nprach_t    nprach = {};
  bool               fe_init = false, nprach_init = false;
  std::vector<cf_t>  decimated;

  // pool of capture buffers: one being filled, the others waiting for / in the worker thread
  std::mutex                        pool_mutex;
  std::vector<std::unique_ptr<job>> all_jobs;
  std::vector<job*>                 free_jobs;
  srsran::block_queue<job*>         pending;
  job*                              current = nullptr;

  std::atomic<bool> running{false};
  uint32_t          skipped = 0;

  void run_thread() final;
  void process(job* j);
};

} // namespace srsenb

#endif // SRSENB_NBIOT_PRACH_WORKER_H
