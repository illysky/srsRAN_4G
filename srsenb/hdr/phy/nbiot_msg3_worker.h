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

#ifndef SRSENB_NBIOT_MSG3_WORKER_H
#define SRSENB_NBIOT_MSG3_WORKER_H

#include "srsran/common/block_queue.h"
#include "srsran/common/threads.h"
#include "srsran/srslog/srslog.h"
#include <atomic>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

extern "C" {
#include "srsran/phy/phch/npusch.h"
#include "srsran/phy/resampling/anchor_fe.h"
}

namespace srsenb {

/// An NPUSCH format 1 transmission the eNB has granted (Msg3 today) and wants received.
struct nbiot_npusch_expect {
  uint64_t            start_sf = 0; ///< absolute subframe of the first slot
  srsran_npusch_cfg_t cfg      = {}; ///< frame / slot of the first slot included
  uint16_t            rnti     = 0;
  uint32_t            preamble = 0; ///< for the log: the random access attempt this belongs to
  bool                connected = false; ///< a grant to a connected UE rather than Msg3
};

/// What came of it.
struct nbiot_npusch_result {
  nbiot_npusch_expect  req;
  srsran_npusch_res_t  res = {};
  std::vector<uint8_t> tb; ///< the transport block, packed, when res.crc_ok
};

/**
 * NB-IoT NPUSCH format 1 receiver of the eNodeB.
 *
 * expect() registers a transmission (any thread). The txrx thread hands over every received subframe in order
 * (new_tti); the subframes of a registered transmission, from one before it to the last one the front end needs, are
 * copied, and a separate thread moves the anchor PRB to DC at 1.92 MS/s (srsran_anchor_fe) and decodes it.
 */
class nbiot_msg3_worker : srsran::thread
{
public:
  using result_callback = std::function<void(const nbiot_npusch_result&)>;

  explicit nbiot_msg3_worker(srslog::basic_logger& logger);
  ~nbiot_msg3_worker();

  int  init(uint32_t lte_nof_prb, uint32_t anchor_prb, int priority, std::string& err);
  void set_callback(result_callback cb) { callback = std::move(cb); }

  /// False if the transmission cannot be received (configuration, or too many outstanding)
  bool expect(const nbiot_npusch_expect& e, std::string& why);

  /// rx: the subframe just received on the LTE carrier (sf_len samples of RF port 0). Must be called for every TTI.
  int new_tti(uint32_t tti, const cf_t* rx);

  void stop();

private:
  struct job {
    nbiot_npusch_expect req;
    std::vector<cf_t>   raw;
    uint32_t            n_capture = 0; ///< subframes to store
    uint32_t            n_sf      = 0; ///< subframes stored
    uint32_t            first_tti = 0; ///< TTI of the pre-roll subframe
  };

  srslog::basic_logger& logger;
  result_callback       callback;
  bool                  initiated = false;
  uint32_t              sf_len    = 0;
  uint32_t              fe_delay  = 0;

  srsran_anchor_fe_t fe      = {};
  srsran_npusch_t    npusch  = {};
  bool               fe_init = false, npusch_init = false;
  std::vector<cf_t>  decimated;

  std::mutex                       lock;
  std::list<std::unique_ptr<job>>  waiting;   ///< registered, capture not started
  std::list<std::unique_ptr<job>>  capturing; ///< being filled by the txrx thread
  srsran::block_queue<job*>        pending;   ///< complete, for the decoder thread
  std::atomic<bool>                running{false};

  void run_thread() final;
  void process(job* j);
};

} // namespace srsenb

#endif // SRSENB_NBIOT_MSG3_WORKER_H
