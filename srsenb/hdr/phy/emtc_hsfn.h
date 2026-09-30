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

#ifndef SRSENB_EMTC_HSFN_H
#define SRSENB_EMTC_HSFN_H

#include <atomic>
#include <cstdint>

namespace srsenb {
namespace emtc {

/// Absolute subframe count of the subframe the PHY received last: its TTI (mod 10240) extended by the hyperframes
/// counted since start. The SIB1-BR hyperSFN and the eDRX paging windows must agree on the H-SFN and are computed in
/// different threads; the TX/RX thread is the only writer, so the count follows the air.
inline std::atomic<uint64_t>& abs_tti_latest()
{
  // starts 1024 hyperframes in, so that a TTI just before the first one received does not underflow
  static std::atomic<uint64_t> latest{10240ull * 1024};
  return latest;
}

/// The TX/RX thread, for every subframe received
inline void set_abs_tti(uint64_t abs_tti)
{
  abs_tti_latest().store(abs_tti, std::memory_order_relaxed);
}

/// Absolute subframe count of a TTI within 5.12 s of the one received last
inline uint64_t abs_tti(uint32_t tti)
{
  const uint64_t l = abs_tti_latest().load(std::memory_order_relaxed);
  const int32_t  d = (int32_t)(((tti % 10240) + 10240 - (uint32_t)(l % 10240) + 5120) % 10240) - 5120;
  return l + d;
}

/// H-SFN (TS 36.331 hyperSFN-r13, 10 bits) of a TTI within 5.12 s of now
inline uint32_t hsfn(uint32_t tti)
{
  return (uint32_t)((abs_tti(tti) / 10240) % 1024);
}

/// Hashed_ID of an S-TMSI (TS 36.304 7.3): the 32-bit FCS over b31..b0 of the S-TMSI (its M-TMSI), msb first
inline uint32_t hashed_id(uint32_t m_tmsi)
{
  uint32_t c = 0xFFFFFFFFu;
  for (int i = 31; i >= 0; i--) {
    const uint32_t bit = ((m_tmsi >> i) & 1u) ^ (c >> 31);
    c                  = (c << 1) ^ (bit ? 0x04C11DB7u : 0u);
  }
  return ~c;
}

} // namespace emtc
} // namespace srsenb

#endif // SRSENB_EMTC_HSFN_H
