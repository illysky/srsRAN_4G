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

#ifndef SRSRAN_ENB_NBIOT_INTERFACES_H
#define SRSRAN_ENB_NBIOT_INTERFACES_H

#include <cstdint>

namespace srsenb {

/// Logical channels of an NB-IoT UE (TS 36.331 6.7.3: SRB1bis uses LCID 3, DRBs are given one of 3..10)
constexpr uint32_t NBIOT_LCID_SRB1    = 1;
constexpr uint32_t NBIOT_LCID_SRB1BIS = 3;
constexpr uint32_t NBIOT_LCID_DRB1    = 5;

/// First C-RNTI given to NB-IoT UEs; the LTE MAC hands out 0x46 + (n mod 60000), which stays below
constexpr uint16_t NBIOT_FIRST_RNTI = 0xF000;
constexpr uint16_t NBIOT_LAST_RNTI  = 0xFFEF;
inline bool        is_nbiot_rnti(uint16_t rnti)
{
  return rnti >= NBIOT_FIRST_RNTI && rnti <= NBIOT_LAST_RNTI;
}

/// The NB-IoT RRC (in the stack) as seen by the NB-IoT MAC (in the PHY). Every method may be called from any thread.
class nbiot_rrc_interface_mac
{
public:
  virtual ~nbiot_rrc_interface_mac() = default;

  /// Msg4 went out: the UE is in RRC_CONNECTED with SRB1bis and SRB1 configured
  virtual void ue_connected(uint16_t rnti) = 0;
  /// The MAC gave up on the UE (no uplink for a long time)
  virtual void ue_lost(uint16_t rnti) = 0;
  /// An uplink MAC SDU
  virtual void write_pdu(uint16_t rnti, uint32_t lcid, const uint8_t* payload, uint32_t nof_bytes) = 0;
  /// Fills payload with at most nof_bytes of an RLC PDU; returns its length
  virtual int read_pdu(uint16_t rnti, uint32_t lcid, uint8_t* payload, uint32_t nof_bytes) = 0;
  /// Bytes waiting for transmission on lcid (RLC data, retransmissions and status)
  virtual uint32_t dl_buffer(uint16_t rnti, uint32_t lcid) = 0;
};

/// The NB-IoT MAC as seen by the NB-IoT RRC. Any thread.
class nbiot_mac_interface_rrc
{
public:
  virtual ~nbiot_mac_interface_rrc() = default;

  virtual void set_rrc(nbiot_rrc_interface_mac* rrc) = 0;
  /// Stop scheduling the UE and forget it
  virtual void release_ue(uint16_t rnti) = 0;
};

} // namespace srsenb

#endif // SRSRAN_ENB_NBIOT_INTERFACES_H
