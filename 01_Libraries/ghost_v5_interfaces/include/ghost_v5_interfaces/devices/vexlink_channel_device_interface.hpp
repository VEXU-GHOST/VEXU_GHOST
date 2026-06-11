/*
 *   Copyright (c) 2024 Maxx Wilson
 *   All rights reserved.

 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

#pragma once

#include <cstring>
#include "ghost_util/byte_utils.hpp"
#include "ghost_v5_interfaces/devices/device_interfaces.hpp"

namespace ghost_v5_interfaces
{

namespace devices
{

// Carries the partner robot's pose received over VEXLink (V5→Jetson, sensor direction)
// and this robot's pose to broadcast over VEXLink (Jetson→V5, actuator direction).
class VexlinkChannelDeviceData : public DeviceData
{
public:
  VexlinkChannelDeviceData(std::string name)
  : DeviceData(name, device_type_e::VEXLINK_CHANNEL)
  {
  }

  // Actuator packet: 3 floats (tx_x, tx_y, tx_theta) — Jetson→V5 → radio TX
  int getActuatorPacketSize() const override { return 12; }

  // Sensor packet: 3 floats + 1 uint8 (rx_x, rx_y, rx_theta, rx_valid) — radio RX → V5→Jetson
  int getSensorPacketSize() const override { return 13; }

  // TX: this robot's pose to broadcast (written by Jetson auton, sent via VEXLink by V5)
  float tx_x = 0.0f;
  float tx_y = 0.0f;
  float tx_theta = 0.0f;

  // RX: partner robot's pose received over VEXLink (filled by V5, read by Jetson)
  float rx_x = 0.0f;
  float rx_y = 0.0f;
  float rx_theta = 0.0f;
  bool rx_valid = false;

  void update(std::shared_ptr<DeviceData> data_ptr) override
  {
    auto p = data_ptr->as<VexlinkChannelDeviceData>();
    tx_x = p->tx_x;
    tx_y = p->tx_y;
    tx_theta = p->tx_theta;
    rx_x = p->rx_x;
    rx_y = p->rx_y;
    rx_theta = p->rx_theta;
    rx_valid = p->rx_valid;
  }

  std::shared_ptr<DeviceBase> clone() const override
  {
    return std::make_shared<VexlinkChannelDeviceData>(*this);
  }

  bool operator==(const DeviceBase & rhs) const override
  {
    const VexlinkChannelDeviceData * d = dynamic_cast<const VexlinkChannelDeviceData *>(&rhs);
    return (d != nullptr) && (name == d->name) && (type == d->type) &&
           (tx_x == d->tx_x) && (tx_y == d->tx_y) && (tx_theta == d->tx_theta) &&
           (rx_x == d->rx_x) && (rx_y == d->rx_y) && (rx_theta == d->rx_theta) &&
           (rx_valid == d->rx_valid);
  }

  std::vector<unsigned char> serialize(hardware_type_e hardware_type) const override
  {
    std::vector<unsigned char> msg;
    if (hardware_type == hardware_type_e::V5_BRAIN) {
      // V5 Brain serializes sensor data: the RX pose received from partner radio
      msg.resize(getSensorPacketSize(), 0);
      auto buf = msg.data();
      int off = 0;
      memcpy(buf + off, &rx_x, 4); off += 4;
      memcpy(buf + off, &rx_y, 4); off += 4;
      memcpy(buf + off, &rx_theta, 4); off += 4;
      uint8_t valid_byte = rx_valid ? 1 : 0;
      memcpy(buf + off, &valid_byte, 1);
    } else {
      // Coprocessor serializes actuator data: the TX pose to broadcast over radio
      msg.resize(getActuatorPacketSize(), 0);
      auto buf = msg.data();
      int off = 0;
      memcpy(buf + off, &tx_x, 4); off += 4;
      memcpy(buf + off, &tx_y, 4); off += 4;
      memcpy(buf + off, &tx_theta, 4);
    }
    int expected = (hardware_type == hardware_type_e::V5_BRAIN) ?
      getSensorPacketSize() : getActuatorPacketSize();
    checkMsgSize(msg, expected);
    return msg;
  }

  void deserialize(const std::vector<unsigned char> & msg, hardware_type_e hardware_type) override
  {
    if (hardware_type == hardware_type_e::COPROCESSOR) {
      // Coprocessor deserializes sensor data: the RX pose from the V5
      checkMsgSize(msg, getSensorPacketSize());
      auto buf = msg.data();
      int off = 0;
      memcpy(&rx_x, buf + off, 4); off += 4;
      memcpy(&rx_y, buf + off, 4); off += 4;
      memcpy(&rx_theta, buf + off, 4); off += 4;
      uint8_t valid_byte = 0;
      memcpy(&valid_byte, buf + off, 1);
      rx_valid = (valid_byte != 0);
    } else {
      // V5 Brain deserializes actuator data: the TX pose from the Jetson
      checkMsgSize(msg, getActuatorPacketSize());
      auto buf = msg.data();
      int off = 0;
      memcpy(&tx_x, buf + off, 4); off += 4;
      memcpy(&tx_y, buf + off, 4); off += 4;
      memcpy(&tx_theta, buf + off, 4);
    }
  }
};

class VexlinkChannelDeviceConfig : public DeviceConfig
{
public:
  std::shared_ptr<DeviceBase> clone() const override
  {
    return std::make_shared<VexlinkChannelDeviceConfig>(*this);
  }

  bool operator==(const DeviceBase & rhs) const override
  {
    const VexlinkChannelDeviceConfig * d =
      dynamic_cast<const VexlinkChannelDeviceConfig *>(&rhs);
    return (d != nullptr) && (port == d->port) && (name == d->name) && (type == d->type);
  }
};

} // namespace devices

} // namespace ghost_v5_interfaces
