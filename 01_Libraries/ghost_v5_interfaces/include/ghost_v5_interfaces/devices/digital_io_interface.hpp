/*
 *   Copyright (c) 2024 Maxx Wilson, Xander Wilson
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

using ghost_util::packByte;
using ghost_util::unpackByte;

namespace ghost_v5_interfaces
{

namespace devices
{

class DigitalIODeviceData : public DeviceData
{
public:
  DigitalIODeviceData(std::string name)
  : DeviceData(name, device_type_e::DIGITAL_IO)
  {
  }

  int getActuatorPacketSize() const override
  {
    return 1;
  }

  int getSensorPacketSize() const override
  {
    return 1;
  }

  // Digital IO State
  std::vector<bool> ports;
  std::vector<bool> is_actuator;

  void update(std::shared_ptr<DeviceData> data_ptr) override
  {
    auto digital_data_ptr = data_ptr->as<DigitalIODeviceData>();
    ports = digital_data_ptr->ports;
  }

  std::shared_ptr<DeviceBase> clone() const override
  {
    return std::make_shared<DigitalIODeviceData>(*this);
  }

  bool operator==(const DeviceBase & rhs) const override
  {
    const DigitalIODeviceData * d_rhs = dynamic_cast<const DigitalIODeviceData *>(&rhs);
    if (d_rhs == nullptr || ports.size() != d_rhs->ports.size()) return false;
    for (int i = 0; i < ports.size(); i++) {
        if (ports[i] != d_rhs->ports[i]) return false;
        if (is_actuator[i] != d_rhs->is_actuator[i]) return false;
    }
    return true;
  }

  std::vector<unsigned char> serialize(hardware_type_e hardware_type) const override
  {
    std::vector<unsigned char> msg(getActuatorPacketSize());
    unsigned char byte_pack = packByte(ports);
    unsigned char write_mask;

    //TODO(xander): change packByte(is_actuator) and ~packByte(is_actuator) into field variables
    if (hardware_type == hardware_type_e::V5_BRAIN) {
      write_mask = ~packByte(is_actuator);
    }
    else if (hardware_type == hardware_type_e::COPROCESSOR) {
      write_mask = packByte(is_actuator);
    }

    *(msg.data()) = byte_pack & write_mask;

    checkMsgSize(msg, getActuatorPacketSize());
    return msg;
  }

  void deserialize(const std::vector<unsigned char> & msg, hardware_type_e hardware_type) override
  {
    checkMsgSize(msg, getActuatorPacketSize());
    auto msg_data = msg.data();
    unsigned char byte_pack;
    unsigned char read_mask;

    //TODO(xander): change packByte(is_actuator) and ~packByte(is_actuator) into field variables
    if (hardware_type == hardware_type_e::V5_BRAIN) {
        read_mask = packByte(is_actuator);
    }
    else if (hardware_type == hardware_type_e::COPROCESSOR) {
        read_mask = ~packByte(is_actuator);
    }

    memcpy(&byte_pack, msg_data, 1);
    auto byte_vector = unpackByte(byte_pack & read_mask);
    
    for (int i = 0; i < byte_vector.size(); i++) {
        ports[i] = ports[i] || byte_vector[i];
    }
  }
};

class DigitalIODeviceConfig : public DeviceConfig
{
public:
  std::shared_ptr<DeviceBase> clone() const override
  {
    return std::make_shared<DigitalIODeviceConfig>(*this);
  }

  bool operator==(const DeviceBase & rhs) const override
  {
    const DigitalIODeviceConfig * d_rhs = dynamic_cast<const DigitalIODeviceConfig *>(&rhs);
    return (d_rhs != nullptr) && (port == d_rhs->port) && (name == d_rhs->name) &&
           (type == d_rhs->type);
  }
};

} // namespace devices

} // namespace ghost_v5_interfaces
