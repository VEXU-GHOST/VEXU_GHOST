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

  unsigned char data{0};

  void update(std::shared_ptr<DeviceData> data_ptr) override
  {
    auto digital_io_data_ptr = data_ptr->as<DigitalIODeviceData>();
    data = digital_io_data_ptr->data;
  }

  std::shared_ptr<DeviceBase> clone() const override
  {
    return std::make_shared<DigitalIODeviceData>(*this);
  }

  bool operator==(const DeviceBase & rhs) const override
  {
    const DigitalIODeviceData * d_rhs = dynamic_cast<const DigitalIODeviceData *>(&rhs);
    return (d_rhs != nullptr) && (data == d_rhs->data) && (type == d_rhs->type) && (name == d_rhs->name);
  }

  std::vector<unsigned char> serialize(hardware_type_e hardware_type) const override
  {
    std::vector<unsigned char> msg;
    int msg_size = ((hardware_type == hardware_type_e::V5_BRAIN)) ? getActuatorPacketSize() : getSensorPacketSize();
    msg.resize(msg_size, 0);

    memcpy(msg.data(), &data, 1);
    checkMsgSize(msg, msg_size);

    return msg;
  }

  void deserialize(const std::vector<unsigned char> & msg, hardware_type_e hardware_type) override
  {
    int msg_size = ((hardware_type == hardware_type_e::V5_BRAIN)) ? getSensorPacketSize() : getActuatorPacketSize();
    checkMsgSize(msg, msg_size);
    memcpy(&data, msg.data(), 1);
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
           (input_mask == d_rhs->input_mask) && (output_mask == d_rhs->output_mask);
  }

  unsigned char input_mask{0};
  unsigned char output_mask{0};
};

} // namespace devices

} // namespace ghost_v5_interfaces
