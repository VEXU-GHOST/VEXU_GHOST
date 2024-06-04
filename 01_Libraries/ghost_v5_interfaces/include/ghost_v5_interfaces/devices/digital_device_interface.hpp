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
#include "ghost_v5_interfaces/devices/device_interfaces.hpp"

namespace ghost_v5_interfaces
{

namespace devices
{

class DigitalDeviceData : public DeviceData
{
public:
  struct SerialConfig
  {
    SerialConfig()
    {
    }

    bool operator== (const SerialConfig &rhs) const {
        return io_type == rhs.io_type;
    }

    digital_io_type_e io_type;
  };

  DigitalDeviceData(std::string name, SerialConfig serial_config = SerialConfig())
  : DeviceData(name, device_type_e::DIGITAL)
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

  bool value;

  void update(std::shared_ptr<DeviceData> data_ptr) override
  {
    auto digital_data_ptr = data_ptr->as<DigitalDeviceData>();
    value = digital_data_ptr->value;
  }

  std::shared_ptr<DeviceBase> clone() const override
  {
    return std::make_shared<DigitalDeviceData>(*this);
  }

  bool operator==(const DeviceBase & rhs) const override
  {
    const DigitalDeviceData * d_rhs = dynamic_cast<const DigitalDeviceData *>(&rhs);
    return (d_rhs != nullptr && value == d_rhs->value);
  }

  std::vector<unsigned char> serialize(hardware_type_e hardware_type) const override
  {
    std::vector<unsigned char> msg(1);
    bool valid = hardware_type == V5_BRAIN && serial_config_.io_type == SENSOR
              || hardware_type == COPROCESSOR && serial_config_.io_type == ACTUATOR;
    if (valid) {
        msg.push_back((unsigned char) value);
    }
    return msg;
  }

  void deserialize(const std::vector<unsigned char> & msg, hardware_type_e hardware_type) override
  {
    bool valid = hardware_type == V5_BRAIN && serial_config_.io_type == SENSOR
              || hardware_type == COPROCESSOR && serial_config_.io_type == ACTUATOR;
    if (!valid) {
        return;
    }
    value = ((bool) msg.data()[0]);
  }

  SerialConfig serial_config_;
};

class DigitalDeviceConfig : public DeviceConfig
{
public:
  std::shared_ptr<DeviceBase> clone() const override
  {
    return std::make_shared<DigitalDeviceConfig>(*this);
  }

  bool operator==(const DeviceBase & rhs) const override
  {
    const DigitalDeviceConfig * d_rhs = dynamic_cast<const DigitalDeviceConfig *>(&rhs);
    return (d_rhs != nullptr) && (port == d_rhs->port) && (name == d_rhs->name) &&
           (type == d_rhs->type);
  }

  DigitalDeviceData::SerialConfig serial_config;
};

} // namespace devices

} // namespace ghost_v5_interfaces
