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

class DigitalInputDeviceData : public DeviceData
{
  public:
  DigitalInputDeviceData(std::string name)
  : DeviceData(name, DIGITAL_INPUT)
  {
  }

  int getActuatorPacketSize() const override
  {
    return 0;
  }
 
  int getSensorPacketSize() const override
  {
    return 1;
  }

  std::shared_ptr<DeviceBase> clone() const override
  {
    return std::make_shared<DigitalInputDeviceData>(*this);
  }

  std::vector<unsigned char> serialize(hardware_type_e hardware_type) const override
  {
    std::vector<unsigned char> msg(1);
    if (hardware_type == V5_BRAIN) {
        msg.push_back((unsigned char) value);
    }
    return msg;
  }

  void deserialize(const std::vector<unsigned char> & msg, hardware_type_e hardware_type) override
  {
    if (hardware_type == COPROCESSOR) {
        value = ((bool) msg.data()[0]);
    }
  }

  void update(std::shared_ptr<DeviceData> data_ptr) override
  {
    auto digital_data_ptr = data_ptr->as<DigitalInputDeviceData>();
    value = digital_data_ptr->value;
  }

  bool operator==(const DeviceBase & rhs) const override
  {
    const DigitalInputDeviceData * d_rhs = dynamic_cast<const DigitalInputDeviceData *>(&rhs);
    return (d_rhs != nullptr) && (name == d_rhs->name) && (type == d_rhs->type) && (value == d_rhs->value);
  }

  bool value;
};

class DigitalOutputDeviceData : public DeviceData
{
  public:
  DigitalOutputDeviceData(std::string name)
  : DeviceData(name, DIGITAL_OUTPUT)
  {
  }

  int getActuatorPacketSize() const override
  {
    return 1;
  }

  int getSensorPacketSize() const override
  {
    return 0;
  }

  std::shared_ptr<DeviceBase> clone() const override
  {
    return std::make_shared<DigitalOutputDeviceData>(*this);
  }

  std::vector<unsigned char> serialize(hardware_type_e hardware_type) const override
  {
    std::vector<unsigned char> msg;
    if (hardware_type == COPROCESSOR) {
        msg.push_back((unsigned char) value);
    }
    return msg;
  }

  void deserialize(const std::vector<unsigned char> & msg, hardware_type_e hardware_type) override
  {
    if (hardware_type == V5_BRAIN) {
        value = ((bool) msg.data()[0]);
    }
  }

  void update(std::shared_ptr<DeviceData> data_ptr) override
  {
    auto digital_data_ptr = data_ptr->as<DigitalOutputDeviceData>();
    value = digital_data_ptr->value;
  }

  bool operator==(const DeviceBase & rhs) const override
  {
    const DigitalOutputDeviceData * d_rhs = dynamic_cast<const DigitalOutputDeviceData *>(&rhs);
    return (d_rhs != nullptr) && (name == d_rhs->name) && (type == d_rhs->type) && (value == d_rhs->value);
  }

  bool value;
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
};

} // namespace devices

} // namespace ghost_v5_interfaces
