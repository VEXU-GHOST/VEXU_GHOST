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

#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace ghost_v5_interfaces
{

namespace devices
{

// List available V5 Devices
enum device_type_e
{
  MOTOR,
  ROTATION_SENSOR,
  INERTIAL_SENSOR,
  DISTANCE_SENSOR,       // Unsupported
  OPTICAL_SENSOR,        // Unsupported
  VISION_SENSOR,         // Unsupported
  GPS_SENSOR,            // Unsupported
  RADIO,                 // Unsupported
  JOYSTICK,
  DIGITAL_SENSOR,
  DIGITAL_ACTUATOR,
  INVALID
};

enum hardware_type_e
{
  COPROCESSOR,
  V5_BRAIN
};

class DeviceBase : public std::enable_shared_from_this<DeviceBase>
{
public:
  std::string name = "";
  device_type_e type = device_type_e::INVALID;

  /**
   * @brief Returns a pointer to a deep copy of this object
   *
   * @return std::shared_ptr<DeviceBase>
   */
  virtual std::shared_ptr<DeviceBase> clone() const = 0;

  /**
   * @brief Helper to convert base class pointer to derived class pointer.
   * DeviceBase is pure virtual, so a DeviceBase pointer is always actually pointing to a Derived class.
   *
   * @return std::shared_ptr<Derived> pointer to this object as its derived class
   */
  template<typename Derived>
  std::shared_ptr<Derived> as()
  {
    auto derived_ptr = std::dynamic_pointer_cast<Derived>(shared_from_this());
    if (derived_ptr == nullptr) {
      throw std::runtime_error("[DeviceBase::as] Error: Cannot downcast device " + name + "!");
    }
    return derived_ptr;
  }

  /**
   * @brief Helper to convert const base class pointer to const derived class pointer.
   * DeviceBase is pure virtual, so a DeviceBase pointer is always actually pointing to a Derived class.
   *
   * @return std::shared_ptr<const Derived> pointer to this object as its derived class
   */
  template<typename Derived>
  std::shared_ptr<const Derived> as() const
  {
    auto derived_ptr = std::dynamic_pointer_cast<const Derived>(shared_from_this());
    if (derived_ptr == nullptr) {
      throw std::runtime_error("[DeviceBase::as] Error: Cannot downcast device " + name + "!");
    }
    return derived_ptr;
  }

  /**
   * @brief Equality operator for unit testing
   */
  virtual bool operator==(const DeviceBase & rhs) const = 0;
};

class DeviceConfig : public DeviceBase
{
public:
  int port = 0;

  /**
   * @brief Returns a pointer to a deep copy of this object
   *
   * @return std::shared_ptr<DeviceBase>
   */
  virtual std::shared_ptr<DeviceBase> clone() const = 0;

  /**
   * @brief Equality operator for unit testing
   */
  virtual bool operator==(const DeviceBase & rhs) const = 0;
};

class DeviceData : public DeviceBase
{
public:
  DeviceData(std::string name, device_type_e type)
  {
    this->type = type;
    this->name = name;
  }

  virtual void update(std::shared_ptr<DeviceData> data_ptr) = 0;

  /**
   * @brief Returns the size of the serial packet in bytes going from the coprocessor to the V5 Brain.
   *
   * @return int
   */
  virtual int getActuatorPacketSize() const = 0;

  /**
   * @brief Returns the size of the serial packet in bytes going from the V5 Brain to the coprocessor.
   *
   * @return int
   */
  virtual int getSensorPacketSize() const = 0;

  /**
   * @brief Returns a pointer to a deep copy of this object
   *
   * @return std::shared_ptr<DeviceBase>
   */
  virtual std::shared_ptr<DeviceBase> clone() const = 0;

  /**
   * @brief Equality operator for unit testing
   */
  virtual bool operator==(const DeviceBase & rhs) const = 0;

  /**
   * @brief Converts device data to byte stream.
   *
   * @param to_v5	set to true when the data is going from coprocessor -> V5 Brain
   * @return std::vector<unsigned char> byte stream
   */
  virtual std::vector<unsigned char> serialize(hardware_type_e hardware_type) const = 0;

  /**
   * @brief Updates device data from byte stream.
   *
   * @param data byte stream as unsigned char vector
   * @param from_coprocessor set to true when the data is going from coprocesspr -> V5 Brain
   */
  virtual void deserialize(
    const std::vector<unsigned char> & data,
    hardware_type_e hardware_type)  = 0;

  /**
   * @brief Validates serial msg size
   *
   * @param data serialized data buffer
   * @param msg_size	expected msg size
   */
  void checkMsgSize(std::vector<unsigned char> data, int msg_size) const
  {
    if (data.size() != msg_size) {
      throw std::runtime_error(
              "[DeviceData::checkMsgSize] Error: Device " + name + " received incorrect serial msg size. " +
              "Expecting " + std::to_string(
                msg_size) + " bytes, received " + std::to_string(data.size()) + " bytes.");
    }
  }

  void checkMsgSize(int true_size, int expected_size) const
  {
    if (true_size != expected_size) {
      throw std::runtime_error(
              "[DeviceData::checkMsgSize] Error: Device " + name + " received incorrect serial msg size. " +
              "Expecting " + std::to_string(
                expected_size) + " bytes, received " + std::to_string(true_size) + " bytes.");
    }
  }
};

struct DevicePair
{
  std::shared_ptr<const DeviceConfig> config_ptr;
  std::shared_ptr<DeviceData> data_ptr;

  bool operator==(const DevicePair & rhs) const
  {
    return (*config_ptr == *rhs.config_ptr) && (*data_ptr == *rhs.data_ptr);
  }


  DevicePair clone() const
  {
    DevicePair obj;
    obj.config_ptr = config_ptr->clone()->as<const DeviceConfig>();
    obj.data_ptr = data_ptr->clone()->as<DeviceData>();
    return obj;
  }
};

} // namespace devices

} // namespace ghost_v5_interfaces
