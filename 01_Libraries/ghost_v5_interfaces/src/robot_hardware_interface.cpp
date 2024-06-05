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

#include "ghost_util/math_util.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"


using namespace ghost_v5_interfaces::devices;
using namespace ghost_v5_interfaces;
namespace ghost_v5_interfaces
{

RobotHardwareInterface::RobotHardwareInterface(
  std::shared_ptr<DeviceConfigMap> robot_config_ptr,
  hardware_type_e hardware_type)
: robot_config_ptr_(robot_config_ptr->clone()),
  hardware_type_(hardware_type)
{
  sensor_update_msg_length_ = 0;
  actuator_command_msg_length_ = 0;

  for (const auto & [key, val] : *robot_config_ptr_) {
    DevicePair pair;
    pair.config_ptr = val;
    if (pair.config_ptr->type == device_type_e::MOTOR) {
      pair.data_ptr = std::make_shared<MotorDeviceData>(
        val->name,
        pair.config_ptr->as<const MotorDeviceConfig>()->serial_config);
    } else if (pair.config_ptr->type == device_type_e::ROTATION_SENSOR) {
      pair.data_ptr = std::make_shared<RotationSensorDeviceData>(
        val->name,
        pair.config_ptr->as<const RotationSensorDeviceConfig>()->serial_config);
    } else if (pair.config_ptr->type == device_type_e::INERTIAL_SENSOR) {
      pair.data_ptr = std::make_shared<InertialSensorDeviceData>(
        val->name,
        pair.config_ptr->as<const InertialSensorDeviceConfig>()->serial_config);
    } else if (pair.config_ptr->type == device_type_e::JOYSTICK) {
      pair.data_ptr = std::make_shared<JoystickDeviceData>(
        val->name);
    } else if (pair.config_ptr->type == device_type_e::DIGITAL) {
      pair.data_ptr = std::make_shared<DigitalDeviceData>(
        val->name,
        pair.config_ptr->as<const DigitalDeviceConfig>()->serial_config);
    } else {
      throw std::runtime_error(
              "[RobotHardwareInterface::RobotHardwareInterface()] Device type " + std::to_string(
                pair.config_ptr->type) + " is unsupported!");
    }
    pair.data_ptr->name = val->name;
    device_pair_name_map_[val->name] = pair;
    device_pair_port_map_[pair.config_ptr->port] = pair;
    port_to_device_name_map_.emplace(pair.config_ptr->port, val->name);

    // Update msg lengths based on each non-digital device
    if (pair.config_ptr->type != device_type_e::DIGITAL) {
        sensor_update_msg_length_ += pair.data_ptr->getSensorPacketSize();
        actuator_command_msg_length_ += pair.data_ptr->getActuatorPacketSize();
    }
  }

  for (const auto & [key, val] : port_to_device_name_map_) {
    device_names_ordered_by_port_.emplace_back(val);
  }

  // Add Competiton State and Digital IO to each command msg
  actuator_command_msg_length_ += 2;
  sensor_update_msg_length_ += 2;
}

std::vector<unsigned char> RobotHardwareInterface::serialize() const
{
  std::vector<unsigned char> serial_data;
  std::unique_lock<CROSSPLATFORM_MUTEX_T> update_lock(update_mutex_);

  if (hardware_type_ == hardware_type_e::V5_BRAIN) {
    serial_data.push_back(
      packByte(
        std::vector<bool>{
        is_disabled_, is_autonomous_, is_connected_, 0, 0, 0, 0, 0
      }));
  } else if (hardware_type_ == hardware_type_e::COPROCESSOR) {
    serial_data.push_back((unsigned char) 0);
  }

  // Reserve empty byte for all Digital IO Ports
  serial_data.push_back((unsigned char) 0);

  for (const auto & [key, val] : device_pair_port_map_) {
    auto device_serial_msg = val.data_ptr->serialize(hardware_type_);
    if (val.config_ptr->type == device_type_e::DIGITAL) {
        setBit(serial_data[1], (val.config_ptr->port - 22), (device_serial_msg[0] == 1));
    }
    else {
      serial_data.insert(serial_data.end(), device_serial_msg.begin(), device_serial_msg.end());
    }
  }

  // Error Checking
  int expected_size = 0;
  if (hardware_type_ == hardware_type_e::V5_BRAIN) {
    expected_size = sensor_update_msg_length_;
  } else if (hardware_type_ == hardware_type_e::COPROCESSOR) {
    expected_size = actuator_command_msg_length_;
  }

  if (serial_data.size() != expected_size) {
    throw std::runtime_error(
            "[RobotHardwareInterface::serialize()] Error: Serial Msg Length does not "
            "match data from Robot Hardware Interface! Expected: " + std::to_string(expected_size) +
            " Actual: " + std::to_string(serial_data.size()));
  }

  return serial_data;
}

int RobotHardwareInterface::deserialize(const std::vector<unsigned char> & msg)
{
  // Error Checking
  int expected_size = 0;

  if (hardware_type_ == hardware_type_e::V5_BRAIN) {
    expected_size = actuator_command_msg_length_;
  } else if (hardware_type_ == hardware_type_e::COPROCESSOR) {
    expected_size = sensor_update_msg_length_;
  }

  if (msg.size() != expected_size) {
    throw std::runtime_error(
            "[RobotHardwareInterface::deserialize] Error: Serial Msg Length does not "
            "match data from Robot Hardware Interface! Expected: " + std::to_string(expected_size) +
            " Actual: " + std::to_string(msg.size()));
  }

  // Deserialize
  std::unique_lock<CROSSPLATFORM_MUTEX_T> update_lock(update_mutex_);

  if (hardware_type_ == hardware_type_e::COPROCESSOR) {
    // Unpack competition state
    auto packet_start_byte = unpackByte(msg[0]);
    is_disabled_ = packet_start_byte[0];
    is_autonomous_ = packet_start_byte[1];
    is_connected_ = packet_start_byte[2];
  }

  int byte_offset = 2;

  // Unpack each device in device tree
  for (const auto & [key, val] : device_pair_port_map_) {
    int msg_len;
    if (hardware_type_ == hardware_type_e::COPROCESSOR) {
      msg_len = val.data_ptr->getSensorPacketSize();
    } else if (hardware_type_ == hardware_type_e::V5_BRAIN) {
      msg_len = val.data_ptr->getActuatorPacketSize();
    } else {
      throw std::runtime_error(
              "[RobotHardwareInterface::deserialize] Error: Attempted to deserialize with unsupported hardware type.");
    }

    if (val.config_ptr->type == device_type_e::DIGITAL) {
      val.data_ptr->deserialize({getBit(msg[1], (val.config_ptr->port - 22))}, hardware_type_);
    }
    else {
      auto start_itr = msg.begin() + byte_offset;
      val.data_ptr->deserialize(
      std::vector<unsigned char>(
          start_itr,
          start_itr + msg_len), hardware_type_);
      byte_offset += msg_len;
    }
  }
  return byte_offset;
}

bool RobotHardwareInterface::isDataEqual(const RobotHardwareInterface & rhs) const
{
  // Competition State
  bool eq = (is_disabled_ == rhs.is_disabled_);
  eq &= (is_autonomous_ == rhs.is_autonomous_);
  eq &= (is_connected_ == rhs.is_connected_);

  // Serialization
  eq &= (msg_id_ == rhs.msg_id_);
  eq &= (actuator_command_msg_length_ == rhs.actuator_command_msg_length_);
  eq &= (sensor_update_msg_length_ == rhs.sensor_update_msg_length_);

  for (const auto & [key, val] : device_pair_name_map_) {
    if (rhs.device_pair_name_map_.count(key) == 0) {
      return false;
    }
    auto rhs_device_pair = rhs.device_pair_name_map_.at(key);
    eq &= (*val.data_ptr == *rhs_device_pair.data_ptr);
    eq &= (*val.config_ptr == *rhs_device_pair.config_ptr);
  }

  for (const auto & [key, val] : device_pair_port_map_) {
    if (rhs.device_pair_port_map_.count(key) == 0) {
      return false;
    }
  }

  return eq;
}

bool RobotHardwareInterface::operator==(const RobotHardwareInterface & rhs) const
{
  return (hardware_type_ == rhs.hardware_type_) && isDataEqual(rhs);
}

DevicePair RobotHardwareInterface::getDevicePair(const std::string & device_name) const
{
  throwOnNonexistentDevice(device_name);
  return device_pair_name_map_.at(device_name).clone();
}

void RobotHardwareInterface::setDeviceDataNoLock(std::shared_ptr<DeviceData> device_data)
{
  device_pair_name_map_.at(device_data->name).data_ptr->update(device_data);
}

void RobotHardwareInterface::setDeviceData(std::shared_ptr<DeviceData> device_data)
{
  throwOnNonexistentDevice(device_data->name);

  std::unique_lock<CROSSPLATFORM_MUTEX_T> update_lock(update_mutex_);
  setDeviceDataNoLock(device_data);
}

float RobotHardwareInterface::getMotorPosition(const std::string & motor_name)
{
  return getDeviceData<MotorDeviceData>(motor_name)->curr_position;
}

float RobotHardwareInterface::getMotorCurrentMA(const std::string & motor_name)
{
  return getDeviceData<MotorDeviceData>(motor_name)->curr_current_ma;
}

float RobotHardwareInterface::getMotorVelocityRPM(const std::string & motor_name)
{
  return getDeviceData<MotorDeviceData>(motor_name)->curr_velocity_rpm;
}

void RobotHardwareInterface::setMotorPositionCommand(
  const std::string & motor_name,
  float position_cmd)
{
  std::unique_lock<CROSSPLATFORM_MUTEX_T> update_lock(update_mutex_);
  auto motor_data_ptr = getDeviceData<MotorDeviceData>(motor_name);
  motor_data_ptr->position_command = position_cmd;
  motor_data_ptr->position_control = true;
  setDeviceDataNoLock(motor_data_ptr);
}

void RobotHardwareInterface::setMotorVelocityCommandRPM(
  const std::string & motor_name,
  float velocity_cmd)
{
  std::unique_lock<CROSSPLATFORM_MUTEX_T> update_lock(update_mutex_);
  auto motor_data_ptr = getDeviceData<MotorDeviceData>(motor_name);
  motor_data_ptr->velocity_command = velocity_cmd;
  motor_data_ptr->velocity_control = true;
  setDeviceDataNoLock(motor_data_ptr);
}

void RobotHardwareInterface::setMotorVoltageCommandPercent(
  const std::string & motor_name,
  float voltage_cmd)
{
  std::unique_lock<CROSSPLATFORM_MUTEX_T> update_lock(update_mutex_);
  auto motor_data_ptr = getDeviceData<MotorDeviceData>(motor_name);
  motor_data_ptr->voltage_command = voltage_cmd;
  motor_data_ptr->voltage_control = true;
  setDeviceDataNoLock(motor_data_ptr);
}

void RobotHardwareInterface::setMotorTorqueCommandPercent(
  const std::string & motor_name,
  float torque_cmd)
{
  std::unique_lock<CROSSPLATFORM_MUTEX_T> update_lock(update_mutex_);
  auto motor_data_ptr = getDeviceData<MotorDeviceData>(motor_name);
  motor_data_ptr->torque_command = torque_cmd;
  motor_data_ptr->torque_control = true;
  setDeviceDataNoLock(motor_data_ptr);
}

void RobotHardwareInterface::setMotorCommand(
  const std::string & motor_name,
  float position_cmd,
  float velocity_cmd,
  float voltage_cmd,
  float torque_cmd)
{
  std::unique_lock<CROSSPLATFORM_MUTEX_T> update_lock(update_mutex_);
  auto motor_data_ptr = getDeviceData<MotorDeviceData>(motor_name);
  motor_data_ptr->position_command = position_cmd;
  motor_data_ptr->velocity_command = velocity_cmd;
  motor_data_ptr->voltage_command = voltage_cmd;
  motor_data_ptr->torque_command = torque_cmd;
  setDeviceDataNoLock(motor_data_ptr);
}

void RobotHardwareInterface::setMotorControlMode(
  const std::string & motor_name,
  bool position_control,
  bool velocity_control,
  bool voltage_control,
  bool torque_control)
{
  std::unique_lock<CROSSPLATFORM_MUTEX_T> update_lock(update_mutex_);
  auto motor_data_ptr = getDeviceData<MotorDeviceData>(motor_name);
  motor_data_ptr->position_control = position_control;
  motor_data_ptr->velocity_control = velocity_control;
  motor_data_ptr->voltage_control = voltage_control;
  motor_data_ptr->torque_control = torque_control;
  setDeviceDataNoLock(motor_data_ptr);
}

void RobotHardwareInterface::setMotorCurrentLimitMilliAmps(
  const std::string & motor_name,
  int32_t current_limit_ma)
{
  std::unique_lock<CROSSPLATFORM_MUTEX_T> update_lock(update_mutex_);
  auto motor_data_ptr = getDeviceData<MotorDeviceData>(motor_name);
  motor_data_ptr->current_limit = ghost_util::clamp<int32_t>(current_limit_ma, 0, 2500);
  setDeviceDataNoLock(motor_data_ptr);
}

float RobotHardwareInterface::getRotationSensorAngleDegrees(const std::string & sensor_name)
{
  if (getDeviceConfig<RotationSensorDeviceConfig>(sensor_name)->serial_config.send_angle_data) {
    return getDeviceData<RotationSensorDeviceData>(sensor_name)->angle;
  } else {
    throw std::runtime_error(
            std::string(
              "[RobotHardwareInterface::getRotationSensorAngleDegrees] Error: ") + sensor_name +
            " is not configured to send angle data!");
  }
}

float RobotHardwareInterface::getRotationSensorPositionDegrees(const std::string & sensor_name)
{
  if (getDeviceConfig<RotationSensorDeviceConfig>(sensor_name)->serial_config.send_position_data) {
    return getDeviceData<RotationSensorDeviceData>(sensor_name)->position;
  } else {
    throw std::runtime_error(
            std::string(
              "[RobotHardwareInterface::getRotationSensorAngleDegrees] Error: ") + sensor_name +
            " is not configured to send position data!");
  }
}

float RobotHardwareInterface::getRotationSensorVelocityRPM(const std::string & sensor_name)
{
  if (getDeviceConfig<RotationSensorDeviceConfig>(sensor_name)->serial_config.send_velocity_data) {
    return getDeviceData<RotationSensorDeviceData>(sensor_name)->velocity;
  } else {
    throw std::runtime_error(
            std::string(
              "[RobotHardwareInterface::getRotationSensorAngleDegrees] Error: ") + sensor_name +
            " is not configured to send velocity data!");
  }
}

float RobotHardwareInterface::getInertialSensorXRate(const std::string & sensor_name)
{
  if (getDeviceConfig<InertialSensorDeviceConfig>(sensor_name)->serial_config.send_gyro_data) {
    return getDeviceData<InertialSensorDeviceData>(sensor_name)->x_rate;
  } else {
    throw std::runtime_error(
            std::string(
              "[RobotHardwareInterface::getInertialSensorXRate] Error: ") + sensor_name +
            " is not configured to send gyro data!");
  }
}

float RobotHardwareInterface::getInertialSensorYRate(const std::string & sensor_name)
{
  if (getDeviceConfig<InertialSensorDeviceConfig>(sensor_name)->serial_config.send_gyro_data) {
    return getDeviceData<InertialSensorDeviceData>(sensor_name)->y_rate;
  } else {
    throw std::runtime_error(
            std::string(
              "[RobotHardwareInterface::getInertialSensorYRate] Error: ") + sensor_name +
            " is not configured to send gyro data!");
  }
}

float RobotHardwareInterface::getInertialSensorZRate(const std::string & sensor_name)
{
  if (getDeviceConfig<InertialSensorDeviceConfig>(sensor_name)->serial_config.send_gyro_data) {
    return getDeviceData<InertialSensorDeviceData>(sensor_name)->z_rate;
  } else {
    throw std::runtime_error(
            std::string(
              "[RobotHardwareInterface::getInertialSensorZRate] Error: ") + sensor_name +
            " is not configured to send gyro data!");
  }
}

float RobotHardwareInterface::getInertialSensorXAccel(const std::string & sensor_name)
{
  if (getDeviceConfig<InertialSensorDeviceConfig>(sensor_name)->serial_config.send_accel_data) {
    return getDeviceData<InertialSensorDeviceData>(sensor_name)->x_accel;
  } else {
    throw std::runtime_error(
            std::string(
              "[RobotHardwareInterface::getInertialSensorXAccel] Error: ") + sensor_name +
            " is not configured to send accel data!");
  }
}

float RobotHardwareInterface::getInertialSensorYAccel(const std::string & sensor_name)
{
  if (getDeviceConfig<InertialSensorDeviceConfig>(sensor_name)->serial_config.send_accel_data) {
    return getDeviceData<InertialSensorDeviceData>(sensor_name)->y_accel;
  } else {
    throw std::runtime_error(
            std::string(
              "[RobotHardwareInterface::getInertialSensorYAccel] Error: ") + sensor_name +
            " is not configured to send accel data!");
  }
}

float RobotHardwareInterface::getInertialSensorZAccel(const std::string & sensor_name)
{
  if (getDeviceConfig<InertialSensorDeviceConfig>(sensor_name)->serial_config.send_accel_data) {
    return getDeviceData<InertialSensorDeviceData>(sensor_name)->z_accel;
  } else {
    throw std::runtime_error(
            std::string(
              "[RobotHardwareInterface::getInertialSensorZAccel] Error: ") + sensor_name +
            " is not configured to send accel data!");
  }
}

float RobotHardwareInterface::getInertialSensorHeading(const std::string & sensor_name)
{
  if (getDeviceConfig<InertialSensorDeviceConfig>(sensor_name)->serial_config.send_heading_data) {
    return getDeviceData<InertialSensorDeviceData>(sensor_name)->heading;
  } else {
    throw std::runtime_error(
            std::string(
              "[RobotHardwareInterface::getInertialSensorHeading] Error: ") + sensor_name +
            " is not configured to send heading data!");
  }
}

bool RobotHardwareInterface::getDigitalDeviceValue(const std::string & sensor_name){
  return getDeviceData<DigitalDeviceData>(sensor_name)->value;
}

bool RobotHardwareInterface::setDigitalDeviceValue(const std::string & device_name, bool value){

  auto io_type = getDeviceConfig<DigitalDeviceConfig>(device_name)->serial_config.io_type;
  if (io_type == SENSOR && hardware_type_ != V5_BRAIN) {
      throw std::runtime_error("[RobotHardwareInterface::setDigitalDeviceValue] Error: Attempted to set Digital Sensor value from Coprocessor.");
  }
  if (io_type == ACTUATOR && hardware_type_ != COPROCESSOR) {
      throw std::runtime_error("[RobotHardwareInterface::setDigitalDeviceValue] Error: Attempted to set Digital Actuator value from V5 Brain.");
  }

  std::unique_lock<CROSSPLATFORM_MUTEX_T> update_lock(update_mutex_);
  auto device_data_ptr = getDeviceData<DigitalDeviceData>(device_name);
  device_data_ptr->value = value;
  setDeviceDataNoLock(device_data_ptr);
}

std::shared_ptr<JoystickDeviceData> RobotHardwareInterface::getMainJoystickData()
{
  return getDeviceData<JoystickDeviceData>(MAIN_JOYSTICK_NAME);
}

std::shared_ptr<JoystickDeviceData> RobotHardwareInterface::getPartnerJoystickData()
{
  return getDeviceData<JoystickDeviceData>(PARTNER_JOYSTICK_NAME);
}

void RobotHardwareInterface::throwOnNonexistentDevice(const std::string & device_name) const
{
  if (device_pair_name_map_.count(device_name) == 0) {
    throw std::runtime_error(
            "[RobotHardwareInterface::getDeviceConfig] Error: Device name " + device_name +
            " does not exist!");
  }
}

} // namespace ghost_v5_interfaces
