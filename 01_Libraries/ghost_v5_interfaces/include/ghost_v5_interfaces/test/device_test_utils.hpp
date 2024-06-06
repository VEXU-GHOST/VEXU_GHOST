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
#include <ghost_util/test_util.hpp>
#include "ghost_v5_interfaces/devices/inertial_sensor_device_interface.hpp"
#include "ghost_v5_interfaces/devices/digital_device_interface.hpp"
#include "ghost_v5_interfaces/devices/joystick_device_interface.hpp"
#include "ghost_v5_interfaces/devices/motor_device_interface.hpp"
#include "ghost_v5_interfaces/devices/rotation_sensor_device_interface.hpp"

namespace ghost_v5_interfaces
{

namespace test_util
{

devices::MotorDeviceData::SerialConfig getRandomMotorSerialConfig()
{
  devices::MotorDeviceData::SerialConfig config;
  config.send_position_command = ghost_util::getRandomBool();
  config.send_velocity_command = ghost_util::getRandomBool();
  config.send_voltage_command = ghost_util::getRandomBool();
  config.send_torque_command = ghost_util::getRandomBool();
  config.send_torque_data = ghost_util::getRandomBool();
  config.send_voltage_data = ghost_util::getRandomBool();
  config.send_current_data = ghost_util::getRandomBool();
  config.send_power_data = ghost_util::getRandomBool();
  config.send_temp_data = ghost_util::getRandomBool();
  return config;
}

devices::RotationSensorDeviceData::SerialConfig getRandomRotationSensorSerialConfig()
{
  devices::RotationSensorDeviceData::SerialConfig config;
  config.send_angle_data = ghost_util::getRandomBool();
  config.send_position_data = ghost_util::getRandomBool();
  config.send_velocity_data = ghost_util::getRandomBool();
  return config;
}

devices::InertialSensorDeviceData::SerialConfig getRandomInertialSensorSerialConfig()
{
  devices::InertialSensorDeviceData::SerialConfig config;
  config.send_accel_data = ghost_util::getRandomBool();
  config.send_gyro_data = ghost_util::getRandomBool();
  config.send_heading_data = ghost_util::getRandomBool();
  return config;
}

std::shared_ptr<devices::JoystickDeviceData> getRandomJoystickData()
{
  auto joy_ptr = std::make_shared<devices::JoystickDeviceData>("joy_" + std::to_string(rand() % 2));

  joy_ptr->left_x = ghost_util::getRandomFloat();
  joy_ptr->left_y = ghost_util::getRandomFloat();
  joy_ptr->right_x = ghost_util::getRandomFloat();
  joy_ptr->right_y = ghost_util::getRandomFloat();
  joy_ptr->btn_a = ghost_util::getRandomBool();
  joy_ptr->btn_b = ghost_util::getRandomBool();
  joy_ptr->btn_x = ghost_util::getRandomBool();
  joy_ptr->btn_y = ghost_util::getRandomBool();
  joy_ptr->btn_r1 = ghost_util::getRandomBool();
  joy_ptr->btn_r2 = ghost_util::getRandomBool();
  joy_ptr->btn_l1 = ghost_util::getRandomBool();
  joy_ptr->btn_l2 = ghost_util::getRandomBool();
  joy_ptr->btn_u = ghost_util::getRandomBool();
  joy_ptr->btn_l = ghost_util::getRandomBool();
  joy_ptr->btn_r = ghost_util::getRandomBool();
  joy_ptr->btn_d = ghost_util::getRandomBool();

  return joy_ptr;
}

std::shared_ptr<devices::MotorDeviceData> getRandomMotorData(
  bool actuator_cmd,
  devices::MotorDeviceData::SerialConfig serial_config = devices::MotorDeviceData::SerialConfig())
{
  auto motor_ptr = std::make_shared<devices::MotorDeviceData>("test", serial_config);

  // Actuator Values
  if (actuator_cmd) {
    if (serial_config.send_position_command) {
      motor_ptr->position_command = ghost_util::getRandomFloat();
      motor_ptr->position_control = ghost_util::getRandomBool();
    }
    if (serial_config.send_velocity_command) {
      motor_ptr->velocity_command = ghost_util::getRandomFloat();
      motor_ptr->velocity_control = ghost_util::getRandomBool();
    }
    if (serial_config.send_voltage_command) {
      motor_ptr->voltage_command = ghost_util::getRandomFloat();
      motor_ptr->voltage_control = ghost_util::getRandomBool();
    }
    if (serial_config.send_torque_command) {
      motor_ptr->torque_command = ghost_util::getRandomFloat();
      motor_ptr->torque_control = ghost_util::getRandomBool();
    }
    motor_ptr->current_limit = ghost_util::getRandomFloat();
  } else {
    motor_ptr->curr_position = ghost_util::getRandomFloat();
    motor_ptr->curr_velocity_rpm = ghost_util::getRandomFloat();

    if (serial_config.send_torque_data) {
      motor_ptr->curr_torque_nm = ghost_util::getRandomFloat();
    }
    if (serial_config.send_voltage_data) {
      motor_ptr->curr_voltage_mv = ghost_util::getRandomFloat();
    }
    if (serial_config.send_current_data) {
      motor_ptr->curr_current_ma = ghost_util::getRandomFloat();
    }
    if (serial_config.send_power_data) {
      motor_ptr->curr_power_w = ghost_util::getRandomFloat();
    }
    if (serial_config.send_temp_data) {
      motor_ptr->curr_temp_c = ghost_util::getRandomFloat();
    }
  }


  return motor_ptr;
}

std::shared_ptr<devices::RotationSensorDeviceData> getRandomRotationSensorData(
  devices::RotationSensorDeviceData::SerialConfig serial_config = devices::RotationSensorDeviceData::SerialConfig())
{
  auto rot_sensor_ptr = std::make_shared<devices::RotationSensorDeviceData>("test", serial_config);
  if (serial_config.send_angle_data) {
    rot_sensor_ptr->angle = ghost_util::getRandomFloat();
  }
  if (serial_config.send_position_data) {
    rot_sensor_ptr->position = ghost_util::getRandomFloat();
  }
  if (serial_config.send_velocity_data) {
    rot_sensor_ptr->velocity = ghost_util::getRandomFloat();
  }

  return rot_sensor_ptr;
}

std::shared_ptr<devices::InertialSensorDeviceData> getRandomInertialSensorData(
  devices::InertialSensorDeviceData::SerialConfig serial_config = devices::InertialSensorDeviceData::SerialConfig())
{
  auto inertial_sensor_ptr = std::make_shared<devices::InertialSensorDeviceData>(
    "test",
    serial_config);
  if (serial_config.send_accel_data) {
    inertial_sensor_ptr->x_accel = ghost_util::getRandomFloat();
    inertial_sensor_ptr->y_accel = ghost_util::getRandomFloat();
    inertial_sensor_ptr->z_accel = ghost_util::getRandomFloat();
  }
  if (serial_config.send_gyro_data) {
    inertial_sensor_ptr->x_rate = ghost_util::getRandomFloat();
    inertial_sensor_ptr->y_rate = ghost_util::getRandomFloat();
    inertial_sensor_ptr->z_rate = ghost_util::getRandomFloat();
  }
  if (serial_config.send_heading_data) {
    inertial_sensor_ptr->heading = ghost_util::getRandomFloat();
  }

  return inertial_sensor_ptr;
}


std::shared_ptr<devices::DigitalOutputDeviceData> getRandomDigitalOutputDeviceData(){
  auto digital_output_ptr = std::make_shared<devices::DigitalOutputDeviceData>("test");
  digital_output_ptr->value = ghost_util::getRandomBool();
  return digital_output_ptr;
}

std::shared_ptr<devices::DigitalInputDeviceData> getRandomDigitalInputDeviceData(){
  auto digital_input_ptr = std::make_shared<devices::DigitalInputDeviceData>("test");
  digital_input_ptr->value = ghost_util::getRandomBool();
  return digital_input_ptr;
}
}

}
