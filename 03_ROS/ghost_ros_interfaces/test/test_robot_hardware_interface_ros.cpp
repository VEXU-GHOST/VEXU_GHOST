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

#include <gtest/gtest.h>
#include "yaml-cpp/yaml.h"

#include <ghost_util/test_util.hpp>

#include "ghost_v5_interfaces/devices/device_config_map.hpp"
#include "ghost_v5_interfaces/devices/joystick_device_interface.hpp"
#include "ghost_v5_interfaces/devices/motor_device_interface.hpp"
#include "ghost_v5_interfaces/devices/rotation_sensor_device_interface.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"
#include "ghost_v5_interfaces/test/device_test_utils.hpp"
#include "ghost_v5_interfaces/util/device_config_factory_utils.hpp"

#include "ghost_ros_interfaces/msg_helpers/msg_helpers.hpp"


using namespace ghost_ros_interfaces::msg_helpers;
using namespace ghost_ros_interfaces;
using namespace ghost_util;
using namespace ghost_v5_interfaces::devices;
using namespace ghost_v5_interfaces::test_util;
using namespace ghost_v5_interfaces::util;
using namespace ghost_v5_interfaces;

class RobotHardwareInterfaceROSTestFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    std::string config_path = std::string(getenv("VEXU_HOME")) +
      "/01_Libraries/ghost_v5_interfaces/test/config/example_robot.yaml";
    config_yaml_ = YAML::LoadFile(config_path);
    config_yaml_["port_configuration"]["use_partner_joystick"] = true;

    device_config_map_ptr_ = loadRobotConfigFromYAML(config_yaml_, false);
    // TODO(maxxwilson): Does this hardware type matter? should one be V5_BRAIN for full coverage?
    rhi_input_ptr_ = std::make_shared<RobotHardwareInterface>(
      device_config_map_ptr_,
      devices::hardware_type_e::COPROCESSOR);
    rhi_output_ptr_ = std::make_shared<RobotHardwareInterface>(
      device_config_map_ptr_,
      devices::hardware_type_e::COPROCESSOR);
  }

  std::shared_ptr<RobotHardwareInterface> rhi_input_ptr_;
  std::shared_ptr<RobotHardwareInterface> rhi_output_ptr_;
  std::shared_ptr<DeviceConfigMap> device_config_map_ptr_;
  YAML::Node config_yaml_;
};

TEST_F(RobotHardwareInterfaceROSTestFixture, testRobotHardwareInterfaceSensorUpdate) {
  rhi_input_ptr_->setDisabledStatus(getRandomBool());
  rhi_input_ptr_->setAutonomousStatus(getRandomBool());
  rhi_input_ptr_->setConnectedStatus(getRandomBool());

  auto joy_data = getRandomJoystickData();
  joy_data->name = MAIN_JOYSTICK_NAME;
  rhi_input_ptr_->setDeviceData(joy_data);

  auto joy_data_2 = getRandomJoystickData();
  joy_data_2->name = PARTNER_JOYSTICK_NAME;
  rhi_input_ptr_->setDeviceData(joy_data_2);

  auto motor_data_ptr = getRandomMotorData(false);
  motor_data_ptr->name = "default_motor";
  rhi_input_ptr_->setDeviceData(motor_data_ptr);

  auto rotation_sensor_data_ptr = getRandomRotationSensorData();
  rotation_sensor_data_ptr->name = "rotation_sensor_1";
  rhi_input_ptr_->setDeviceData(rotation_sensor_data_ptr);

  auto msg = std::make_shared<ghost_msgs::msg::V5SensorUpdate>();

  // Convert to ROS Msg
  toROSMsg(*rhi_input_ptr_, *msg);
  fromROSMsg(*rhi_output_ptr_, *msg);

  EXPECT_EQ(*rhi_input_ptr_, *rhi_output_ptr_);
}

TEST_F(RobotHardwareInterfaceROSTestFixture, testRobotHardwareInterfaceActuatorCommand) {
  auto motor_data_ptr = getRandomMotorData(true);
  motor_data_ptr->name = "default_motor";
  rhi_input_ptr_->setDeviceData(motor_data_ptr);
  auto msg = std::make_shared<ghost_msgs::msg::V5ActuatorCommand>();

  // Convert to ROS Msg
  toROSMsg(*rhi_input_ptr_, *msg);
  fromROSMsg(*rhi_output_ptr_, *msg);

  EXPECT_EQ(*rhi_input_ptr_, *rhi_output_ptr_);
}

TEST_F(RobotHardwareInterfaceROSTestFixture, testRobotHardwareInterfaceFullCycle) {
  rhi_input_ptr_->setDisabledStatus(getRandomBool());
  rhi_input_ptr_->setAutonomousStatus(getRandomBool());
  rhi_input_ptr_->setConnectedStatus(getRandomBool());

  auto joy_data = getRandomJoystickData();
  joy_data->name = MAIN_JOYSTICK_NAME;
  rhi_input_ptr_->setDeviceData(joy_data);

  auto joy_data_2 = getRandomJoystickData();
  joy_data_2->name = PARTNER_JOYSTICK_NAME;
  rhi_input_ptr_->setDeviceData(joy_data_2);

  auto motor_data_ptr = std::make_shared<devices::MotorDeviceData>("default_motor");
  motor_data_ptr->position_command = getRandomFloat();
  motor_data_ptr->velocity_command = getRandomFloat();
  motor_data_ptr->torque_command = getRandomFloat();
  motor_data_ptr->voltage_command = getRandomFloat();
  motor_data_ptr->current_limit = getRandomFloat();
  motor_data_ptr->position_control = getRandomBool();
  motor_data_ptr->velocity_control = getRandomBool();
  motor_data_ptr->torque_control = getRandomBool();
  motor_data_ptr->voltage_control = getRandomBool();
  motor_data_ptr->curr_position = getRandomFloat();
  motor_data_ptr->curr_velocity_rpm = getRandomFloat();
  motor_data_ptr->curr_torque_nm = getRandomFloat();
  motor_data_ptr->curr_voltage_mv = getRandomFloat();
  motor_data_ptr->curr_current_ma = getRandomFloat();
  motor_data_ptr->curr_power_w = getRandomFloat();
  motor_data_ptr->curr_temp_c = getRandomFloat();
  rhi_input_ptr_->setDeviceData(motor_data_ptr);

  auto rotation_sensor_data_ptr = getRandomRotationSensorData();
  rotation_sensor_data_ptr->name = "rotation_sensor_1";
  rhi_input_ptr_->setDeviceData(rotation_sensor_data_ptr);

  auto msg_sensor_update = std::make_shared<ghost_msgs::msg::V5SensorUpdate>();
  auto msg_actuator_command = std::make_shared<ghost_msgs::msg::V5ActuatorCommand>();

  // Convert to ROS Msg
  toROSMsg(*rhi_input_ptr_, *msg_sensor_update);
  toROSMsg(*rhi_input_ptr_, *msg_actuator_command);
  fromROSMsg(*rhi_output_ptr_, *msg_sensor_update);
  fromROSMsg(*rhi_output_ptr_, *msg_actuator_command);

  EXPECT_EQ(*rhi_input_ptr_, *rhi_output_ptr_);
}

TEST_F(RobotHardwareInterfaceROSTestFixture, testRobotHardwareInterfaceFullCycleReverse) {
  rhi_input_ptr_->setDisabledStatus(getRandomBool());
  rhi_input_ptr_->setAutonomousStatus(getRandomBool());
  rhi_input_ptr_->setConnectedStatus(getRandomBool());

  auto joy_data = getRandomJoystickData();
  joy_data->name = MAIN_JOYSTICK_NAME;
  rhi_input_ptr_->setDeviceData(joy_data);

  auto joy_data_2 = getRandomJoystickData();
  joy_data_2->name = PARTNER_JOYSTICK_NAME;
  rhi_input_ptr_->setDeviceData(joy_data_2);

  auto motor_data_ptr = std::make_shared<devices::MotorDeviceData>("default_motor");
  motor_data_ptr->position_command = getRandomFloat();
  motor_data_ptr->velocity_command = getRandomFloat();
  motor_data_ptr->torque_command = getRandomFloat();
  motor_data_ptr->voltage_command = getRandomFloat();
  motor_data_ptr->current_limit = getRandomFloat();
  motor_data_ptr->position_control = getRandomBool();
  motor_data_ptr->velocity_control = getRandomBool();
  motor_data_ptr->torque_control = getRandomBool();
  motor_data_ptr->voltage_control = getRandomBool();
  motor_data_ptr->curr_position = getRandomFloat();
  motor_data_ptr->curr_velocity_rpm = getRandomFloat();
  motor_data_ptr->curr_torque_nm = getRandomFloat();
  motor_data_ptr->curr_voltage_mv = getRandomFloat();
  motor_data_ptr->curr_current_ma = getRandomFloat();
  motor_data_ptr->curr_power_w = getRandomFloat();
  motor_data_ptr->curr_temp_c = getRandomFloat();
  rhi_input_ptr_->setDeviceData(motor_data_ptr);

  auto rotation_sensor_data_ptr = getRandomRotationSensorData();
  rotation_sensor_data_ptr->name = "rotation_sensor_1";
  rhi_input_ptr_->setDeviceData(rotation_sensor_data_ptr);

  auto msg_sensor_update = std::make_shared<ghost_msgs::msg::V5SensorUpdate>();
  auto msg_actuator_command = std::make_shared<ghost_msgs::msg::V5ActuatorCommand>();

  // Convert to ROS Msg
  toROSMsg(*rhi_input_ptr_, *msg_actuator_command);
  toROSMsg(*rhi_input_ptr_, *msg_sensor_update);
  fromROSMsg(*rhi_output_ptr_, *msg_actuator_command);
  fromROSMsg(*rhi_output_ptr_, *msg_sensor_update);

  EXPECT_EQ(*rhi_input_ptr_, *rhi_output_ptr_);
}
