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
#include <bitset>
#include "ghost_util/test_util.hpp"
#include "ghost_v5_interfaces/devices/device_config_map.hpp"
#include "ghost_v5_interfaces/devices/joystick_device_interface.hpp"
#include "ghost_v5_interfaces/devices/motor_device_interface.hpp"
#include "ghost_v5_interfaces/devices/digital_device_interface.hpp"
#include "ghost_v5_interfaces/devices/rotation_sensor_device_interface.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"
#include "ghost_v5_interfaces/test/device_test_utils.hpp"
#include "ghost_v5_interfaces/util/device_config_factory_utils.hpp"
#include "yaml-cpp/yaml.h"

#include <algorithm>

using ghost_v5_interfaces::util::loadRobotConfigFromYAML;
using namespace ghost_util;
using namespace ghost_v5_interfaces::devices;
using namespace ghost_v5_interfaces::test_util;
using namespace ghost_v5_interfaces;

class RobotHardwareInterfaceTestFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    std::string config_path = std::string(getenv("VEXU_HOME")) +
      "/01_Libraries/ghost_v5_interfaces/test/config/example_robot_2.yaml";
    config_yaml_ = YAML::LoadFile(config_path);

    device_config_map_ptr_single_joy_ = loadRobotConfigFromYAML(config_yaml_, false);

    config_yaml_["port_configuration"]["use_partner_joystick"] = true;
    device_config_map_ptr_dual_joy_ = loadRobotConfigFromYAML(config_yaml_, false);
  }

  std::shared_ptr<DeviceConfigMap> device_config_map_ptr_single_joy_;
  std::shared_ptr<DeviceConfigMap> device_config_map_ptr_dual_joy_;
  YAML::Node config_yaml_;
};

TEST_F(RobotHardwareInterfaceTestFixture, testSerializationPipelineCoprocessorToV5) {
  RobotHardwareInterface hw_interface(device_config_map_ptr_single_joy_,
    hardware_type_e::COPROCESSOR);

  // Update all motors in the default robot config
  auto motor_data_1 = getRandomMotorData(true);
  motor_data_1->name = "left_drive_motor";
  hw_interface.setDeviceData(motor_data_1);
  auto motor_data_2 = getRandomMotorData(true);
  motor_data_2->name = "test_motor";
  hw_interface.setDeviceData(motor_data_2);
  auto motor_data_3 = getRandomMotorData(true);
  motor_data_3->name = "default_motor";
  hw_interface.setDeviceData(motor_data_3);
  
  // Update Digital In
  auto digital_in = getRandomDigitalInputDeviceData();
  digital_in->name = "digital_in";
  digital_in->value = false;
  hw_interface.setDeviceData(digital_in);

  // Update Digital Out
  auto digital_out = getRandomDigitalOutputDeviceData();
  digital_out->name = "digital_out";
  hw_interface.setDeviceData(digital_out);

  RobotHardwareInterface hw_interface_copy(device_config_map_ptr_single_joy_,
    hardware_type_e::V5_BRAIN);
  std::vector<unsigned char> serial_data = hw_interface.serialize();
  hw_interface_copy.deserialize(serial_data);

  EXPECT_TRUE(hw_interface.isDataEqual(hw_interface_copy));
}


TEST_F(RobotHardwareInterfaceTestFixture, testSerializationPipelineV5ToCoprocessor) {
  RobotHardwareInterface hw_interface(device_config_map_ptr_single_joy_, hardware_type_e::V5_BRAIN);

  // Update Motors
  auto motor_data_1 = getRandomMotorData(false);
  motor_data_1->name = "left_drive_motor";
  hw_interface.setDeviceData(motor_data_1);
  auto motor_data_2 = getRandomMotorData(false);
  motor_data_2->name = "test_motor";
  hw_interface.setDeviceData(motor_data_2);
  auto motor_data_3 = getRandomMotorData(false);
  motor_data_3->name = "default_motor";
  hw_interface.setDeviceData(motor_data_3);

  // Update Rotation Sensors
  auto rotation_sensor_1 = getRandomRotationSensorData();
  rotation_sensor_1->name = "rotation_sensor_1";
  hw_interface.setDeviceData(rotation_sensor_1);
  auto rotation_sensor_2 = getRandomRotationSensorData();
  rotation_sensor_2->name = "rotation_sensor_2";
  hw_interface.setDeviceData(rotation_sensor_2);

  // Update Inertial Sensor
  auto inertial_sensor_1 = getRandomInertialSensorData();
  inertial_sensor_1->name = "inertial_sensor_1";
  hw_interface.setDeviceData(inertial_sensor_1);

  // Update Digital In
  auto digital_in = getRandomDigitalInputDeviceData();
  digital_in->name = "digital_in";
  digital_in->value = false;
  hw_interface.setDeviceData(digital_in);

  // Update Digital Out
  auto digital_out = getRandomDigitalOutputDeviceData();
  digital_out->name = "digital_out";
  digital_out->value = false;
  hw_interface.setDeviceData(digital_out);

  // Update Competition State
  hw_interface.setDisabledStatus(getRandomBool());
  hw_interface.setAutonomousStatus(getRandomBool());
  hw_interface.setConnectedStatus(getRandomBool());

  // Update Joystick
  auto joy = getRandomJoystickData();
  joy->name = MAIN_JOYSTICK_NAME;
  hw_interface.setDeviceData(joy);

  RobotHardwareInterface hw_interface_copy(device_config_map_ptr_single_joy_,
    hardware_type_e::COPROCESSOR);
  std::vector<unsigned char> serial_data = hw_interface.serialize();
  hw_interface_copy.deserialize(serial_data);

  EXPECT_TRUE(hw_interface.isDataEqual(hw_interface_copy));
}

TEST_F(RobotHardwareInterfaceTestFixture, testSerializationPipelineV5ToCoprocessorDualJoystick) {
  RobotHardwareInterface hw_interface(device_config_map_ptr_dual_joy_, hardware_type_e::V5_BRAIN);

  // Update Motors
  auto motor_data_1 = getRandomMotorData(false);
  motor_data_1->name = "left_drive_motor";
  hw_interface.setDeviceData(motor_data_1);
  auto motor_data_2 = getRandomMotorData(false);
  motor_data_2->name = "test_motor";
  hw_interface.setDeviceData(motor_data_2);
  auto motor_data_3 = getRandomMotorData(false);
  motor_data_3->name = "default_motor";
  hw_interface.setDeviceData(motor_data_3);

  // Update Rotation Sensors
  auto rotation_sensor_1 = getRandomRotationSensorData();
  rotation_sensor_1->name = "rotation_sensor_1";
  hw_interface.setDeviceData(rotation_sensor_1);
  auto rotation_sensor_2 = getRandomRotationSensorData();
  rotation_sensor_2->name = "rotation_sensor_2";
  hw_interface.setDeviceData(rotation_sensor_2);

  // Update Inertial Sensor
  auto inertial_sensor_1 = getRandomInertialSensorData();
  inertial_sensor_1->name = "inertial_sensor_1";
  hw_interface.setDeviceData(inertial_sensor_1);

  // Update Digital In
  auto digital_in = getRandomDigitalInputDeviceData();
  digital_in->name = "digital_in";
  hw_interface.setDeviceData(digital_in);

  // Update Digital Out
  auto digital_out = getRandomDigitalOutputDeviceData();
  digital_out->name = "digital_out";
  digital_out->value = false;
  hw_interface.setDeviceData(digital_out);

  // Update Competition State
  hw_interface.setDisabledStatus(getRandomBool());
  hw_interface.setAutonomousStatus(getRandomBool());
  hw_interface.setConnectedStatus(getRandomBool());

  // Update Joysticks
  auto joy = getRandomJoystickData();
  joy->name = MAIN_JOYSTICK_NAME;
  hw_interface.setDeviceData(joy);

  auto joy_2 = getRandomJoystickData();
  joy_2->name = PARTNER_JOYSTICK_NAME;
  hw_interface.setDeviceData(joy_2);

  RobotHardwareInterface hw_interface_copy(device_config_map_ptr_dual_joy_,
    hardware_type_e::COPROCESSOR);
  std::vector<unsigned char> serial_data = hw_interface.serialize();
  hw_interface_copy.deserialize(serial_data);
  
  EXPECT_TRUE(hw_interface.isDataEqual(hw_interface_copy));
}
