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

#include <ghost_v5_interfaces/util/load_inertial_sensor_device_config_yaml.hpp>
#include "gtest/gtest.h"
#include "yaml-cpp/yaml.h"

using namespace ghost_v5_interfaces::devices;
using namespace ghost_v5_interfaces::util;
using namespace ghost_v5_interfaces;

class TestLoadInertialSensorDeviceConfigYAML : public ::testing::Test
{
protected:
  void SetUp() override
  {
    std::string config_path = std::string(getenv("VEXU_HOME")) +
      "/01_Libraries/ghost_v5_interfaces/test/config/test_load_inertial_sensor_config.yaml";
    config_yaml_ = YAML::LoadFile(config_path);

    // Changed every param
    inertial_sensor_1_ = std::make_shared<InertialSensorDeviceConfig>();
    inertial_sensor_1_->port = 1;
    inertial_sensor_1_->name = "inertial_sensor_1";
    inertial_sensor_1_->type = device_type_e::INERTIAL_SENSOR;
    inertial_sensor_1_->serial_config.send_accel_data = true;
    inertial_sensor_1_->serial_config.send_gyro_data = true;
    inertial_sensor_1_->serial_config.send_heading_data = false;

    // Default (minimal required params)
    inertial_sensor_2_ = std::make_shared<InertialSensorDeviceConfig>();
    inertial_sensor_2_->port = 2;
    inertial_sensor_2_->name = "inertial_sensor_2";
    inertial_sensor_2_->type = device_type_e::INERTIAL_SENSOR;
  }

  std::shared_ptr<InertialSensorDeviceConfig> inertial_sensor_1_;
  std::shared_ptr<InertialSensorDeviceConfig> inertial_sensor_2_;

  YAML::Node config_yaml_;
};

/**
 * @brief Test that a InertialSensorDeviceData::SerialConfig struct can be properly loaded from YAML
 */
TEST_F(TestLoadInertialSensorDeviceConfigYAML, testLoadInertialSensorSerialConfig) {
  InertialSensorDeviceData::SerialConfig test_serial_config{};
  test_serial_config.send_accel_data = true;
  test_serial_config.send_gyro_data = true;
  test_serial_config.send_heading_data = false;

  InertialSensorDeviceData::SerialConfig loaded_serial_config;
  EXPECT_NO_THROW(
    loaded_serial_config =
    loadInertialSensorSerialConfigFromYAML(
      config_yaml_["port_configuration"][
        "device_configurations"]["inertial_sensor_config"]["serial"]));
  EXPECT_EQ(test_serial_config, loaded_serial_config);
}

/**
 * @brief Test that all fields are properly set when loading the InertialSensorDeviceConfig class
 */
TEST_F(TestLoadInertialSensorDeviceConfigYAML, testLoadCustomSensorConfigFromYAML) {
  auto config_node = config_yaml_["port_configuration"];
  std::string sensor_name = "inertial_sensor_1";
  auto config_ptr = std::make_shared<InertialSensorDeviceConfig>();
  loadInertialSensorDeviceConfigFromYAML(config_node, sensor_name, config_ptr);
  EXPECT_EQ(*config_ptr, *inertial_sensor_1_);
}

/**
 * @brief Test that if the inertial sensor config is empty, it loads the default.
 */
TEST_F(TestLoadInertialSensorDeviceConfigYAML, testEmptyConfigIsDefault) {
  auto config_node = config_yaml_["port_configuration"];
  std::string sensor_name = "inertial_sensor_2";
  auto config_ptr = std::make_shared<InertialSensorDeviceConfig>();
  EXPECT_NO_THROW(loadInertialSensorDeviceConfigFromYAML(config_node, sensor_name, config_ptr));
  EXPECT_EQ(*config_ptr, *inertial_sensor_2_);
}

/**
 * @brief Test that if the device config is missing the port attribute, it throws.
 */
TEST_F(TestLoadInertialSensorDeviceConfigYAML, testMissingConfigThrowsError) {
  auto config_node = config_yaml_["port_configuration"];
  std::string sensor_name = "inertial_sensor_no_config";
  auto config_ptr = std::make_shared<InertialSensorDeviceConfig>();
  EXPECT_THROW(
    loadInertialSensorDeviceConfigFromYAML(
      config_node, sensor_name,
      config_ptr), std::runtime_error);
}

/**
 * @brief Test that if the device config is missing the port attribute, it throws.
 */
TEST_F(TestLoadInertialSensorDeviceConfigYAML, testMissingPortThrowsError) {
  auto config_node = config_yaml_["port_configuration"];
  std::string sensor_name = "inertial_sensor_no_port";
  auto config_ptr = std::make_shared<InertialSensorDeviceConfig>();
  EXPECT_THROW(
    loadInertialSensorDeviceConfigFromYAML(
      config_node, sensor_name,
      config_ptr), std::runtime_error);
}

/**
 * @brief Test that if the device config is missing the type attribute, it throws
 */
TEST_F(TestLoadInertialSensorDeviceConfigYAML, testMissingTypeThrowsError) {
  auto config_node = config_yaml_["port_configuration"];
  std::string sensor_name = "inertial_sensor_no_type";
  auto config_ptr = std::make_shared<InertialSensorDeviceConfig>();
  EXPECT_THROW(
    loadInertialSensorDeviceConfigFromYAML(
      config_node, sensor_name,
      config_ptr), std::runtime_error);
}
