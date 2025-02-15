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

#include <ghost_v5_interfaces/util/load_digital_io_device_config_yaml.hpp>
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
      "/01_Libraries/ghost_v5_interfaces/test/config/test_load_digital_io_config.yaml";
    config_yaml_ = YAML::LoadFile(config_path);


    // Changed every param
    digital_io_config = std::make_shared<DigitalIODeviceConfig>();
    digital_io_config->port = -3;
    digital_io_config->name = "digital_io";
    digital_io_config->type = device_type_e::DIGITAL_IO;
    digital_io_config->input_mask = packByte(std::vector<bool>{true, false, false, false, false, false, false, true});
    digital_io_config->output_mask = packByte(std::vector<bool>{false, true, false, false, false, false, false, false});
  }

  std::shared_ptr<DigitalIODeviceConfig> digital_io_config;

  YAML::Node config_yaml_;
};

TEST_F(TestLoadInertialSensorDeviceConfigYAML, testLoadSuccessful) {

  std::shared_ptr<DigitalIODeviceConfig> loaded_config = std::make_shared<DigitalIODeviceConfig>();
  EXPECT_NO_THROW(loadDigitalIODeviceConfigFromYAML(config_yaml_["port_configuration"], loaded_config));

  EXPECT_EQ(*loaded_config, *digital_io_config);
}

TEST_F(TestLoadInertialSensorDeviceConfigYAML, testLoadEmptyYAMLNodeIsDefault) {
  std::shared_ptr<DigitalIODeviceConfig> default_config = std::make_shared<DigitalIODeviceConfig>();
  default_config->name = "digital_io";
  default_config->type = device_type_e::DIGITAL_IO;
  default_config->port = -3;

  std::shared_ptr<DigitalIODeviceConfig> loaded_config = std::make_shared<DigitalIODeviceConfig>();
  EXPECT_NO_THROW(loadDigitalIODeviceConfigFromYAML(config_yaml_, loaded_config));

  EXPECT_EQ(*loaded_config, *default_config);

}
