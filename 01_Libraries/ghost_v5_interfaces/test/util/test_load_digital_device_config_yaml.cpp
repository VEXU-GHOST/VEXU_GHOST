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


#include <ghost_v5_interfaces/util/load_digital_device_config_yaml.hpp>
#include "gtest/gtest.h"
#include "yaml-cpp/yaml.h"

using namespace ghost_v5_interfaces::devices;
using namespace ghost_v5_interfaces::util;
using namespace ghost_v5_interfaces;

class TestLoadDigitalDeviceConfigYAML : public ::testing::Test
{
protected:
  void SetUp() override
  {
    std::string config_path = std::string(getenv("VEXU_HOME")) +
      "/01_Libraries/ghost_v5_interfaces/test/config/test_load_digital_device_config.yaml";
    config_yaml_ = YAML::LoadFile(config_path);

    // Expected motor configurations
    digital_input_config_ = std::make_shared<DigitalDeviceConfig>();
    digital_input_config_->port = 22;
    digital_input_config_->name = "in";
    digital_input_config_->type = device_type_e::DIGITAL_INPUT;

    digital_output_config_ = std::make_shared<DigitalDeviceConfig>();
    digital_output_config_->port = 23;
    digital_output_config_->name = "out";
    digital_output_config_->type = device_type_e::DIGITAL_OUTPUT;

  }

  std::shared_ptr<DigitalDeviceConfig> digital_input_config_;
  std::shared_ptr<DigitalDeviceConfig> digital_output_config_;

  YAML::Node config_yaml_;
};

TEST_F(TestLoadDigitalDeviceConfigYAML, testLoadDigitalIn) {
    std::shared_ptr<DigitalDeviceConfig> input_ptr = std::make_shared<DigitalDeviceConfig>();
    loadDigitalDeviceConfigFromYAML(config_yaml_["port_configuration"], "in", input_ptr, false);
    EXPECT_EQ(*input_ptr, *digital_input_config_);
}

TEST_F(TestLoadDigitalDeviceConfigYAML, testLoadDigitalOut) {
    std::shared_ptr<DigitalDeviceConfig> output_ptr = std::make_shared<DigitalDeviceConfig>();
    loadDigitalDeviceConfigFromYAML(config_yaml_["port_configuration"], "out", output_ptr, false);
    EXPECT_EQ(*output_ptr, *digital_output_config_);
}