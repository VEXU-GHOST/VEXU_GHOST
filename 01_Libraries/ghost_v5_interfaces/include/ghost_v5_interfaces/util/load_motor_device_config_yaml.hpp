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

#pragma once

#include <iostream>
#include <memory>
#include "ghost_v5_interfaces/devices/device_interfaces.hpp"
#include "ghost_v5_interfaces/devices/motor_device_interface.hpp"
#include "yaml-cpp/yaml.h"

namespace ghost_v5_interfaces
{

namespace util
{

bool loadEncoderUnitFromYAML(YAML::Node node, devices::ghost_encoder_unit & encoder_unit_value);
bool loadGearsetFromYAML(YAML::Node node, devices::ghost_gearset & gearset_value);
bool loadBrakeModeFromYAML(YAML::Node node, devices::ghost_brake_mode & brake_mode_value);

DCMotorModel::Config loadMotorModelConfigFromYAML(YAML::Node node, bool verbose = false);
MotorController::Config loadMotorControllerConfigFromYAML(YAML::Node node, bool verbose = false);
SecondOrderLowPassFilter::Config loadLowPassFilterConfigFromYAML(
  YAML::Node node,
  bool verbose = false);
devices::MotorDeviceData::SerialConfig loadMotorSerialConfigFromYAML(
  YAML::Node node,
  bool verbose = false);

void loadMotorDeviceConfigFromYAML(
  YAML::Node node,
  std::string motor_name,
  std::shared_ptr<devices::MotorDeviceConfig> motor_device_config_ptr,
  bool verbose = false);

} // namespace util

} // namespace ghost_v5_interfaces
