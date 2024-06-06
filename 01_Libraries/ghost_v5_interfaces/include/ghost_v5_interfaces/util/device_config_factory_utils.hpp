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

#include <memory>
#include <ghost_v5_interfaces/devices/device_config_map.hpp>
#include <ghost_v5_interfaces/util/device_type_helpers.hpp>
#include "yaml-cpp/yaml.h"

namespace ghost_v5_interfaces
{

namespace util
{

/**
 * @brief Given a YAML node of appropriate format, loads a DeviceConfigMap which represents a given V5 Robot's
 * hardware configuration.
 *
 * YAML files must have the following form.
 *
 * port_configuration:
 *              use_partner_joystick = false/true
 *      adi:
 *          limit_switch_1:
 *              port: A
 *              type: DIGITAL
 *              io_type: SENSOR
 *      devices:
 *          my_motor_name_here:
 *              port: 1
 *              type: MOTOR
 *              reversed: true/false
 *              config: my_motor_config_name
 *          rotation_sensor_1:
 *              port: 2
 *              type: ROTATION_SENSOR
 *              reversed: true/false
 *              data_rate: 5
 *          ...
 *      device_configurations:
 *          my_motor_config_name:
 *              ...
 *              filter:
 *                  ...
 *              controller:
 *                  ...
 *              model:
 *                  ...
 *
 * @param node
 * @param verbose
 * @return devices::DeviceConfigMap
 */
std::shared_ptr<devices::DeviceConfigMap> loadRobotConfigFromYAML(
  YAML::Node node,
  bool verbose = false);

/**
 * @brief Helper function which calls loadRobotConfigFromYAML after opening YAML from filepath.
 *
 * @param filepath
 * @param verbose
 * @return std::shared_ptr<devices::DeviceConfigMap>
 */
std::shared_ptr<devices::DeviceConfigMap> loadRobotConfigFromYAMLFile(
  std::string filepath,
  bool verbose = false);


/**
 * @brief Given a DeviceConfigMap, generate C++ source code which reconstructs the DeviceConfigMap at compile-time.
 * This allows for loading data on to the V5 Brain via a runtime configurable format (i.e. YAML) on the coprocessor.
 *
 * @param config
 * @param output_filepath
 */
void generateCodeFromRobotConfig(
  std::shared_ptr<devices::DeviceConfigMap> config_ptr,
  std::string output_filepath);

} // namespace util

} // namespace ghost_v5_interfaces
