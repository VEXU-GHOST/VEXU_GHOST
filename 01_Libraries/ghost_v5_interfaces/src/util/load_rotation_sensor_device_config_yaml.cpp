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

#include <ghost_util/yaml_utils.hpp>
#include <ghost_v5_interfaces/util/load_rotation_sensor_device_config_yaml.hpp>

using ghost_util::loadYAMLParam;
using namespace ghost_v5_interfaces::devices;
namespace ghost_v5_interfaces
{

namespace util
{

RotationSensorDeviceData::SerialConfig loadRotationSensorSerialConfigFromYAML(
  YAML::Node node,
  bool verbose)
{
  RotationSensorDeviceData::SerialConfig config;
  loadYAMLParam(node, "send_angle_data", config.send_angle_data, verbose);
  loadYAMLParam(node, "send_position_data", config.send_position_data, verbose);
  loadYAMLParam(node, "send_velocity_data", config.send_velocity_data, verbose);
  return config;
}

void loadRotationSensorDeviceConfigFromYAML(
  YAML::Node node,
  std::string sensor_name,
  std::shared_ptr<RotationSensorDeviceConfig> sensor_device_config_ptr,
  bool verbose)
{
  // Get device base config
  auto device_node = node["devices"][sensor_name];
  if (!device_node) {
    throw std::runtime_error(
            "[loadRotationSensorDeviceConfigFromYAML] Error: Rotation Sensor " + sensor_name +
            " not found!");
  }

  // Get name of sensor-specific config
  if (!device_node["config"]) {
    throw std::runtime_error(
            "[loadRotationSensorDeviceConfigFromYAML] Error: Rotation Sensor Config not found for sensor " + sensor_name +
            "!");
  }
  auto config_name = device_node["config"].as<std::string>();

  // Retrieve sensor-specific config node
  auto config_node = node["device_configurations"][config_name];
  if (!config_node) {
    throw std::runtime_error(
            "[loadRotationSensorDeviceConfigFromYAML] Error: Config not found for name " + config_name +
            "!");
  }

  loadYAMLParam(config_node, "data_rate", sensor_device_config_ptr->data_rate, verbose);
  if (config_node["serial"]) {
    sensor_device_config_ptr->serial_config =
      loadRotationSensorSerialConfigFromYAML(config_node["serial"], verbose);
  }

  // Set base attributes
  sensor_device_config_ptr->name = sensor_name;
  sensor_device_config_ptr->type = device_type_e::ROTATION_SENSOR;

  if (device_node["type"].as<std::string>() != "ROTATION_SENSOR") {
    throw std::runtime_error(
            "[loadRotationSensorDeviceConfigFromYAML] Error: Device type is incorrect for Rotation Sensor " + sensor_name +
            "!");
  }

  if (!loadYAMLParam(device_node, "port", sensor_device_config_ptr->port, verbose)) {
    throw std::runtime_error(
            "[loadRotationSensorDeviceConfigFromYAML] Error: Port not specified for Rotation Sensor " + sensor_name +
            "!");
  }

  // Set Sensor specific attributes
  loadYAMLParam(device_node, "reversed", sensor_device_config_ptr->reversed, verbose);
}

} // namespace util

} // namespace ghost_v5_interfaces
