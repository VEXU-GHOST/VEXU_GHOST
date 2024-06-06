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

#include <ghost_util/yaml_utils.hpp>
#include <ghost_v5_interfaces/util/load_digital_device_config_yaml.hpp>

using ghost_util::loadYAMLParam;
using namespace ghost_v5_interfaces::devices;
namespace ghost_v5_interfaces
{

namespace util
{

void loadDigitalDeviceConfigFromYAML(
  YAML::Node node,
  std::string device_name,
  std::shared_ptr<DigitalDeviceConfig> device_config_ptr,
  bool verbose)
{
  // Get device node
  auto device_node = node["adi"][device_name];
  if (!device_node) {
    throw std::runtime_error(
            "[loadDigitalDeviceConfigFromYAML] Error: Device Sensor " + device_name +
            " not found!");
  }

  // Set base attributes
  device_config_ptr->name = device_name;
  device_config_ptr->type = device_type_e::DIGITAL;

  // Get port
  char port;
  if (!loadYAMLParam(device_node, "port", port, verbose)) {
    throw std::runtime_error(
            "[loadDigitalDeviceConfigFromYAML] Error: Port not specified for Digital Device " +
            device_name + "!");
  }

  // Validate port
  const std::vector<char> valid_ports = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'};
  if (std::count(valid_ports.begin(), valid_ports.end(), port) == 0) {
    throw std::runtime_error(
            "[loadDigitalDeviceConfigFromYAML] Error: Invalid port specified for Digital Device " +
            device_name + "!");
  }

  // Convert port from A-H to 22-28
  port += 22 - 'A';

  // Set port in device config
  device_config_ptr->port = (int) port;

  // Load IO type
  std::string type;
  if (!loadYAMLParam(device_node, "io_type", type, verbose)) {
    throw std::runtime_error(
            "[loadDigitalDeviceConfigFromYAML] Error: Type not specified for Digital Device " +
            device_name + "!");
  }

  // Validate IO type
  if (type != "SENSOR" && type != "ACTUATOR") {
    throw std::runtime_error(
            "[loadDigitalDeviceConfigFromYAML] Error: Invalid type specified for Digital Device " +
            device_name + "!");
  }

  // Set IO type in device config
  device_config_ptr->serial_config.io_type = (type == "SENSOR") ? SENSOR : ACTUATOR;
}

} // namespace util

} // namespace ghost_v5_interfaces
