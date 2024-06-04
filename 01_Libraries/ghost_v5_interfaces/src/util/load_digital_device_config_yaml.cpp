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
  // Get ADI node
  auto adi_node = node["ADI"];
  if (!adi_node || !adi_node.IsMap()) {
    //TODO(xander): set digital_io_config_ptr to default values and return
  }

  for (const auto &device_node : adi_node) {
    //set ports and is_actuator mask
  }

  // Set base attributes
  digital_io_config_ptr->type = device_type_e::DIGITAL_IO;


  // example of param loading process
  if (!loadYAMLParam(adi_node, "port", digital_io_config_ptr->port, verbose)) {
    throw std::runtime_error(
            "[loadDigitalDeviceConfigFromYAML] Error: Port not specified for Rotation Sensor " + sensor_name +
            "!");
  }
}

} // namespace util

} // namespace ghost_v5_interfaces
