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
#include <ghost_v5_interfaces/util/load_digital_io_device_config_yaml.hpp>

using ghost_util::loadYAMLParam;
using namespace ghost_v5_interfaces::devices;
using ghost_util::packByte;

namespace ghost_v5_interfaces
{

namespace util
{

void loadDigitalIODeviceConfigFromYAML(
  YAML::Node node,
  std::shared_ptr<DigitalIODeviceConfig> sensor_device_config_ptr,
  bool verbose)
{
  // Set base attributes
  sensor_device_config_ptr->name = "digital_io";
  sensor_device_config_ptr->type = device_type_e::DIGITAL_IO;
  sensor_device_config_ptr->port = -3;

  if (!node["digital_io"]) {
    return;
  }

  auto device_node = node["digital_io"];

  std::vector<std::string> ports{"A", "B", "C", "D", "E", "F", "G", "H"};
  std::vector<bool> output_mask(8, false);
  std::vector<bool> input_mask(8, false);
  for (int i = 0; i < 8; i++) {
    auto port = ports[i];
    if (node["digital_io"][port]) {
      auto port_config = node["digital_io"][port].as<std::string>();
      if (port_config == "IN") {
        input_mask[i] = true;
      } else if (port_config == "OUT") {
        output_mask[i] = true;
      } else {
        std::cout << "[loadDigitalIODeviceConfigFromYAML] WARNING: Invalid config type at digital_io, port " << port <<
          "! Value: " << port_config << std::endl;
      }
    }
  }
  sensor_device_config_ptr->input_mask = packByte(input_mask);
  sensor_device_config_ptr->output_mask = packByte(output_mask);
}

} // namespace util

} // namespace ghost_v5_interfaces
