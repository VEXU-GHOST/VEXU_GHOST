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

#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include <ghost_util/yaml_utils.hpp>
#include <ghost_v5_interfaces/devices/device_config_map.hpp>
#include <ghost_v5_interfaces/util/device_config_factory_utils.hpp>
#include "yaml-cpp/yaml.h"

using namespace ghost_util;
using namespace ghost_v5_interfaces::devices;
using namespace ghost_v5_interfaces::util;
using namespace ghost_v5_interfaces;

int main(int argc, char * argv[])
{
  auto robots_yaml = YAML::LoadFile(std::string(getenv("VEXU_HOME")) + "/robots.yaml");

  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << "--- Generating PROS Headers for Robot Configuration ---" << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << std::endl;

  std::string selected_robot;
  if (!loadYAMLParam(robots_yaml, "selected_robot", selected_robot, false)) {
    throw std::runtime_error("Error: selected_robot not found!");
  }

  std::string config_file;
  if (!loadYAMLParam(robots_yaml[selected_robot], "config_file", config_file, false)) {
    throw std::runtime_error("Error: config_file not found!");
  }

  std::cout << "SELECTED_ROBOT: " << selected_robot << std::endl << std::endl;

  std::string input_filepath = std::string(getenv("VEXU_HOME")) + "/" + config_file;
  if (!std::filesystem::exists(std::filesystem::path(input_filepath))) {
    throw std::runtime_error(
            "Error: robot configuration filepath not found! Filepath: " + input_filepath);
  }

  std::string output_filepath = std::string(getenv("VEXU_HOME")) +
    "/02_V5/ghost_pros/include/robot_config.hpp";

  std::cout << "INPUT FILEPATH: " << input_filepath << std::endl;
  std::cout << "OUTPUT FILEPATH: " << output_filepath << std::endl << std::endl;

  // Create directory for code generation
  if (std::filesystem::exists(std::filesystem::path(output_filepath))) {
    std::filesystem::remove_all(std::filesystem::path(output_filepath));
  }

  std::shared_ptr<DeviceConfigMap> robot_config_ptr;
  YAML::Node input_yaml;
  try {
    input_yaml = YAML::LoadFile(input_filepath);
    robot_config_ptr = loadRobotConfigFromYAML(input_yaml);
  } catch (std::runtime_error & e) {
    std::cout << "[GenerateProsHeader] Error: Failed to load Robot Config from Input YAML." <<
      std::endl;
    throw e;
  }

  try {
    generateCodeFromRobotConfig(robot_config_ptr, output_filepath);
  } catch (std::runtime_error & e) {
    std::cout << "[GenerateProsHeader] Error: Failed to generate code from Robot Config." <<
      std::endl;
    throw e;
  }

  // Append the inter-robot VEXlink configuration. The radio link is infrastructure, not a port
  // device, so rather than routing it through the DeviceConfigMap it is read straight from the YAML
  // here and emitted as plain constants. A missing "inter_robot_link" block (or port 0) leaves the
  // relay disabled.
  try {
    int link_port = 0;
    std::string link_id = "";
    bool link_is_transmitter = true;
    if (input_yaml["inter_robot_link"]) {
      auto link_yaml = input_yaml["inter_robot_link"];
      loadYAMLParam(link_yaml, "port", link_port, false);
      loadYAMLParam(link_yaml, "link_id", link_id, false);
      loadYAMLParam(link_yaml, "is_transmitter", link_is_transmitter, false);
    }

    std::ofstream out(output_filepath, std::ios::app);
    out << "\n// Inter-Robot Comms (VEXlink) configuration. Port 0 disables the relay.\n";
    out << "constexpr int INTER_ROBOT_LINK_PORT = " << link_port << ";\n";
    out << "constexpr char INTER_ROBOT_LINK_ID[] = \"" << link_id << "\";\n";
    out << "constexpr bool INTER_ROBOT_LINK_IS_TRANSMITTER = " <<
      (link_is_transmitter ? "true" : "false") << ";\n";
    std::cout << "INTER_ROBOT_LINK_PORT: " << link_port << std::endl;
  } catch (std::exception & e) {
    std::cout << "[GenerateProsHeader] Error: Failed to append inter-robot link config." <<
      std::endl;
    throw;
  }

  std::cout << "Code successfully generated at " << output_filepath << std::endl;

  // Generate code
  return 0;
}
