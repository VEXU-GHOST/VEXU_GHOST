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
