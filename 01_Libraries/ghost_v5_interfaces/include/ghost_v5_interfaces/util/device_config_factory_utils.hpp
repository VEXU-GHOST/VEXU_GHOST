#pragma once

#include <memory>
#include <ghost_v5_interfaces/devices/device_config_map.hpp>
#include "yaml-cpp/yaml.h"

namespace ghost_v5_interfaces {

namespace util {

// Maps device types from string to enum
const std::unordered_map<std::string, devices::device_type_e> STRING_TO_DEVICE_TYPE_MAP{
	{"MOTOR",           devices::device_type_e::MOTOR},
	{"ROTATION_SENSOR", devices::device_type_e::ROTATION_SENSOR},
	{"JOYSTICK",        devices::device_type_e::JOYSTICK},
	{"INERTIAL_SENSOR", devices::device_type_e::INERTIAL_SENSOR},
	{"DISTANCE_SENSOR", devices::device_type_e::DISTANCE_SENSOR},
	{"OPTICAL_SENSOR",  devices::device_type_e::OPTICAL_SENSOR},
	{"VISION_SENSOR",   devices::device_type_e::VISION_SENSOR},
	{"GPS_SENSOR",      devices::device_type_e::GPS_SENSOR},
	{"RADIO",           devices::device_type_e::RADIO},
	{"INVALID",         devices::device_type_e::INVALID},
};

const std::unordered_map<devices::device_type_e, std::string> DEVICE_TYPE_TO_STRING_MAP{
	{devices::device_type_e::MOTOR,                 "MOTOR"},
	{devices::device_type_e::ROTATION_SENSOR,       "ROTATION_SENSOR"},
	{devices::device_type_e::JOYSTICK,              "JOYSTICK"},
	{devices::device_type_e::INERTIAL_SENSOR,       "INERTIAL_SENSOR"},
	{devices::device_type_e::DISTANCE_SENSOR,       "DISTANCE_SENSOR"},
	{devices::device_type_e::OPTICAL_SENSOR,        "OPTICAL_SENSOR"},
	{devices::device_type_e::VISION_SENSOR,         "VISION_SENSOR"},
	{devices::device_type_e::GPS_SENSOR,            "GPS_SENSOR"},
	{devices::device_type_e::RADIO,                 "RADIO"},
	{devices::device_type_e::INVALID,               "INVALID"}
};

/**
 * @brief Given a YAML node of appropriate format, loads a DeviceConfigMap which represents a given V5 Robot's
 * hardware configuration.
 *
 * YAML files must have the following form.
 *
 * port_configuration:
 * 		use_partner_joystick = false/true
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
std::shared_ptr<devices::DeviceConfigMap> loadRobotConfigFromYAML(YAML::Node node, bool verbose = false);

/**
 * @brief Helper function which calls loadRobotConfigFromYAML after opening YAML from filepath.
 *
 * @param filepath
 * @param verbose
 * @return std::shared_ptr<devices::DeviceConfigMap>
 */
std::shared_ptr<devices::DeviceConfigMap> loadRobotConfigFromYAMLFile(std::string filepath, bool verbose = false);


/**
 * @brief Given a DeviceConfigMap, generate C++ source code which reconstructs the DeviceConfigMap at compile-time.
 * This allows for loading data on to the V5 Brain via a runtime configurable format (i.e. YAML) on the coprocessor.
 *
 * @param config
 * @param output_filepath
 */
void generateCodeFromRobotConfig(std::shared_ptr<devices::DeviceConfigMap> config_ptr, std::string output_filepath);

} // namespace util

} // namespace ghost_v5_interfaces