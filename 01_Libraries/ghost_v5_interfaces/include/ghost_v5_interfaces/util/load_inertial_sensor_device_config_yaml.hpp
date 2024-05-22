#pragma once

#include <iostream>
#include <memory>
#include "ghost_v5_interfaces/devices/device_interfaces.hpp"
#include "ghost_v5_interfaces/devices/inertial_sensor_device_interface.hpp"
#include "yaml-cpp/yaml.h"

namespace ghost_v5_interfaces
{

namespace util
{

devices::InertialSensorDeviceData::SerialConfig loadInertialSensorSerialConfigFromYAML(
  YAML::Node node, bool verbose = false);

void loadInertialSensorDeviceConfigFromYAML(
  YAML::Node node,
  std::string sensor_name,
  std::shared_ptr<devices::InertialSensorDeviceConfig> sensor_device_config_ptr,
  bool verbose = false);

} // namespace util

} // namespace ghost_v5_interfaces
