#pragma once

#include <iostream>
#include <memory>
#include "ghost_v5_interfaces/devices/device_interfaces.hpp"
#include "ghost_v5_interfaces/devices/rotation_sensor_device_interface.hpp"
#include "yaml-cpp/yaml.h"

namespace ghost_v5_interfaces {

namespace util {

void loadRotationSensorDeviceConfigFromYAML(YAML::Node node,
                                            std::string sensor_name,
                                            std::shared_ptr<RotationSensorDeviceConfig> sensor_device_config_ptr,
                                            bool verbose = false);

} // namespace util

} // namespace ghost_v5_interfaces