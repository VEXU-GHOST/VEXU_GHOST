#pragma once

#include <iostream>
#include <memory>
#include "../base/device_interfaces.hpp"
#include "rotation_sensor_device_config.hpp"
#include "yaml-cpp/yaml.h"

namespace ghost_v5_core {

namespace util {

void loadRotationSensorDeviceConfigFromYAML(YAML::Node node,
                                            std::string sensor_name,
                                            std::shared_ptr<RotationSensorDeviceConfig> sensor_device_config_ptr,
                                            bool verbose = false);

} // namespace util

} // namespace ghost_v5_core