#pragma once

#include <memory>
#include "ghost_v5_core/devices/v5_device_interface.hpp"
#include "ghost_v5_core/devices/v5_motor_interface.hpp"
#include "yaml-cpp/yaml.h"

namespace ghost_v5_core {

namespace util {

DCMotorModel::Config loadMotorModelConfigFromYAML(YAML::Node node, bool verbose = false);
MotorController::Config loadMotorControllerConfigFromYAML(YAML::Node node, bool verbose = false);
SecondOrderLowPassFilter::Config loadLowPassFilterConfigFromYAML(YAML::Node node, bool verbose = false);

bool loadEncoderUnitFromYAML(YAML::Node node, ghost_encoder_unit& encoder_unit_value);
bool loadGearsetFromYAML(YAML::Node node, ghost_gearset& gearset_value);
bool loadBrakeModeFromYAML(YAML::Node node, ghost_brake_mode& brake_mode_value);

std::shared_ptr<V5MotorConfig> loadV5MotorConfigFromYAML(YAML::Node node);

DeviceInterfaceMap loadDeviceInterfaceMapFromYAML(YAML::Node node, bool verbose = false);

} // namespace util

} // namespace ghost_v5_core