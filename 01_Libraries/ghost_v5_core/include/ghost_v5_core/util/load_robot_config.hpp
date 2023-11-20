#pragma once

#include "yaml-cpp/yaml.h"

namespace ghost_v5_core {

namespace util {

MotorModelConfig loadMotorModelConfigFromYAML(YAML::Node node);
MotorControllerConfig loadMotorControllerConfigFromYAML(YAML::Node node);
LowPassFilterConfig loadLowPassFilterConfigFromYAML(YAML::Node node);

bool loadEncoderUnitFromYAML(YAML::Node node, ghost_encoder_unit& encoder_unit_value);
bool loadGearsetFromYAML(YAML::Node node, ghost_gearset& gearset_value);
bool loadBrakeModeFromYAML(YAML::Node node, ghost_brake_mode& brake_mode_value);

V5MotorInterfaceConfig loadV5MotorInterfaceConfigFromYAML(YAML::Node node);

std::unordered_map<std::string, motor_access_helper> loadMotorConfigurations(YAML::Node node);
std::unordered_map<std::string, encoder_access_helper> loadEncoderConfigurations(YAML::Node node);

} // namespace util

} // namespace ghost_v5_core