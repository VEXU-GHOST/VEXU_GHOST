#pragma once

#include <ghost_common/v5_robot_config_defs.hpp>
#include "yaml-cpp/yaml.h"

using ghost_v5_config::encoder_access_helper;
using ghost_v5_config::motor_access_helper;
using ghost_v5_config::V5MotorInterfaceConfig;

namespace ghost_common {

V5MotorInterfaceConfig loadMotorConfigStruct(YAML::Node node);

std::unordered_map<std::string, motor_access_helper> loadMotorConfigurations(YAML::Node node);

std::unordered_map<std::string, encoder_access_helper> loadEncoderConfigurations(YAML::Node node);

}