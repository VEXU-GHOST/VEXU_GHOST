#pragma once

#include <iostream>
#include <memory>
#include "ghost_v5_interfaces/devices/device_interfaces.hpp"
#include "ghost_v5_interfaces/devices/motor_device_interface.hpp"
#include "yaml-cpp/yaml.h"

namespace ghost_v5_interfaces
{

namespace util
{

bool loadEncoderUnitFromYAML(YAML::Node node, devices::ghost_encoder_unit & encoder_unit_value);
bool loadGearsetFromYAML(YAML::Node node, devices::ghost_gearset & gearset_value);
bool loadBrakeModeFromYAML(YAML::Node node, devices::ghost_brake_mode & brake_mode_value);

DCMotorModel::Config loadMotorModelConfigFromYAML(YAML::Node node, bool verbose = false);
MotorController::Config loadMotorControllerConfigFromYAML(YAML::Node node, bool verbose = false);
SecondOrderLowPassFilter::Config loadLowPassFilterConfigFromYAML(
  YAML::Node node,
  bool verbose = false);
devices::MotorDeviceData::SerialConfig loadMotorSerialConfigFromYAML(
  YAML::Node node,
  bool verbose = false);

void loadMotorDeviceConfigFromYAML(
  YAML::Node node,
  std::string motor_name,
  std::shared_ptr<devices::MotorDeviceConfig> motor_device_config_ptr,
  bool verbose = false);

} // namespace util

} // namespace ghost_v5_interfaces
