#pragma once

#include <iostream>
#include <memory>
#include "ghost_v5_core/devices/v5_device_interface.hpp"
#include "ghost_v5_core/devices/v5_motor_interface.hpp"
#include "yaml-cpp/yaml.h"

namespace ghost_v5_core {

namespace util {

template <typename T>
bool loadYAMLParam(YAML::Node node, std::string param_name, T& var, bool verbose){
	if(node[param_name]){
		var = node[param_name].as<T>();
		if(verbose){
			std::cout << "Loaded <" << param_name << "> with value " << var << std::endl;
		}
		return true;
	}
	else if(verbose){
		std::cout << "Failed to Load <" + param_name << ">" << std::endl;
	}
	return false;
}

bool loadEncoderUnitFromYAML(YAML::Node node, ghost_encoder_unit& encoder_unit_value);
bool loadGearsetFromYAML(YAML::Node node, ghost_gearset& gearset_value);
bool loadBrakeModeFromYAML(YAML::Node node, ghost_brake_mode& brake_mode_value);

DCMotorModel::Config loadMotorModelConfigFromYAML(YAML::Node node, bool verbose = false);
MotorController::Config loadMotorControllerConfigFromYAML(YAML::Node node, bool verbose = false);
SecondOrderLowPassFilter::Config loadLowPassFilterConfigFromYAML(YAML::Node node, bool verbose = false);

std::shared_ptr<V5MotorConfig> loadV5MotorConfigFromYAML(YAML::Node node, bool verbose = false);

DeviceInterfaceMap loadDeviceInterfaceMapFromYAML(YAML::Node node, bool verbose = false);

} // namespace util

} // namespace ghost_v5_core