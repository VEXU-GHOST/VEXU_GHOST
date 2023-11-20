#include <iostream>
#include <ghost_v5_core/utils/load_config_yaml.hpp>

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

DCMotorModel::Config loadMotorModelConfigFromYAML(YAML::Node node, bool verbose){
	DCMotorModel::Config config;
	loadYAMLParam(node, "free_speed", config.free_speed, verbose);
	loadYAMLParam(node, "stall_torque", config.stall_torque, verbose);
	loadYAMLParam(node, "free_current", config.free_current, verbose);
	loadYAMLParam(node, "stall_current", config.stall_current, verbose);
	loadYAMLParam(node, "nominal_voltage", config.nominal_voltage, verbose);
	loadYAMLParam(node, "gear_ratio", config.gear_ratio, verbose);
	return config;
}
MotorController::Config loadMotorControllerConfigFromYAML(YAML::Node node, bool verbose){
	MotorController::Config config;
	loadYAMLParam(node, "pos_gain", config.pos_gain, verbose);
	loadYAMLParam(node, "vel_gain", config.vel_gain, verbose);
	loadYAMLParam(node, "ff_vel_gain", config.ff_vel_gain, verbose);
	loadYAMLParam(node, "ff_torque_gain", config.ff_torque_gain, verbose);
	return config;
}
SecondOrderLowPassFilter::Config loadLowPassFilterConfigFromYAML(YAML::Node node, bool verbose){
	SecondOrderLowPassFilter::Config config;
	loadYAMLParam(node, "cutoff_frequency", config.cutoff_frequency, verbose);
	loadYAMLParam(node, "damping_ratio", config.damping_ratio, verbose);
	loadYAMLParam(node, "timestep", config.timestep, verbose);
	return config;
}

bool loadEncoderUnitFromYAML(YAML::Node node, ghost_encoder_unit& encoder_unit_value){
	if(!node["encoder_units"]){
		return false;
	}

	auto encoder_unit = node["encoder_units"].as<std::string>();

	if(encoder_unit == "DEGREES"){
		encoder_unit_value = ghost_encoder_unit::ENCODER_DEGREES;
	}
	else if(encoder_unit == "ROTATIONS"){
		encoder_unit_value = ghost_encoder_unit::ENCODER_ROTATIONS;
	}
	else if(encoder_unit == "COUNTS"){
		encoder_unit_value = ghost_encoder_unit::ENCODER_COUNTS;
	}
	else{
		return false;
	}
	return true;
}

bool loadGearsetFromYAML(YAML::Node node, ghost_gearset& gearset_value){
	if(!node["gearset"]){
		return false;
	}

	auto gearset = node["gearset"].as<int>();

	if(gearset == 100){
		gearset_value = ghost_gearset::GEARSET_100;
	}
	else if(gearset == 200){
		gearset_value = ghost_gearset::GEARSET_200;
	}
	else if(gearset == 600){
		gearset_value = ghost_gearset::GEARSET_600;
	}
	else{
		return false;
	}

	return true;
}

bool loadBrakeModeFromYAML(YAML::Node node, ghost_brake_mode& brake_mode_value){
	if(!node["encoder_units"]){
		return false;
	}

	auto brake_mode = node["brake_mode"].as<std::string>();

	if(brake_mode == "COAST"){
		brake_mode_value = ghost_brake_mode::BRAKE_MODE_COAST;
	}
	else if(brake_mode == "BRAKE"){
		brake_mode_value = ghost_brake_mode::BRAKE_MODE_BRAKE;
	}
	else if(brake_mode == "HOLD"){
		brake_mode_value = ghost_brake_mode::BRAKE_MODE_HOLD;
	}
	else{
		return false;
	}

	return true;
}


std::shared_ptr<V5MotorConfig> loadV5MotorConfigFromYAML(YAML::Node node){
	std::shared_ptr<V5MotorConfig> config = std::make_shared<V5MotorConfig>();
	if(node["model"]){
		config->model_config = loadMotorModelConfigFromYAML(node["model"]);
	}

	if(node["controller"]){
		config->controller_config = loadMotorControllerConfigFromYAML(node["controller"]);
	}

	if(node["filter"]){
		config->filter_config = loadLowPassFilterConfigFromYAML(node["filter"]);
	}

	if(!loadEncoderUnitFromYAML(node, config->encoder_units)){
		throw std::runtime_error("[loadV5MotorInterfaceConfig] Error: Failed to load Encoder Units.");
	}

	if(!loadGearsetFromYAML(node, config->gearset)){
		throw std::runtime_error("[loadV5MotorInterfaceConfig] Error: Failed to load Gearset.");
	}

	if(!loadBrakeModeFromYAML(node, config->brake_mode)){
		throw std::runtime_error("[loadV5MotorInterfaceConfig] Error: Failed to load Brake Mode.");
	}

	return config;
}

DeviceConfigMap loadDeviceConfigMapFromYAML(YAML::Node node){
	DeviceConfigMap device_config_map;
	return device_config_map;
}

} // namespace util

} // namespace ghost_v5_core