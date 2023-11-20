#include <iostream>
#include <ghost_v5_core/util/load_robot_config.hpp>

namespace ghost_v5_core {

namespace util {

template <typename T>
void loadYAMLParam(YAML::Node node, std::string param_name, T& var, bool verbose){
	if(node[param_name]){
		var = node[param_name].as<T>();
		if(verbose){
			std::cout << "Loaded <" << param_name << "> with value " << var << std::endl;
		}
	}
	else if(verbose){
		std::cout << "Failed to Load <" + param_name << ">" << std::endl;
	}
}

MotorModelConfig loadMotorModelConfigFromYAML(YAML::Node node){
	MotorModelConfig config;
	loadYAMLParam(node, "nominal_free_speed", config.nominal_free_speed, true);
	loadYAMLParam(node, "stall_torque", config.stall_torque, true);
	loadYAMLParam(node, "free_current", config.free_current, true);
	loadYAMLParam(node, "stall_current", config.stall_current, true);
	loadYAMLParam(node, "max_voltage", config.max_voltage, true);
	return config;
}
MotorControllerConfig loadMotorControllerConfigFromYAML(YAML::Node node){
	MotorControllerConfig config;
	loadYAMLParam(node, "pos_gain", config.pos_gain, true);
	loadYAMLParam(node, "vel_gain", config.vel_gain, true);
	loadYAMLParam(node, "ff_vel_gain", config.ff_vel_gain, true);
	loadYAMLParam(node, "ff_torque_gain", config.ff_torque_gain, true);
	return config;
}
LowPassFilterConfig loadLowPassFilterConfigFromYAML(YAML::Node node){
	LowPassFilterConfig config;
	loadYAMLParam(node, "cutoff_frequency", config.cutoff_frequency, true);
	loadYAMLParam(node, "damping_ratio", config.damping_ratio, true);
	loadYAMLParam(node, "timestep", config.timestep, true);
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


V5MotorInterfaceConfig loadV5MotorInterfaceConfigFromYAML(YAML::Node node){
	V5MotorInterfaceConfig config{};

	if(node["model"]){
		config.model = loadMotorModelConfigFromYAML(node["model"]);
	}

	if(node["controller"]){
		config.controller = loadMotorControllerConfigFromYAML(node["controller"]);
	}

	if(node["filter"]){
		config.filter = loadLowPassFilterConfigFromYAML(node["filter"]);
	}

	if(!loadEncoderUnitFromYAML(node, config.encoder_units)){
		throw std::runtime_error("[loadV5MotorInterfaceConfig] Error: Failed to load Encoder Units.");
	}

	if(!loadGearsetFromYAML(node, config.gearset)){
		throw std::runtime_error("[loadV5MotorInterfaceConfig] Error: Failed to load Gearset.");
	}

	if(!loadBrakeModeFromYAML(node, config.brake_mode)){
		throw std::runtime_error("[loadV5MotorInterfaceConfig] Error: Failed to load Brake Mode.");
	}

	return config;
}

std::unordered_map<std::string, motor_access_helper> loadMotorConfigurations(YAML::Node node){
	std::unordered_map<std::string, motor_access_helper> motor_config_map;
	return motor_config_map;
}

std::unordered_map<std::string, encoder_access_helper> loadEncoderConfigurations(YAML::Node node){
	std::unordered_map<std::string, encoder_access_helper> encoder_config_map;
	return encoder_config_map;
}

} // namespace util

} // namespace ghost_v5_core