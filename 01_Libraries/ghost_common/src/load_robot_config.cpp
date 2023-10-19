#include <iostream>
#include <ghost_common/util/load_robot_config.hpp>

namespace ghost_common {

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

MotorConfigStruct loadMotorConfigStruct(YAML::Node node){
	MotorConfigStruct motor_config{};

	std::cout << "---------------------" << std::endl;
	if(node["motor"]){
		loadYAMLParam(node["motor"], "nominal_free_speed", motor_config.motor__nominal_free_speed, true);
		loadYAMLParam(node["motor"], "stall_torque", motor_config.motor__stall_torque, true);
		loadYAMLParam(node["motor"], "free_current", motor_config.motor__free_current, true);
		loadYAMLParam(node["motor"], "stall_current", motor_config.motor__stall_current, true);
		loadYAMLParam(node["motor"], "max_voltage", motor_config.motor__max_voltage, true);


		if(node["encoder_units"]){
			auto unit = node["encoder_units"].as<std::string>();
			if(unit == "DEGREES"){
				motor_config.motor__encoder_units = ghost_v5_config::ghost_encoder_unit::ENCODER_DEGREES;
			}
			else if(unit == "ROTATIONS"){
				motor_config.motor__encoder_units = ghost_v5_config::ghost_encoder_unit::ENCODER_ROTATIONS;
			}
			else if(unit == "COUNTS"){
				motor_config.motor__encoder_units = ghost_v5_config::ghost_encoder_unit::ENCODER_COUNTS;
			}
			else{
				throw std::runtime_error("[loadMotorConfigStruct] Error: " + unit + " is not a valid encoder unit.");
			}
		}

		if(node["gear_ratio"]){
			auto gear_ratio = node["gear_ratio"].as<int>();
			if(gear_ratio == 100){
				motor_config.motor__gear_ratio = ghost_v5_config::ghost_gearset::GEARSET_100;
			}
			else if(gear_ratio == 200){
				motor_config.motor__gear_ratio = ghost_v5_config::ghost_gearset::GEARSET_200;
			}
			else if(gear_ratio == 600){
				motor_config.motor__gear_ratio = ghost_v5_config::ghost_gearset::GEARSET_600;
			}
			else{
				throw std::runtime_error("[loadMotorConfigStruct] Error: " + gear_ratio + " is not a valid gearset unit.");
			}
		}

		if(node[]["brake_mode"]){
			auto brake_mode = node["brake_mode"].as<std::string>();
			if(brake_mode == "COAST"){
				motor_config.motor__brake_mode = ghost_v5_config::ghost_brake_mode::BRAKE_MODE_COAST;
			}
			else if(brake_mode == "BRAKE"){
				motor_config.motor__brake_mode = ghost_v5_config::ghost_brake_mode::BRAKE_MODE_BRAKE;
			}
			else if(brake_mode == "HOLD"){
				motor_config.motor__brake_mode = ghost_v5_config::ghost_brake_mode::BRAKE_MODE_HOLD;
			}
			else{
				throw std::runtime_error("[loadMotorConfigStruct] Error: " + brake_mode + " is not a valid brake mode.");
			}
		}
	}

	if(node["controller"]){
		loadYAMLParam(node["controller"], "pos_gain", motor_config.ctl__pos_gain, true);
		loadYAMLParam(node["controller"], "vel_gain", motor_config.ctl__vel_gain, true);
		loadYAMLParam(node["controller"], "ff_vel_gain", motor_config.ctl__ff_vel_gain, true);
		loadYAMLParam(node["controller"], "ff_torque_gain", motor_config.ctl__ff_torque_gain, true);
	}

	if(node["filter"]){
		loadYAMLParam(node["filter"], "cutoff_frequency", motor_config.filter__cutoff_frequency, true);
		loadYAMLParam(node["filter"], "damping_ratio", motor_config.filter__damping_ratio, true);
		loadYAMLParam(node["filter"], "timestep", motor_config.filter__timestep, true);
	}

	return motor_config;
}

std::unordered_map<std::string, motor_access_helper> loadMotorConfigurations(YAML::Node node){
	std::unordered_map<std::string, motor_access_helper> motor_config_map;
	return motor_config_map;
}

std::unordered_map<std::string, encoder_access_helper> loadEncoderConfigurations(YAML::Node node){
	std::unordered_map<std::string, encoder_access_helper> encoder_config_map;
	return encoder_config_map;
}

}