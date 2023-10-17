#include <ghost_common/util/load_robot_config.hpp>

namespace ghost_common {

MotorConfigStruct loadMotorConfigStruct(YAML::Node node){
	MotorConfigStruct motor_config_struct;
	return motor_config_struct;
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