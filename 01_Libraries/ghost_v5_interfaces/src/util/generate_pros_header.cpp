#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

#include <ghost_util/yaml_utils.hpp>
#include <ghost_v5_interfaces/devices/device_config_map.hpp>
#include <ghost_v5_interfaces/util/device_config_factory_utils.hpp>
#include "yaml-cpp/yaml.h"

using namespace ghost_util;
using namespace ghost_v5_interfaces::devices;
using namespace ghost_v5_interfaces::util;
using namespace ghost_v5_interfaces;

int main(int argc, char* argv[]){
	auto robots_yaml = YAML::LoadFile(std::string(getenv("HOME")) + "/VEXU_GHOST/robots.yaml");

	std::cout << "-------------------------------------------------------" << std::endl;
	std::cout << "--- Generating PROS Headers for Robot Configuration ---" << std::endl;
	std::cout << "-------------------------------------------------------" << std::endl;
	std::cout << std::endl;

	std::string selected_robot;
	if(!loadYAMLParam(robots_yaml, "selected_robot", selected_robot, false)){
		throw std::runtime_error("Error: selected_robot not found!");
	}

	std::string config_file;
	if(!loadYAMLParam(robots_yaml[selected_robot], "config_file", config_file, false)){
		throw std::runtime_error("Error: config_file not found!");
	}

	std::cout << "SELECTED_ROBOT: " << selected_robot << std::endl << std::endl;

	std::string input_filepath = std::string(getenv("HOME")) + "/VEXU_GHOST/" + config_file;
	if(!std::filesystem::exists(std::filesystem::path(input_filepath))){
		throw std::runtime_error("Error: robot configuration filepath not found! Filepath: " + input_filepath);
	}

	std::string output_filepath = std::string(getenv("HOME")) + "/VEXU_GHOST/02_V5/ghost_pros/include/robot_config.hpp";

	std::cout << "INPUT FILEPATH: " << input_filepath << std::endl;
	std::cout << "OUTPUT FILEPATH: " << output_filepath << std::endl << std::endl;

	// Create directory for code generation
	if(std::filesystem::exists(std::filesystem::path(output_filepath))){
		std::filesystem::remove_all(std::filesystem::path(output_filepath));
	}

	std::shared_ptr<DeviceConfigMap> robot_config_ptr;
	try{
		auto input_yaml = YAML::LoadFile(input_filepath);
		robot_config_ptr = loadRobotConfigFromYAML(input_yaml);
	}
	catch(std::runtime_error &e){
		std::cout << "[GenerateProsHeader] Error: Failed to load Robot Config from Input YAML." << std::endl;
		throw e;
	}

	try{
		generateCodeFromRobotConfig(robot_config_ptr, output_filepath);
	}
	catch(std::runtime_error &e){
		std::cout << "[GenerateProsHeader] Error: Failed to generate code from Robot Config." << std::endl;
		throw e;
	}

	std::cout << "Code successfully generated at " << output_filepath << std::endl;

	// Generate code
	return 0;
}