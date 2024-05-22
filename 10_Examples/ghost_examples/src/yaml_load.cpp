/*
 * Filename: yaml_load.cpp
 * Created Date: Wednesday October 26th 2022
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 *
 * Last Modified: Monday October 24th 2022 2:43:40 pm
 * Modified By: Maxx Wilson
 */

#include <iostream>
#include "yaml-cpp/yaml.h"

int main(int argc, char* argv[]){
	std::string config_path = std::string(getenv("VEXU_HOME")) + "/10_Examples/ghost_examples/config/example.yaml";
	YAML::Node config = YAML::LoadFile(config_path);
	const std::string string_val = config["ns1"]["ns2"]["string_val"].as<std::string>();
	const int an_integer = config["ns1"]["ns2"]["an_int"].as<int>();
	const double some_number = config["ns1"]["ns2"]["a_double"].as<double>();

	std::cout << "Example String: " << string_val << std::endl;
	std::cout << "Example Integer: " << an_integer << std::endl;
	std::cout << "Example Double: " << some_number << std::endl;

	std::cout << "Change example.yaml and run again!" << std::endl;

	return 0;
}