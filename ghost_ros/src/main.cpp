/*
 * Filename: main.cpp
 * Created Date: Monday October 24th 2022
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 * 
 * Last Modified: Monday October 24th 2022 2:43:40 pm
 * Modified By: Maxx Wilson
 */

#include <iostream>
#include "yaml-cpp/yaml.h"

int main(int argc, char* argv[]){

    YAML::Node config = YAML::LoadFile("config/config.yaml");

    const std::string username = config["test"]["test2"].as<std::string>();
    std::cout << username << std::endl;

    return 0;
}