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
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "particle_filter/particle_filter.h"

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);

    // Initialize modules
    std::thread particle_filter_thread(particle_filter::Run);    


    return 0;
}