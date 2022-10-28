/*
 * Filename: main.cpp
 * Created Date: Monday October 24th 2022
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 * 
 * Last Modified: Thursday October 27th 2022 7:40:53 pm
 * Modified By: Maxx Wilson
 */

#include <iostream>
#include <thread>
#include <chrono>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "globals/globals.hpp"

#include "particle_filter/particle_filter_node.hpp"

using namespace std::literals::chrono_literals;

// Define Global Variables in shared memory
bool run_ = true;
std::string globals::test = "test";


void SignalHandler(int) {
    if(rclcpp::ok()){
        rclcpp::shutdown();
    }

    if (!run_) {
        printf("Force Exit.\n");
        exit(0);
    }

    printf("Exiting.\n");
    run_ = false;
}

void particle_filter_main(){
    rclcpp::spin(std::make_shared<particle_filter::ParticleFilterNode>());
}

int main(int argc, char* argv[]){
    signal(SIGINT, SignalHandler);
    rclcpp::init(argc, argv);

    // Initialize modules
    std::thread particle_filter_thread(&particle_filter_main);    

    particle_filter_thread.join();

    return 0;
}