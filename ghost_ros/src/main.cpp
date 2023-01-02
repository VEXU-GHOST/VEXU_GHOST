/*
 * Filename: main.cpp
 * Created Date: Monday October 24th 2022
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 * 
 * Last Modified: Friday October 28th 2022 3:45:21 pm
 * Modified By: Maxx Wilson
 */

#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "ghost_ros/globals/globals.hpp"
#include "ghost_ros/particle_filter/particle_filter_node.hpp"
#include "ghost_ros/jetson_serial/jetson_v5_serial_node.hpp"

using namespace std::literals::chrono_literals;

// Define Global Variables in shared memory
namespace globals{
    std::string repo_base_dir;
    std::atomic_bool run(true);
    std::chrono::time_point<std::chrono::system_clock> program_start_time;
}

void SignalHandler(int) {
    if(rclcpp::ok()){
        std::cout << "RCLCPP Shutdown" << std::endl;
        rclcpp::shutdown();

    }

    if (!globals::run) {
        printf("Force Exit.\n");
        exit(0);
    }

    printf("Exiting.\n");
    globals::run = false;
}

// Particle Filter Main Thread
void particle_filter_main(std::string config_file){
    rclcpp::spin(std::make_shared<particle_filter::ParticleFilterNode>(config_file));
}

void serial_interface_main(std::string config_file)
{
    bool verbose = YAML::LoadFile(config_file)["verbose"].as<bool>();

    if(verbose){
        std::cout << "[START] Serial Writer Thread" << std::endl;
    }
    auto serial_node = std::make_shared<ghost_serial::JetsonV5SerialNode>(config_file);
    
    // Wait for serial port, then start reader thread
    serial_node->initSerialBlocking();

    // Process ROS Callbacks until exit
    rclcpp::spin(serial_node);
    if(verbose){
        std::cout << "[END] Serial Writer Thread" << std::endl;
    }
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    signal(SIGINT, SignalHandler);

    globals::program_start_time = std::chrono::system_clock::now();
    
    globals::repo_base_dir = std::string(getenv("HOME")) + "/VEXU_GHOST/";

    // // Initialize modules
    // std::thread particle_filter_thread(
    //     particle_filter_main,
    //     globals::repo_base_dir + "ghost_ros/config/particle_filter.yaml"
    //     );
    
    std::thread serial_interface_thread(
        serial_interface_main,
        globals::repo_base_dir + "ghost_ros/config/ghost_serial.yaml"
    );

    // Start threads
    // particle_filter_thread.join();
    serial_interface_thread.join();

    return 0;
}