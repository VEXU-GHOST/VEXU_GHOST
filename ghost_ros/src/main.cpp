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

#include "globals/globals.hpp"
#include "particle_filter/particle_filter_node.hpp"
#include "ghost_serial/ghost_serial_node.hpp"

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
    auto serial_node = std::make_shared<ghost_serial::GhostSerialNode>(config_file);
    std::cout << "Init" << std::endl;
    // // Open serial port
    // bool serial_open = false;
    // while(globals::run && !serial_open){
    //     serial_open = serial_node->trySerialOpen();
    // }

    // serial_node->startReaderThread();

    // rclcpp::spin(serial_node);
    std::cout << "End" << std::endl;
}

int main(int argc, char* argv[]){
    globals::program_start_time = std::chrono::system_clock::now();
    
    signal(SIGINT, SignalHandler);
    rclcpp::init(argc, argv);
    
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

    // particle_filter_thread.join();
    std::cout << "Gonna Join" << std::endl;
    serial_interface_thread.join();
    std::cout << "Joined" << std::endl;

    return 0;
}