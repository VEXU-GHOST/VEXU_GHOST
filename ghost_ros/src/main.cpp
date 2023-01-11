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
#include "ghost_ros/ros_nodes/particle_filter_node.hpp"
#include "ghost_ros/ros_nodes/jetson_v5_serial_node.hpp"
#include "ghost_ros/ros_nodes/robot_state_machine_node.hpp"

using namespace std::literals::chrono_literals;

// Define Global Variables in shared memory
namespace globals{
    std::string repo_base_dir;
    std::atomic_bool run(true);
    std::chrono::time_point<std::chrono::system_clock> program_start_time;
}

void SignalHandler(int) {
    if (!globals::run) {
        printf("Force Exit.\n");
        exit(0);
    }

    printf("Exiting.\n");
    globals::run = false;
}

// Particle Filter Main Thread
void particle_filter_main(std::string config_file, bool verbose){
    if(verbose){
        std::cout << "[START] Particle Filter Thread" << std::endl;
    }

    rclcpp::spin(std::make_shared<particle_filter::ParticleFilterNode>(config_file));
    
    if(verbose){
        std::cout << "[END] Particle Filter Thread" << std::endl;
    }
}

void serial_interface_main(std::string config_file, bool verbose)
{
    if(verbose){
        std::cout << "[START] Serial Writer Thread" << std::endl;
    }
    auto serial_node = std::make_shared<ghost_serial::JetsonV5SerialNode>(config_file);
    
    // Wait for serial port, then start reader thread
    bool success = serial_node->initSerialBlocking();

    if(success){
        // Process ROS Callbacks until exit
        rclcpp::spin(serial_node);
    }
    if(verbose){
        std::cout << "[END] Serial Writer Thread" << std::endl;
    }
}

void controller_main(std::string config_file, bool verbose){
    if(verbose){
        std::cout << "[START] State Machine Thread" << std::endl;
    }

    auto state_machine_node = std::make_shared<ghost_ros::RobotStateMachineNode>(config_file);
    
    // Process ROS Callbacks until exit
    rclcpp::spin(state_machine_node);
    
    if(verbose){
        std::cout << "[END] State Machine Thread" << std::endl;
    }
}

int main(int argc, char* argv[]){
    signal(SIGINT, SignalHandler);
    rclcpp::init(argc, argv);

    globals::program_start_time = std::chrono::system_clock::now();    
    globals::repo_base_dir = std::string(getenv("HOME")) + "/VEXU_GHOST/";
    auto main_config = YAML::LoadFile(globals::repo_base_dir + "ghost_ros/config/main_config.yaml");
    bool verbose = main_config["verbose"].as<bool>();

    std::unique_ptr<std::thread> serial_interface_thread;
    std::unique_ptr<std::thread> particle_filter_thread;
    std::unique_ptr<std::thread> controller_thread;

    if(!main_config["simulated"].as<bool>()){
        serial_interface_thread = std::make_unique<std::thread>(
            serial_interface_main,
            globals::repo_base_dir + "ghost_ros/config/ghost_serial.yaml",
            verbose
        );
    }

    controller_thread = std::make_unique<std::thread>(
        controller_main,
        globals::repo_base_dir + "ghost_ros/config/ghost_controller.yaml",
        verbose
    );

    // // Initialize modules
    // particle_filter_thread = std::make_unique<particle_filter_thread>(
    //     particle_filter_main,
    //     globals::repo_base_dir + "ghost_ros/config/particle_filter.yaml"
    //     );

    while(globals::run){
        std::this_thread::sleep_for(100ms);
    }
    
    if(rclcpp::ok()){
        std::cout << "RCLCPP Shutdown" << std::endl;
        rclcpp::shutdown();
    }

    // Cleanup threads
    if(!main_config["simulated"].as<bool>()){
        serial_interface_thread->join();
    }
    // particle_filter_thread.join();
    controller_thread->join();

    return 0;
}