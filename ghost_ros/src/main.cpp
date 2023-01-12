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
#include "ghost_ros/ros_nodes/ghost_estimator_node.hpp"
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

int main(int argc, char* argv[]){
    signal(SIGINT, SignalHandler);
    rclcpp::init(argc, argv);

    globals::program_start_time = std::chrono::system_clock::now();    
    globals::repo_base_dir = std::string(getenv("HOME")) + "/VEXU_GHOST/";
    auto main_config = YAML::LoadFile(globals::repo_base_dir + "ghost_ros/config/main_config.yaml");

    // Define Nodes and MultiThreadedExecutor
    auto serial_node = std::make_shared<ghost_serial::JetsonV5SerialNode>(
        globals::repo_base_dir + "ghost_ros/config/ghost_serial_config.yaml");

    auto ghost_estimator_node = std::make_shared<ghost_ros::GhostEstimatorNode>(
        globals::repo_base_dir + "ghost_ros/config/ghost_estimator_config.yaml",
        main_config["simulated"].as<bool>()
        );

    auto state_machine_node = std::make_shared<ghost_ros::RobotStateMachineNode>(
        globals::repo_base_dir + "ghost_ros/config/ghost_state_machine_config.yaml"
        );
    
    rclcpp::executors::MultiThreadedExecutor executor;

    // Wait for serial port, then start reader thread
    if(!main_config["simulated"].as<bool>()){
        serial_node->initSerialBlocking();
        executor.add_node(serial_node);
    }

    // Start ROS Executor
    executor.add_node(ghost_estimator_node);
    executor.add_node(state_machine_node);

    executor.spin();

    return 0;
}