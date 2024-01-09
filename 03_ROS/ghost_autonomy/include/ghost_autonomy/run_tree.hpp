#pragma once
#include "behaviortree_cpp/bt_factory.h"
#include "bt_nodes/driveForward.hpp"
#include "bt_nodes/loggingNode.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"

// file that contains the custom nodes definitions
// #include "dummy_nodes.h"
// using namespace DummyNodes;

class RunTreeNode : public rclcpp::Node {
private:
		std::string bt_path_;
		std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> robot_hardware_interface_ptr_;
public:
		RunTreeNode(std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> robot_hardware_interface_ptr);

		void run_tree();
};