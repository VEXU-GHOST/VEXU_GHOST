#pragma once
#include "behaviortree_cpp/bt_factory.h"
#include "bt_nodes/driveForward.hpp"
#include "bt_nodes/loggingNode.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

// file that contains the custom nodes definitions
// #include "dummy_nodes.h"
// using namespace DummyNodes;

class RunTree {
private:
		std::string bt_path_;
		BT::Tree tree_;
		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bt_auton_sub_;
public:
		RunTree(std::string bt_path, std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> robot_hardware_interface_ptr);

		void tick_tree();
};