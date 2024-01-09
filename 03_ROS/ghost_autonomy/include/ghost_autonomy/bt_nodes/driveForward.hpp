#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "ghost_msgs/msg/v5_actuator_command.hpp"
#include "ghost_msgs/msg/v5_sensor_update.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

// SyncActionNode (synchronous action) with an input port.
class DriveForward : public BT::SyncActionNode,
		             public rclcpp::Node {
private:
		
public:
		// If your Node has ports, you must use this constructor signature
		DriveForward(const std::string& name, const BT::NodeConfig& config);

		// It is mandatory to define this STATIC method.
		static BT::PortsList providedPorts();

		// Override the virtual function tick()
		BT::NodeStatus tick() override;
};