#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "ghost_msgs/msg/drivetrain_command.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "ghost_util/angle_util.hpp"
#include "ghost_util/unit_conversion_utils.hpp"

using std::placeholders::_1;

// SyncActionNode (synchronous action) with an input port.
class MoveToPose : public BT::SyncActionNode,
	               public rclcpp::Node {
private:
	std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> robot_hardware_interface_ptr_;
	rclcpp::Publisher<ghost_msgs::msg::DrivetrainCommand>::SharedPtr command_pub_;
public:
	// If your Node has ports, you must use this constructor signature
	MoveToPose(const std::string& name, const BT::NodeConfig& config,
	           std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> robot_hardware_interface_ptr);

	// It is mandatory to define this STATIC method.
	static BT::PortsList providedPorts();
	template <typename T>
	T get_input(std::string key);

	// Override the virtual function tick()
	BT::NodeStatus tick() override;
};