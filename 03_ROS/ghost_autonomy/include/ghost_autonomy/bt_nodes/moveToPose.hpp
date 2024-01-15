#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

// SyncActionNode (synchronous action) with an input port.
class MoveToPose : public BT::SyncActionNode,
		           public rclcpp::Node {
private:
		std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> robot_hardware_interface_ptr_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_; 
public:
		// If your Node has ports, you must use this constructor signature
		MoveToPose(const std::string& name, const BT::NodeConfig& config,
		           std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> robot_hardware_interface_ptr);

		// It is mandatory to define this STATIC method.
		static BT::PortsList providedPorts();

        double get_input(std::string key);

		// Override the virtual function tick()
		BT::NodeStatus tick() override;
};