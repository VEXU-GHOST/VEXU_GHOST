#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "ghost_msgs/msg/v5_actuator_command.hpp"
#include "ghost_msgs/msg/v5_sensor_update.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

namespace ghost_swerve {

// SyncActionNode (synchronous action) with an input port.
class CheckForRestart : public BT::SyncActionNode{
private:
	std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> robot_hardware_interface_ptr_;
	std::chrono::time_point<std::chrono::system_clock> start_time_;
	std::shared_ptr<rclcpp::Node> node_ptr_;
	bool restarted_ = false;
public:
	// If your Node has ports, you must use this constructor signature
	CheckForRestart(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node_ptr,
	                std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> robot_hardware_interface_ptr);

	// It is mandatory to define this STATIC method.
	static BT::PortsList providedPorts();

	// Override the virtual function tick()
	BT::NodeStatus tick() override;
};

} // namespace ghost_swerve
