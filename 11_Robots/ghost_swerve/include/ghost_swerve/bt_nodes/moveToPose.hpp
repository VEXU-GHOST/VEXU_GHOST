#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "ghost_msgs/msg/drivetrain_command.hpp"
#include "ghost_swerve/swerve_model.hpp"
#include "ghost_util/angle_util.hpp"
#include "ghost_util/unit_conversion_utils.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"

using std::placeholders::_1;

namespace ghost_swerve {

// SyncActionNode (synchronous action) with an input port.
class MoveToPose : public BT::StatefulActionNode {
public:
	// If your Node has ports, you must use this constructor signature
	MoveToPose(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node_ptr,
	           std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr,
	           std::shared_ptr<SwerveModel> swerve_ptr);

	// It is mandatory to define this STATIC method.
	static BT::PortsList providedPorts();

	/// Method called once, when transitioning from the state IDLE.
	/// If it returns RUNNING, this becomes an asynchronous node.
	BT::NodeStatus onStart();

	/// method invoked when the action is already in the RUNNING state.
	BT::NodeStatus onRunning();

	/// when the method halt() is called and the action is RUNNING, this method is invoked.
	/// This is a convenient place todo a cleanup, if needed.
	void onHalted();

	// Override the virtual function tick()
	// BT::NodeStatus tick() override;

private:
	template <typename T>
	T get_input(std::string key);

	std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr_;
	std::shared_ptr<SwerveModel> swerve_ptr_;
	rclcpp::Publisher<ghost_msgs::msg::DrivetrainCommand>::SharedPtr command_pub_;
	std::chrono::time_point<std::chrono::system_clock> start_time_;
	std::chrono::time_point<std::chrono::system_clock> plan_time_;
	std::shared_ptr<rclcpp::Node> node_ptr_;
	bool started_;
};

} // namespace ghost_swerve {