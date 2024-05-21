#pragma once
#include "behaviortree_cpp/bt_factory.h"
#include "bt_nodes/checkForRestart.hpp"
#include "bt_nodes/intakeCmd.hpp"
#include "bt_nodes/loggingNode.hpp"
#include "bt_nodes/moveToPose.hpp"
#include "bt_nodes/swipeTail.hpp"
// #include "bt_nodes/autoDone.hpp"
#include "bt_nodes/climb.hpp"
#include "ghost_swerve/swerve_model.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

// file that contains the custom nodes definitions
// #include "dummy_nodes.h"
// using namespace DummyNodes;

namespace ghost_swerve {

class SwerveTree {
public:
	SwerveTree(std::string bt_path,
	           std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> robot_hardware_interface_ptr,
	           std::shared_ptr<SwerveModel> swerve_ptr,
	           std::shared_ptr<rclcpp::Node> node_ptr,
	           double burnout_absolute_rpm_threshold,
	           double burnout_stall_duration_ms,
	           double burnout_cooldown_duration_ms);
	void tick_tree();

private:
	std::string bt_path_;
	BT::Tree tree_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bt_auton_sub_;
	std::shared_ptr<rclcpp::Node> node_ptr_;
};

} // namespace ghost_swerve