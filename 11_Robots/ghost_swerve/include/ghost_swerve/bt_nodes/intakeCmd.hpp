#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ghost_swerve/swerve_model.hpp"

namespace ghost_swerve {
    
class IntakeCmd : public BT::SyncActionNode{

public:
    IntakeCmd(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node_ptr,
	           	std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr,
	           	std::shared_ptr<SwerveModel> swerve_ptr,
			   	double burnout_absolute_rpm_threshold,
			   	double burnout_stall_duration_ms,
				double burnout_cooldown_duration_ms);

    // It is mandatory to define this STATIC method.
	static BT::PortsList providedPorts();

	BT::NodeStatus tick();

private:
	template <typename T>
	T get_input(std::string key);
	
	bool intake_stalling_;
	bool intake_cooling_down_;
	double burnout_absolute_rpm_threshold_;
	double burnout_stall_duration_ms_;
	double burnout_cooldown_duration_ms_;
	std::chrono::time_point<std::chrono::system_clock> intake_stall_start_;
	std::chrono::time_point<std::chrono::system_clock> intake_cooldown_start_;

    std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr_;
	std::shared_ptr<rclcpp::Node> node_ptr_;
	std::shared_ptr<SwerveModel> swerve_ptr_;
};


} // namespace ghost_swerve
