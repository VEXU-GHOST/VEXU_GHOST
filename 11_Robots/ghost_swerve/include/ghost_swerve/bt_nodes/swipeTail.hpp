#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ghost_swerve/swerve_model.hpp"

namespace ghost_swerve {
    
class SwipeTail : public BT::SyncActionNode,
	              public rclcpp::Node {

public:
    SwipeTail(const std::string& name, const BT::NodeConfig& config,
	           std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr,
	           std::shared_ptr<SwerveModel> swerve_ptr);

    // It is mandatory to define this STATIC method.
	static BT::PortsList providedPorts();

	// Override the virtual function tick()
	BT::NodeStatus tick() override;

private:
	template <typename T>
	T get_input(std::string key);
	
    std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr_;
	std::shared_ptr<SwerveModel> swerve_ptr_;
    bool started_;
	std::chrono::time_point<std::chrono::system_clock> start_time_;

};


} // namespace ghost_swerve
