#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "alpha_jerry/tank_model.hpp"
#include "alpha_jerry/bt_nodes/bt_util.hpp"

namespace alpha_jerry {
    
class AutoDone : public BT::SyncActionNode{

public:
    AutoDone(const std::string& name, const BT::NodeConfig& config);

    // It is mandatory to define this STATIC method.
	static BT::PortsList providedPorts();

	BT::NodeStatus tick();

private:
 	std::shared_ptr<rclcpp::Node> node_ptr_;
    std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr_;
	BT::Blackboard::Ptr blackboard_;
};


} // namespace alpha_jerry
