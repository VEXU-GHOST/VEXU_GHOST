#pragma once

#include <string>
#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "ghost_tank/tank_tree.hpp"
#include "ghost_tank/bt_nodes/bt_util.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"

namespace ghost_tank {

class ClimbCmd : public BT::SyncActionNode {
public:
  ClimbCmd(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick();

private:
  std::shared_ptr<rclcpp::Node> node_ptr_;
  std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr_;
  BT::Blackboard::Ptr blackboard_;
};

} // namespace ghost_tank
