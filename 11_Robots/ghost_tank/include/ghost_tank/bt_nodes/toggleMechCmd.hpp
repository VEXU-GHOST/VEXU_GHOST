#pragma once

#include <string>
#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "ghost_tank/bt_nodes/bt_util.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"

namespace ghost_tank {

/**
 * ToggleMechCmd - Spin toggle_mech_motor a set number of rotations (signed).
 */
class ToggleMechCmd : public BT::StatefulActionNode {
public:
  ToggleMechCmd(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart();
  BT::NodeStatus onRunning();
  void onHalted();

private:
  void setPower(double power);

  std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr_;
  BT::Blackboard::Ptr blackboard_;

  double target_deg_ = 0.0;
};

} // ghost_tank
