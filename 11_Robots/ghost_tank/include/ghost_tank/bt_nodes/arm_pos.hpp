/*
 *   Copyright (c) 2026 Jonathan Ruiz
 *   All rights reserved.
 *
 *   ArmPosCmd - Drive the arm to one of its preset stage positions.
 */

#pragma once

#include <string>
#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "ghost_tank/tank_tree.hpp"
#include "ghost_tank/bt_nodes/bt_util.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"

namespace ghost_tank {

/**
 * ArmPosCmd - Sets the arm's target preset stage (0-2), matching the 3 stages
 * driver control cycles through with L2. TankRobotPlugin reads
 * "arm_target_stage" off the blackboard and drives arm_motor to that stage's
 * position with the same PID controller teleop uses.
 */
class ArmPosCmd : public BT::SyncActionNode {
public:
  ArmPosCmd(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick();

private:
  std::shared_ptr<rclcpp::Node> node_ptr_;
  std::shared_ptr<TankModel> tank_model_ptr_;
  std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr_;
  BT::Blackboard::Ptr blackboard_;
};

} // ghost_tank
