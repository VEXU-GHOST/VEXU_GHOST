#pragma once

#include <string>
#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "ghost_tank/tank_tree.hpp"
#include "ghost_tank/bt_nodes/bt_util.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"

namespace ghost_tank {

/**
 * MatchloadCmd - Auton version of clyde's d-pad right macro.
 *
 * active="true": lift to lift_stage_positions_deg[0], open the clamp, and
 * extend the arm to arm_macro_extend_deg. While on, the auto_claw sensor closes
 * the clamp (like holding L1 in teleop) unless auto_close="false", and a
 * nonzero timeout_s closes it that many seconds after the grab started.
 * active="false": close the clamp if it is still open (like releasing L1).
 *
 * Whenever the clamp closes with the arm still extended (sensor, timeout, toggle
 * off, ClampCmd, AutoClawCmd), the arm reverts to arm_macro_revert_deg and the
 * lift rises from where it is, then both hold until an ArmPosCmd / LiftPosCmd
 * takes over.
 *
 * Async: this node only posts the request to the blackboard and returns SUCCESS
 * immediately, so the tree keeps driving while ClydePlugin::updateMatchloadAuton
 * runs the macro in the background.
 */
class MatchloadCmd : public BT::SyncActionNode {
public:
  MatchloadCmd(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick();

private:
  std::shared_ptr<rclcpp::Node> node_ptr_;
  std::shared_ptr<TankModel> tank_model_ptr_;
  std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr_;
  BT::Blackboard::Ptr blackboard_;
};

} // ghost_tank
