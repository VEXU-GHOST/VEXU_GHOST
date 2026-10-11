/*
 *   Copyright (c) 2026 Jonathan Ruiz
 *   All rights reserved.
 *
 *   LiftPosCmd - Drive the lift to one of its preset stage positions.
 */

#include <algorithm>

#include "ghost_tank/bt_nodes/lift_pos.hpp"

namespace ghost_tank
{

LiftPosCmd::LiftPosCmd(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "rhi_ptr", rhi_ptr_);
}

BT::PortsList LiftPosCmd::providedPorts()
{
  return {
    BT::InputPort<int>("stage", 0,
      "Lift preset stage index (0-4) to drive to and hold"),
  };
}

BT::NodeStatus LiftPosCmd::tick()
{
  int stage = BT_Util::get_input<int>(this, "stage");

  // Clamp to valid range
  stage = std::clamp(stage, 0, 4);

  BT_Util::put_in_blackboard(blackboard_, "lift_target_stage", stage);

  return BT::NodeStatus::SUCCESS;
}

} // ghost_tank
