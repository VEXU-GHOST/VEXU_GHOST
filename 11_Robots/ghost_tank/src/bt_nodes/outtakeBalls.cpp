/*
 *   Copyright (c) 2024 Jake Wendling
 *   All rights reserved.
 *
 *   OuttakeBallsCmd - Spit out balls through the intake.
 */

#include "ghost_tank/bt_nodes/outtakeBalls.hpp"

namespace ghost_tank
{

OuttakeBallsCmd::OuttakeBallsCmd(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "rhi_ptr", rhi_ptr_);
}

BT::PortsList OuttakeBallsCmd::providedPorts()
{
  return {
    BT::InputPort<bool>("active", false, "true = outtake through intake"),
  };
}

BT::NodeStatus OuttakeBallsCmd::tick()
{
  bool active = BT_Util::get_input<bool>(this, "active");

  BT_Util::put_in_blackboard(blackboard_, "outtake_active", active);

  return BT::NodeStatus::SUCCESS;
}

} // ghost_tank
