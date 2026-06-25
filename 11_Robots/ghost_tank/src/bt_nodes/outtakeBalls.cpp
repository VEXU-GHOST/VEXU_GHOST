/*
 *   Copyright (c) 2024 Jake Wendling
 *   All rights reserved.
 *
 *   OuttakeBallsCmd - Control intake/outtake motor direction.
 */

#include <algorithm>

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
    BT::InputPort<int>("direction", 0,
      "-1=outtake (spit out), 0=stop, 1=intake (pull in)"),
  };
}

BT::NodeStatus OuttakeBallsCmd::tick()
{
  int direction = BT_Util::get_input<int>(this, "direction");

  // Clamp to valid range
  direction = std::clamp(direction, -1, 1);

  BT_Util::put_in_blackboard(blackboard_, "outtake_direction", direction);

  return BT::NodeStatus::SUCCESS;
}

} // ghost_tank
