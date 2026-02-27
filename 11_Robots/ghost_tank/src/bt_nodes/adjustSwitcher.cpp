/*
 *   Copyright (c) 2024 Jake Wendling
 *   All rights reserved.
 *
 *   AdjustSwitcher - Moves the upper portion between scoring on long goal vs center goal.
 */

#include "ghost_tank/bt_nodes/adjustSwitcher.hpp"

namespace ghost_tank
{

AdjustSwitcherCmd::AdjustSwitcherCmd(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "rhi_ptr", rhi_ptr_);
}

BT::PortsList AdjustSwitcherCmd::providedPorts()
{
  return {
    BT::InputPort<bool>("long_goal", true, "true = long goal, false = center goal"),
  };
}

BT::NodeStatus AdjustSwitcherCmd::tick()
{
  bool long_goal = BT_Util::get_input<bool>(this, "long_goal");

  BT_Util::put_in_blackboard(blackboard_, "switcher_long_goal", long_goal);

  return BT::NodeStatus::SUCCESS;
}

} // ghost_tank
