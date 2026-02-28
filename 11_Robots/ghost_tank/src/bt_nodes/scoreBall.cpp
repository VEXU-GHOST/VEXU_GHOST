/*
 *   Copyright (c) 2024 Jake Wendling
 *   All rights reserved.
 *
 *   ScoreBallCmd - Control scorer (top) motor for scoring balls into the goal.
 */

#include <algorithm>

#include "ghost_tank/bt_nodes/scoreBall.hpp"

namespace ghost_tank
{

ScoreBallCmd::ScoreBallCmd(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "rhi_ptr", rhi_ptr_);
}

BT::PortsList ScoreBallCmd::providedPorts()
{
  return {
    BT::InputPort<int>("direction", 0,
      "-1=reverse, 0=stop, 1=score (scorer/top motor forward)"),
  };
}

BT::NodeStatus ScoreBallCmd::tick()
{
  int direction = BT_Util::get_input<int>(this, "direction");

  // Clamp to valid range
  direction = std::clamp(direction, -1, 1);

  BT_Util::put_in_blackboard(blackboard_, "score_ball_direction", direction);

  return BT::NodeStatus::SUCCESS;
}

} // ghost_tank
