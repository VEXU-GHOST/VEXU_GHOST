/*
 *   Copyright (c) 2024 Jake Wendling
 *   All rights reserved.
 *
 *   ScorePosCmd - Score/dump balls through the top, running all rollers.
 */

#include "ghost_tank/bt_nodes/score_pos.hpp"

namespace ghost_tank
{

ScorePosCmd::ScorePosCmd(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "rhi_ptr", rhi_ptr_);
}

BT::PortsList ScorePosCmd::providedPorts()
{
  return {
    BT::InputPort<bool>("active", false, "true = score position up / dump balls"),
  };
}

BT::NodeStatus ScorePosCmd::tick()
{
  bool active = BT_Util::get_input<bool>(this, "active");

  BT_Util::put_in_blackboard(blackboard_, "score_pos_up", active);

  return BT::NodeStatus::SUCCESS;
}

} // ghost_tank
