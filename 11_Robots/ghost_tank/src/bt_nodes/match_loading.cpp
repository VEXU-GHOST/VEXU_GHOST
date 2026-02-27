/*
 *   Copyright (c) 2024 Jake Wendling
 *   All rights reserved.
 *
 *   MatchLoadingCmd - Bring match loading mechanism up or down.
 */

#include "ghost_tank/bt_nodes/match_loading.hpp"

namespace ghost_tank
{

MatchLoadingCmd::MatchLoadingCmd(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "rhi_ptr", rhi_ptr_);
}

BT::PortsList MatchLoadingCmd::providedPorts()
{
  return {
    BT::InputPort<bool>("up", true, "true = match load up, false = match load down"),
  };
}

BT::NodeStatus MatchLoadingCmd::tick()
{
  bool up = BT_Util::get_input<bool>(this, "up");

  BT_Util::put_in_blackboard(blackboard_, "match_loading_up", up);

  return BT::NodeStatus::SUCCESS;
}

} // ghost_tank
