/*
 *   Copyright (c) 2024 Jake Wendling
 *   All rights reserved.
 *
 *   DescorerCmd - Toggle the descoring mechanism up and down.
 */

#include "ghost_tank/bt_nodes/descorer.hpp"

namespace ghost_tank
{

DescorerCmd::DescorerCmd(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "rhi_ptr", rhi_ptr_);
}

BT::PortsList DescorerCmd::providedPorts()
{
  return {
    BT::InputPort<bool>("up", true, "true = descorer up, false = descorer down"),
  };
}

BT::NodeStatus DescorerCmd::tick()
{
  bool up = BT_Util::get_input<bool>(this, "up");

  BT_Util::put_in_blackboard(blackboard_, "descorer_up", up);

  return BT::NodeStatus::SUCCESS;
}

} // ghost_tank
