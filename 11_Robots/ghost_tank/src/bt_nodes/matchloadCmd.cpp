/*
 *   MatchloadCmd - Toggle the auton version of clyde's d-pad right macro.
 */

#include "ghost_tank/bt_nodes/matchloadCmd.hpp"

namespace ghost_tank
{

MatchloadCmd::MatchloadCmd(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "rhi_ptr", rhi_ptr_);
}

BT::PortsList MatchloadCmd::providedPorts()
{
  return {
    BT::InputPort<bool>("active", true,
      "true = lift down, clamp open, arm extended; false = close the clamp (arm reverts)"),
    BT::InputPort<bool>("auto_close", true,
      "while on, close the clamp when the auto_claw sensor triggers"),
    BT::InputPort<double>("timeout_s", 0.0,
      "while on, close the clamp this many seconds after the grab started (0 = never)"),
  };
}

BT::NodeStatus MatchloadCmd::tick()
{
  bool active = BT_Util::get_input<bool>(this, "active");
  bool auto_close = BT_Util::get_input<bool>(this, "auto_close");
  double timeout_s = BT_Util::get_input<double>(this, "timeout_s");

  if (active) {
    // Hand the arm and lift to the macro (ClydePlugin::updateMatchloadAuton) until
    // an ArmPosCmd / LiftPosCmd posts a stage.
    BT_Util::put_in_blackboard(blackboard_, "clamp_closed", false);
    BT_Util::put_in_blackboard(blackboard_, "lift_target_stage", -1);
    BT_Util::put_in_blackboard(blackboard_, "arm_target_stage", -1);
    BT_Util::put_in_blackboard(blackboard_, "matchload_auto_close", auto_close);
    BT_Util::put_in_blackboard(blackboard_, "matchload_timeout_s", timeout_s);
  }
  // Consumed (reset to -1) by ClydePlugin::updateMatchloadAuton
  BT_Util::put_in_blackboard(blackboard_, "matchload_cmd", active ? 1 : 0);

  return BT::NodeStatus::SUCCESS;
}

} // ghost_tank
