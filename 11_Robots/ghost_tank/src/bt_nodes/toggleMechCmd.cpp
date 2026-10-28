/*
 *   ToggleMechCmd - Spin toggle_mech_motor a set number of rotations (signed).
 */

#include <cmath>

#include "ghost_tank/bt_nodes/toggleMechCmd.hpp"

namespace ghost_tank
{

ToggleMechCmd::ToggleMechCmd(
  const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "rhi_ptr", rhi_ptr_);
}

BT::PortsList ToggleMechCmd::providedPorts()
{
  return {
    BT::InputPort<double>("rotations", 1.0, "Rotations to spin (negative = reverse)"),
  };
}

BT::NodeStatus ToggleMechCmd::onStart()
{
  double rotations = BT_Util::get_input<double>(this, "rotations");
  target_deg_ = rhi_ptr_->getMotorPosition("toggle_mech_motor") + rotations * 360.0;
  return onRunning();
}

BT::NodeStatus ToggleMechCmd::onRunning()
{
  double error = target_deg_ - rhi_ptr_->getMotorPosition("toggle_mech_motor");

  // Bang-bang until we pass the target
  if (std::fabs(error) > 5.0) {
    setPower(error > 0 ? 1.0 : -1.0);
    return BT::NodeStatus::RUNNING;
  }

  setPower(0.0);
  return BT::NodeStatus::SUCCESS;
}

void ToggleMechCmd::onHalted()
{
  setPower(0.0);
  resetStatus();
}

void ToggleMechCmd::setPower(double power)
{
  rhi_ptr_->setMotorCurrentLimitMilliAmps("toggle_mech_motor", 2500);
  rhi_ptr_->setMotorVoltageCommandPercent("toggle_mech_motor", power);
}

} // ghost_tank
