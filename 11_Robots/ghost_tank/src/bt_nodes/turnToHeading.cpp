#include "ghost_tank/bt_nodes/turnToHeading.hpp"
#include <cmath>

namespace ghost_tank
{

TurnToHeading::TurnToHeading(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  std::cout << "[TurnToHeading::TurnToHeading]" << std::endl;

  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "arc_turn_controller_ptr", m_arc_turn_controller_ptr);

  timeout_ms = BT_Util::get_input<int>(this, "timeout_ms");
  backwards = BT_Util::get_input<bool>(this, "backwards");
  angle_exit_threshold_rad = BT_Util::get_input<double>(this, "angle_exit_threshold_deg", 5.0) * ghost_util::DEG_TO_RAD;
}

BT::PortsList TurnToHeading::providedPorts()
{
  return {
    BT::InputPort<double>("theta_deg", "absolute map-frame heading to settle on, degrees"),
    BT::InputPort<bool>("backwards", false, "settle on theta_deg + 180 (drive backwards-facing)"),
    BT::InputPort<int>("timeout_ms"),
    BT::InputPort<double>("angle_exit_threshold_deg")
  };
}

BT::NodeStatus TurnToHeading::onStart()
{
  bool mirrored = false;
  BT_Util::get_from_blackboard(blackboard_, "mirrored", mirrored);

  start_time_ = std::chrono::system_clock::now();
  des_ang_rad = BT_Util::get_input<double>(this, "theta_deg") * ghost_util::DEG_TO_RAD;

  if (backwards) {
    des_ang_rad = ghost_util::FlipAnglePI(des_ang_rad);
  }

  // Mirror about center line of VEX field (matches GeneratePlannerPath).
  if (mirrored) {
    des_ang_rad = ghost_util::WrapAngle2PI(M_PI - des_ang_rad);
  }

  m_arc_turn_controller_ptr->reset();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TurnToHeading::onRunning()
{
  double theta_err_rad = ghost_util::SmallestAngleDistRad(des_ang_rad, tank_model_ptr_->getWorldPose().z());
  bool angle_satisfied = std::fabs(theta_err_rad) < angle_exit_threshold_rad;

  int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time_).count();
  if (angle_satisfied || time_elapsed > timeout_ms) {
    tank_model_ptr_->driveCommandArcade(0.0, 0.0);
    return BT::NodeStatus::SUCCESS;
  }

  tank_model_ptr_->driveCommandArcade(0.0, m_arc_turn_controller_ptr->calculateCommand(theta_err_rad, -tank_model_ptr_->getWorldTwist().z()));
  return BT::NodeStatus::RUNNING;
}

void TurnToHeading::onHalted()
{
  resetStatus();
}

} // namespace ghost_tank
