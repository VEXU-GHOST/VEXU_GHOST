/*
 *   TurnToAngle - Point turn in place to a world heading.
 */

#include "ghost_tank/bt_nodes/turnToAngle.hpp"

namespace ghost_tank
{

TurnToAngle::TurnToAngle(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "arc_turn_controller_ptr", m_arc_turn_controller_ptr);
}

BT::PortsList TurnToAngle::providedPorts()
{
  return {
    BT::InputPort<double>("angle_deg", "field heading: 0 = +x, 90 = +y, CCW positive"),
    BT::InputPort<int>("timeout_ms", 1500, "give up after this long"),
    BT::InputPort<double>("angle_exit_threshold_deg", 3.0, "done when within this of the target"),
    BT::InputPort<int>("angle_settle_ms", 0, "must stay within threshold this long to exit"),
    BT::InputPort<double>("max_effort_percent", 1.0, "turn power cap (0-1)"),
  };
}

BT::NodeStatus TurnToAngle::onStart()
{
  bool mirrored = false;
  BT_Util::get_from_blackboard(blackboard_, "mirrored", mirrored);

  des_ang_rad = BT_Util::get_input<double>(this, "angle_deg") * ghost_util::DEG_TO_RAD;
  // Mirroring flips x, so a heading theta becomes 180 - theta
  if (mirrored) {
    des_ang_rad = M_PI - des_ang_rad;
  }
  des_ang_rad = ghost_util::WrapAnglePI(des_ang_rad);

  timeout_ms = BT_Util::get_input<int>(this, "timeout_ms");
  angle_exit_threshold_rad = BT_Util::get_input<double>(this, "angle_exit_threshold_deg") * ghost_util::DEG_TO_RAD;
  angle_settle_ms = BT_Util::get_input<int>(this, "angle_settle_ms");
  max_effort = BT_Util::get_input<double>(this, "max_effort_percent");

  start_time_ = std::chrono::system_clock::now();
  m_arc_turn_controller_ptr->reset();
  angle_settling_ = false;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TurnToAngle::onRunning()
{
  double theta_err_rad = ghost_util::SmallestAngleDistRad(des_ang_rad, tank_model_ptr_->getWorldPose().z());
  bool angle_in_threshold = std::fabs(theta_err_rad) < angle_exit_threshold_rad;

  // Require the heading to stay within threshold for angle_settle_ms before
  // exiting, so a momentary pass-through on overshoot doesn't end the turn early.
  bool angle_satisfied = false;
  if (angle_in_threshold) {
    if (!angle_settling_) {
      angle_settling_ = true;
      angle_settle_start_ = std::chrono::system_clock::now();
    }
    int settled_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now() - angle_settle_start_).count();
    angle_satisfied = settled_ms >= angle_settle_ms;
  } else {
    angle_settling_ = false;
  }

  int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now() - start_time_).count();
  if (angle_satisfied || time_elapsed > timeout_ms) {
    tank_model_ptr_->driveCommandArcade(0.0, 0.0);
    return BT::NodeStatus::SUCCESS;
  }

  double turn_cmd = m_arc_turn_controller_ptr->calculateCommand(
    theta_err_rad, -tank_model_ptr_->getWorldTwist().z());
  turn_cmd = ghost_util::clamp(turn_cmd, -max_effort, max_effort);
  tank_model_ptr_->driveCommandArcade(0.0, turn_cmd);
  return BT::NodeStatus::RUNNING;
}

void TurnToAngle::onHalted()
{
  tank_model_ptr_->driveCommandArcade(0.0, 0.0);
  resetStatus();
}

} // namespace ghost_tank
