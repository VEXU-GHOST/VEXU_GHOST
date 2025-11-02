#include "ghost_tank/bt_nodes/turnToPoint.hpp"
#include <cmath>

namespace ghost_tank
{

TurnToPoint::TurnToPoint(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  std::cout << "[TurnToPoint::TurnToPoint]" << std::endl;

  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "arc_turn_controller_ptr", m_arc_turn_controller_ptr);

  posX_m = BT_Util::get_input<double>(this, "posX_tiles") * ghost_util::TILES_TO_METERS;
  posY_m = BT_Util::get_input<double>(this, "posY_tiles") * ghost_util::TILES_TO_METERS;
  timeout_ms = BT_Util::get_input<int>(this, "timeout_ms");
  backwards = BT_Util::get_input<bool>(this, "backwards");
  angle_exit_threshold_rad = BT_Util::get_input<double>(this, "angle_exit_threshold_deg", 5.0) * ghost_util::DEG_TO_RAD;
}

BT::PortsList TurnToPoint::providedPorts()
{
  return {
    BT::InputPort<double>("posX_tiles"),
    BT::InputPort<double>("posY_tiles"),
    BT::InputPort<bool>("backwards"),
    BT::InputPort<int>("timeout_ms"),
    BT::InputPort<double>("angle_exit_threshold_deg")
  };
}

BT::NodeStatus TurnToPoint::onStart()
{
  bool mirrored = false;
  BT_Util::get_from_blackboard(blackboard_, "mirrored", mirrored);

  double cur_x = tank_model_ptr_->getWorldPose().x();
  double cur_y = tank_model_ptr_->getWorldPose().y();

  if (mirrored) {
    posX_m = 6.0 * ghost_util::TILES_TO_METERS - posX_m;
  }

  start_time_ = std::chrono::system_clock::now();
  des_ang_rad = std::atan2(posY_m - cur_y, posX_m - cur_x);

  if (backwards) {
    des_ang_rad = ghost_util::FlipAnglePI(des_ang_rad);
  }

  m_arc_turn_controller_ptr->reset();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TurnToPoint::onRunning()
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

void TurnToPoint::onHalted()
{
  resetStatus();
}

} // namespace ghost_tank
