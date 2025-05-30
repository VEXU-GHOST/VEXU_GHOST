#include "ghost_tank/bt_nodes/arcturnToPoint.hpp"
#include "ghost_util/angle_util.hpp"
#include <cmath>

namespace ghost_tank
{

ArcturnToPoint::ArcturnToPoint(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "arc_turn_controller_ptr", m_arc_turn_controller_ptr);

  posX_m = BT_Util::get_input<double>(this, "posX_tiles") * ghost_util::TILES_TO_METERS;
  posY_m = BT_Util::get_input<double>(this, "posY_tiles") * ghost_util::TILES_TO_METERS;
  timeout_ms = BT_Util::get_input<int>(this, "timeout_ms");
  angle_exit_threshold_rad = BT_Util::get_input<double>(this, "angle_exit_threshold_deg") * ghost_util::DEG_TO_RAD;
  drive_backwards = BT_Util::get_input<bool>(this, "drive_backwards");
  face_backwards = BT_Util::get_input<bool>(this, "face_backwards");
}

BT::PortsList ArcturnToPoint::providedPorts()
{
  return {
    BT::InputPort<double>("posX_tiles"),
    BT::InputPort<double>("posY_tiles"),
    BT::InputPort<int>("timeout_ms"),
    BT::InputPort<double>("angle_exit_threshold_deg"),
    BT::InputPort<bool>("face_backwards"),
    BT::InputPort<bool>("drive_backwards")
  };
}

BT::NodeStatus ArcturnToPoint::onStart()
{
  BT_Util::get_from_blackboard(blackboard_, "mirrored", mirrored);
  start_time_ = std::chrono::system_clock::now();
  m_arc_turn_controller_ptr->reset();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ArcturnToPoint::onRunning()
{
  double cur_x = tank_model_ptr_->getWorldPose().x();
  double cur_y = tank_model_ptr_->getWorldPose().y();

  if (mirrored) {
    posX_m = 6.0 * tile_to_meters - posX_m;
  }

  des_ang_rad = std::atan2(posY_m - cur_y, posX_m - cur_x);

  if (face_backwards) {
    des_ang_rad = ghost_util::FlipAnglePI(des_ang_rad);
  }

  double theta_err_rad = ghost_util::SmallestAngleDistRad(des_ang_rad, tank_model_ptr_->getWorldPose().z());
  bool angle_satisfied = std::fabs(theta_err_rad) < angle_exit_threshold_rad;

  // std::cout << "theta error: " << theta_err_rad << std::endl;
  
  int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time_).count();
  // std::cout << "time elapsed: " << time_elapsed << std::endl;
  if (angle_satisfied || time_elapsed > timeout_ms) {
    tank_model_ptr_->driveCommand(0.0, 0.0);
    return BT::NodeStatus::SUCCESS;
  }

  auto command = m_arc_turn_controller_ptr->calculateCommand(theta_err_rad, -tank_model_ptr_->getWorldTwist().z());
  if (drive_backwards){
    tank_model_ptr_->driveCommand(-abs(command), command);
  } else {
    tank_model_ptr_->driveCommand(abs(command), command);
  }

  return BT::NodeStatus::RUNNING;
}

void ArcturnToPoint::onHalted()
{
  resetStatus();
}

} // namespace ghost_tank
