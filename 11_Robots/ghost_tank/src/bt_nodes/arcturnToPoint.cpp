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
  BT_Util::get_from_blackboard(blackboard_, "pd_control_arc_ptr", pd_control_ptr_);

  posX_m = BT_Util::get_input<double>(this, "posX_tiles") * tile_to_meters;
  posY_m = BT_Util::get_input<double>(this, "posY_tiles") * tile_to_meters;
  timeout_ms = BT_Util::get_input<int>(this, "timeout_ms");
  angle_exit_threshold_rad = BT_Util::get_input<double>(this, "angle_exit_threshold_deg") * ghost_util::DEG_TO_RAD;
  drive_backwards = BT_Util::get_input<bool>(this, "drive_backwards");
  face_backwards = BT_Util::get_input<bool>(this, "face_backwards");
  first_tick = true;
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
  first_tick = true;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ArcturnToPoint::onRunning()
{
  if (first_tick){
    first_tick = false;
    BT_Util::get_from_blackboard(blackboard_, "mirrored", mirrored);
    start_time_ = std::chrono::system_clock::now();
  }

  double cur_x = tank_model_ptr_->getWorldPose().x();
  double cur_y = tank_model_ptr_->getWorldPose().y();

  if (mirrored) {
    posX_m = 6.0 * tile_to_meters - posX_m;
  }

  des_ang_rad = std::atan2(posY_m - cur_y, posX_m - cur_x);

  if (face_backwards) {
    des_ang_rad = ghost_util::FlipAnglePI(des_ang_rad);
  }

  double theta_err_rad = std::fabs(ghost_util::SmallestAngleDistRad(tank_model_ptr_->getWorldPose().z(), des_ang_rad));
  bool angle_satisfied = theta_err_rad < angle_exit_threshold_rad;

  std::cout << "theta error: " << theta_err_rad << std::endl;
  
  int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time_).count();
  std::cout << "time elapsed: " << time_elapsed << std::endl;
  if (angle_satisfied || time_elapsed > timeout_ms) {
    tank_model_ptr_->driveCommand(0.0, 0.0);
    return BT::NodeStatus::SUCCESS;
  }

  Eigen::Vector3d final_pose_ = Eigen::Vector3d(0.0, 0.0, des_ang_rad);
  auto command = pd_control_ptr_->theta_pid(tank_model_ptr_->getWorldPose(), tank_model_ptr_->getWorldTwist(), final_pose_);

  if (drive_backwards){
    tank_model_ptr_->driveCommand(-abs(command[1]), command[1]);
  } else {
    tank_model_ptr_->driveCommand(abs(command[1]), command[1]);
  }

  return BT::NodeStatus::RUNNING;
}

void ArcturnToPoint::onHalted()
{
  resetStatus();
}

} // namespace ghost_tank
