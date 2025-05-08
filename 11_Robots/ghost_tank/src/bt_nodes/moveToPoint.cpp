#include "ghost_tank/pdcontrol.hpp"
#include "ghost_tank/bt_nodes/moveToPoint.hpp"
#include <cmath>

namespace ghost_tank
{
MoveToPoint::MoveToPoint(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  first_loop_ = true;

}

BT::PortsList MoveToPoint::providedPorts()
{
  return {
    BT::InputPort<double>("forward_effort"),
    BT::InputPort<double>("angular_effort"),
    BT::InputPort<int>("timeout_ms"),
    BT::InputPort<double>("xy_exit_threshold_tiles"),
  };
}

/// Method called once, when transitioning from the state IDLE.
/// If it returns RUNNING, this becomes an asynchronous node.
BT::NodeStatus MoveToPoint::onStart()
{
  // plan_time_ = std::chrono::();
  return BT::NodeStatus::RUNNING;
}

/// when the method halt() is called and the action is RUNNING, this method is invoked.
/// This is a convenient place todo a cleanup, if needed.
void MoveToPoint::onHalted()
{
  resetStatus();
}

BT::NodeStatus MoveToPoint::onRunning()
{
  posX_m = BT_Util::get_input<double>(this, "posX_tiles") * tile_to_meters;
  posY_m = BT_Util::get_input<double>(this, "posY_tiles") * tile_to_meters;
  forward_effort = BT_Util::get_input<double>(this, "forward_effort");
  angular_effort = BT_Util::get_input<double>(this, "angular_effort");
  timeout_ms = BT_Util::get_input<int>(this, "timeout_ms");
  xy_exit_threshold_m = BT_Util::get_input<double>(this, "xy_exit_threshold_tiles", 0.1) * tile_to_meters;
  max_speed_linear_percent = BT_Util::get_input<double>(this, "max_speed_linear_percent", 1.0);
  max_speed_angular_percent = BT_Util::get_input<double>(this, "max_speed_angular_percent", 1.0);
  backwards = BT_Util::get_input<bool>(this, "backwards", false);

  if (first_loop_) {
    start_time_ = std::chrono::system_clock::now();
    first_loop_ = false;
  }
  double cur_z = tank_model_ptr_->getWorldTwist().z();
  double cur_x = tank_model_ptr_->getWorldTwist().x();
  double cur_y = tank_model_ptr_->getWorldTwist().y();

  Eigen::Vector2d des_pos = Eigen::Vector2d(posX_m, posY_m);

  double dist_err = (des_pos - tank_model_ptr_->getWorldPose().head<2>()).norm();
  bool xy_satisfied = dist_err < xy_exit_threshold_m;

  int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time_).count();
  if (xy_satisfied) {
    tank_model_ptr_->driveCommand(0.0, 0.0);
    return BT::NodeStatus::SUCCESS;
  }
  move();
  return BT::NodeStatus::RUNNING;
}

void MoveToPoint::move()
{

  auto x_trajectory = robot_trajectory_.x_trajectory.position_vector;
  auto y_trajectory = robot_trajectory_.y_trajectory.position_vector;
  auto theta_trajectory = robot_trajectory_.theta_trajectory.position_vector;

  Eigen::Vector3d desired_pose;
  past_index_ = x_trajectory.size() - 1; // Initialize to end so if we are way off the path (no points inside pursuit radius), we go straight to final pose

  desired_pose = Eigen::Vector3d(x_trajectory[past_index_], y_trajectory[past_index_], 0.0);

  Eigen::Vector3d final_pose_ = Eigen::Vector3d(posY_m, posX_m, cur_z); //ending point uses the current z to insure that it stays straight
  Eigen::Vector2d command;
  Eigen::Vector2d des_pos = Eigen::Vector2d(posX_m, posY_m);

  double dist_err = (des_pos - tank_model_ptr_->getWorldPose().head<2>()).norm();
  search_radius = BT_Util::get_input<double>(this, "search_radius_tiles", 0.3) * tile_to_meters;

  bool within_pursuit_radius = dist_err < search_radius;
  command = pd_control_ptr_->tank_pid(tank_model_ptr_->getWorldPose(), tank_model_ptr_->getWorldTwist(), desired_pose, final_pose_, backwards, within_pursuit_radius);
  // Clamp steering and lateral thrust to bounds
  auto fwd_cmd = ghost_util::clamp(command[0], -max_speed_linear_percent, max_speed_linear_percent);

  // Normalize to avoid output saturation.
  double left_cmd = fwd_cmd;
  double right_cmd = fwd_cmd;

  // Scale commands so that max command equals full thrust
  double normalizer = 1.0 / std::max(1.0, std::max(std::fabs(left_cmd), std::fabs(right_cmd)));
  // double normalizer = 1.0;

  fwd_cmd *= normalizer;



  BT_Util::put_in_blackboard(blackboard_, "fwd_cmd", fwd_cmd);
  tank_model_ptr_->driveCommand(fwd_cmd, 0.0);
}


}
