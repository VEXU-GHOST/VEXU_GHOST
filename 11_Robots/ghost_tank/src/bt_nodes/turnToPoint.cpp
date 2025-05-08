#include "ghost_tank/pdcontrol.hpp"
#include "ghost_tank/bt_nodes/turnToPoint.hpp"
#include <cmath>

namespace ghost_tank
{
TurnToPoint::TurnToPoint(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  first_loop_ = true;

}

BT::PortsList TurnToPoint::providedPorts()
{
  return {
    BT::InputPort<double>("forward_effort"),
    BT::InputPort<double>("angular_effort"),
    BT::InputPort<int>("timeout_ms"),
    BT::InputPort<double>("angle_exit_threshold_deg"),
  };
}

/// Method called once, when transitioning from the state IDLE.
/// If it returns RUNNING, this becomes an asynchronous node.
BT::NodeStatus TurnToPoint::onStart()
{
  // plan_time_ = std::chrono::();
  return BT::NodeStatus::RUNNING;
}

/// when the method halt() is called and the action is RUNNING, this method is invoked.
/// This is a convenient place todo a cleanup, if needed.
void TurnToPoint::onHalted()
{
  resetStatus();
}

BT::NodeStatus TurnToPoint::onRunning()
{
  posX_m = BT_Util::get_input<double>(this, "posX_tiles") * tile_to_meters;
  posY_m = BT_Util::get_input<double>(this, "posY_tiles") * tile_to_meters;
  forward_effort = BT_Util::get_input<double>(this, "forward_effort");
  angular_effort = BT_Util::get_input<double>(this, "angular_effort");
  timeout_ms = BT_Util::get_input<int>(this, "timeout_ms");
  angle_exit_threshold_rad = BT_Util::get_input<double>(this, "angle_exit_threshold_deg", 5.0) * ghost_util::DEG_TO_RAD;
  max_speed_linear_percent = BT_Util::get_input<double>(this, "max_speed_linear_percent", 1.0);
  max_speed_angular_percent = BT_Util::get_input<double>(this, "max_speed_angular_percent", 1.0);

  if (first_loop_) {
    start_time_ = std::chrono::system_clock::now();
    findAngle();
    first_loop_ = false;
  }

  double theta_err = std::fabs((tank_model_ptr_->getWorldTwist().z() - des_ang));

  bool angle_satisfied = theta_err < angle_exit_threshold_rad;

  int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time_).count();
  if (angle_satisfied) {
    tank_model_ptr_->driveCommand(0.0, 0.0);
    return BT::NodeStatus::SUCCESS;
  }
  turn();
  return BT::NodeStatus::RUNNING;
}


void TurnToPoint::findAngle()
{
  double cur_x = tank_model_ptr_->getWorldTwist().x();
  double cur_y = tank_model_ptr_->getWorldTwist().y();

  des_ang = std::atan2(posY_m - cur_y, posX_m - cur_x) + 3.14;

}

void TurnToPoint::turn()
{
  Eigen::Vector3d final_pose_ = Eigen::Vector3d(0.0, 0.0, des_ang);
  Eigen::Vector2d command;
  command = pd_control_threshold_ptr_->theta_pid(tank_model_ptr_->getWorldPose(), tank_model_ptr_->getWorldTwist(), final_pose_);
  // Clamp steering and lateral thrust to bounds
  auto fwd_cmd = ghost_util::clamp(command[0], -max_speed_linear_percent, max_speed_linear_percent);
  auto turn_cmd = ghost_util::clamp(command[1], -max_speed_angular_percent, max_speed_angular_percent);

  // Normalize to avoid output saturation.
  double left_cmd = fwd_cmd - turn_cmd;
  double right_cmd = fwd_cmd + turn_cmd;

  // Scale commands so that max command equals full thrust
  double normalizer = 1.0 / std::max(1.0, std::max(std::fabs(left_cmd), std::fabs(right_cmd)));
  // double normalizer = 1.0;
  fwd_cmd *= normalizer;
  turn_cmd *= normalizer;


  BT_Util::put_in_blackboard(blackboard_, "fwd_cmd", fwd_cmd);
  BT_Util::put_in_blackboard(blackboard_, "turn_cmd", turn_cmd);

  tank_model_ptr_->driveCommand(fwd_cmd, turn_cmd);
}


}
