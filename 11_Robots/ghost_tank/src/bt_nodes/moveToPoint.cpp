#include "ghost_tank/control/tank_pid_controller.hpp"
#include "ghost_tank/bt_nodes/moveToPoint.hpp"
#include <cmath>

namespace ghost_tank
{
MoveToPoint::MoveToPoint(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  std::cout << "[MoveToPoint::MoveToPoint]" << std::endl;

  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  first_loop_ = true;

}

BT::PortsList MoveToPoint::providedPorts()
{
  return {
    BT::InputPort<double>("kp"),
    BT::InputPort<double>("kd"),
    BT::InputPort<double>("max_effort_percent"),
    BT::InputPort<int>("timeout_ms"),
    BT::InputPort<double>("xy_exit_threshold_tiles"),
  };
}

/// Method called once, when transitioning from the state IDLE.
/// If it returns RUNNING, this becomes an asynchronous node.
BT::NodeStatus MoveToPoint::onStart()
{
  first_loop_ = true;
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
  timeout_ms = BT_Util::get_input<int>(this, "timeout_ms");
  xy_exit_threshold_m = BT_Util::get_input<double>(this, "xy_exit_threshold_tiles", 0.1) * tile_to_meters;

  if (first_loop_) {
    start_time_ = std::chrono::system_clock::now();
    first_loop_ = false;
    start_position_= tank_model_ptr_->getWorldPose().head<2>();
  }
  Eigen::Vector2d current_position_ = tank_model_ptr_->getWorldPose().head<2>();

  double dist_err = (current_position_ - start_position_).norm();
  bool xy_satisfied = dist_err > abs(xy_exit_threshold_m);

  int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time_).count();
  if (xy_satisfied || time_elapsed > timeout_ms) {
    tank_model_ptr_->driveCommand(0.0, 0.0);
    return BT::NodeStatus::SUCCESS;
  }
  move();
  return BT::NodeStatus::RUNNING;
}

void MoveToPoint::move()
{
  double max_effort_percent = BT_Util::get_input<double>(this, "max_effort_percent");
  Kp = BT_Util::get_input<double>(this, "kp");
  Kd = BT_Util::get_input<double>(this, "kd");
  
  Eigen::Vector2d current_position_ = tank_model_ptr_->getWorldPose().head<2>();
  double dist_moved = (current_position_ - start_position_).norm();
  double dist_err = abs(xy_exit_threshold_m) - dist_moved;
  double fwd_cmd; 
  if(dist_err>0){
    fwd_cmd = dist_err*Kp - (tank_model_ptr_->getWorldTwist().head<2>().norm())*Kd;
  }else{
    fwd_cmd = 0;
  }

  fwd_cmd = ghost_util::clamp(fwd_cmd, -max_effort_percent, max_effort_percent);

  if (xy_exit_threshold_m < 0){
    fwd_cmd *= -1;
  }

  BT_Util::put_in_blackboard(blackboard_, "fwd_cmd", fwd_cmd);//store back in to memory 
  tank_model_ptr_->driveCommand(fwd_cmd, 0.0);
}


}
