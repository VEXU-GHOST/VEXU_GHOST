#include "ghost_tank/pdcontrol.hpp"
#include "ghost_tank/bt_nodes/moveToTheta.hpp"
#include <cmath>

namespace ghost_tank
{
MoveToTheta::MoveToTheta(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  first_loop_ = true;

}

BT::PortsList MoveToTheta::providedPorts()
{
  return {
    BT::InputPort<double>("kp"),
    BT::InputPort<double>("kd"),
    BT::InputPort<double>("max_effort_percent"),
    BT::InputPort<int>("timeout_ms"),
    BT::InputPort<double>("target_theta_degrees"),
  };
}

/// Method called once, when transitioning from the state IDLE.
/// If it returns RUNNING, this becomes an asynchronous node.
BT::NodeStatus MoveToTheta::onStart()
{
  // plan_time_ = std::chrono::();
  return BT::NodeStatus::RUNNING;
}

/// when the method halt() is called and the action is RUNNING, this method is invoked.
/// This is a convenient place todo a cleanup, if needed.
void MoveToTheta::onHalted()
{
  resetStatus();
}

BT::NodeStatus MoveToTheta::onRunning()
{
  timeout_ms = BT_Util::get_input<int>(this, "timeout_ms");
  target_theta_radians = BT_Util::get_input<double>(this, "target_theta_degrees", 0.1) * ghost_util::DEG_TO_RAD;

  if (first_loop_) {
    start_time_ = std::chrono::system_clock::now();
    first_loop_ = false;
    start_theta_= tank_model_ptr_->getWorldPose().z();
  }
  double current_theta_ = tank_model_ptr_->getWorldPose().z();

  double theta_err = (current_theta_ - start_theta_);
  bool theta_satisfied = theta_err > target_theta_radians;

  int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time_).count();
  if (theta_satisfied || time_elapsed > timeout_ms) {
    tank_model_ptr_->driveCommand(0.0, 0.0);
    return BT::NodeStatus::SUCCESS;
  }
  move();
  return BT::NodeStatus::RUNNING;
}

void MoveToTheta::move()
{
  double max_effort_percent = BT_Util::get_input<double>(this, "max_effort_percent");
  Kp = BT_Util::get_input<double>(this, "kp");
  Kd = BT_Util::get_input<double>(this, "kd");

  double theta_err = ghost_util::SmallestAngleDistRad(target_theta_radians, tank_model_ptr_->getWorldPose().z());
  double turn_cmd; 
  if(theta_err>0){
    turn_cmd = theta_err*Kp - (tank_model_ptr_->getWorldTwist().z())*Kd;
  }else{
    turn_cmd = 0;
  }

  turn_cmd = ghost_util::clamp(turn_cmd, -abs(max_effort_percent), abs(max_effort_percent));

  if (max_effort_percent < 0){
    turn_cmd *= -1;
  }

  BT_Util::put_in_blackboard(blackboard_, "turn_cmd", turn_cmd);//store back in to memory 
  tank_model_ptr_->driveCommand(0.0, turn_cmd);
}


}
