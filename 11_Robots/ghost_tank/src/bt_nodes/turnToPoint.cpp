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
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);
  if (node_ptr_) {
    expected_pose_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>("/expectedpose", 10);
  }

  posX_m = BT_Util::get_input<double>(this, "posX_tiles") * ghost_util::TILES_TO_METERS;
  posY_m = BT_Util::get_input<double>(this, "posY_tiles") * ghost_util::TILES_TO_METERS;
  timeout_ms = BT_Util::get_input<int>(this, "timeout_ms");
  backwards = BT_Util::get_input<bool>(this, "backwards");
  angle_exit_threshold_rad = BT_Util::get_input<double>(this, "angle_exit_threshold_deg", 5.0) * ghost_util::DEG_TO_RAD;
  angle_settle_ms = BT_Util::get_input<int>(this, "angle_settle_ms", 0);
}

BT::PortsList TurnToPoint::providedPorts()
{
  return {
    BT::InputPort<double>("posX_tiles"),
    BT::InputPort<double>("posY_tiles"),
    BT::InputPort<bool>("backwards"),
    BT::InputPort<int>("timeout_ms"),
    BT::InputPort<double>("angle_exit_threshold_deg"),
    BT::InputPort<int>("angle_settle_ms")
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
  angle_settling_ = false;

  // Publish the target the controller is driving toward: the robot does not
  // translate during a turn, so the expected pose keeps the current position
  // and adopts the desired heading.
  if (expected_pose_pub_) {
    geometry_msgs::msg::PoseStamped expected_pose;
    expected_pose.header.stamp = node_ptr_->get_clock()->now();
    expected_pose.header.frame_id = "map";
    expected_pose.pose.position.x = cur_x;
    expected_pose.pose.position.y = cur_y;
    expected_pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, des_ang_rad);
    expected_pose.pose.orientation.x = q.x();
    expected_pose.pose.orientation.y = q.y();
    expected_pose.pose.orientation.z = q.z();
    expected_pose.pose.orientation.w = q.w();
    expected_pose_pub_->publish(expected_pose);
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TurnToPoint::onRunning()
{
  double theta_err_rad = ghost_util::SmallestAngleDistRad(des_ang_rad, tank_model_ptr_->getWorldPose().z());
  bool angle_in_threshold = std::fabs(theta_err_rad) < angle_exit_threshold_rad;

  // Require the heading to stay within threshold for angle_settle_ms before
  // exiting, so a momentary pass-through on overshoot doesn't end the turn
  // early. angle_settle_ms == 0 keeps the instantaneous behaviour.
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
