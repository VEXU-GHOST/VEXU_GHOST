#include "ghost_tank/bt_nodes/actuallyMoveToPoint.hpp"
#include <cmath>

namespace ghost_tank
{

ActuallyMoveToPoint::ActuallyMoveToPoint(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  std::cout << "[ActuallyMoveToPoint::ActuallyMoveToPoint]" << std::endl;

  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "arc_turn_controller_ptr", m_arc_turn_controller_ptr);
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);
  if (node_ptr_) {
    expected_pose_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>("/expectedpose", 10);
  }
}

BT::PortsList ActuallyMoveToPoint::providedPorts()
{
  return {
    BT::InputPort<double>("posX_tiles"),
    BT::InputPort<double>("posY_tiles"),
    BT::InputPort<bool>("backwards"),
    BT::InputPort<int>("timeout_ms"),
    BT::InputPort<double>("angle_exit_threshold_deg"),
    BT::InputPort<double>("kp"),
    BT::InputPort<double>("kd"),
    BT::InputPort<double>("max_effort_percent")
  };
}

BT::NodeStatus ActuallyMoveToPoint::onStart()
{
  posX_m = BT_Util::get_input<double>(this, "posX_tiles") * ghost_util::TILES_TO_METERS;
  posY_m = BT_Util::get_input<double>(this, "posY_tiles") * ghost_util::TILES_TO_METERS;
  timeout_ms = BT_Util::get_input<int>(this, "timeout_ms");
  backwards = BT_Util::get_input<bool>(this, "backwards", false);
  angle_exit_threshold_rad = BT_Util::get_input<double>(this, "angle_exit_threshold_deg", 5.0) * ghost_util::DEG_TO_RAD;

  bool mirrored = false;
  BT_Util::get_from_blackboard(blackboard_, "mirrored", mirrored);
  if (mirrored) {
    posX_m = 6.0 * ghost_util::TILES_TO_METERS - posX_m;
  }

  double cur_x = tank_model_ptr_->getWorldPose().x();
  double cur_y = tank_model_ptr_->getWorldPose().y();

  start_time_ = std::chrono::system_clock::now();
  des_ang_rad = std::atan2(posY_m - cur_y, posX_m - cur_x);
  if (backwards) {
    des_ang_rad = ghost_util::FlipAnglePI(des_ang_rad);
  }

  m_arc_turn_controller_ptr->reset();
  phase_ = Phase::TURNING;

  // Publish the heading target for the turn phase: the robot does not translate
  // while turning, so the expected pose keeps the current position.
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

BT::NodeStatus ActuallyMoveToPoint::onRunning()
{
  int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now() - start_time_).count();
  if (time_elapsed > timeout_ms) {
    tank_model_ptr_->driveCommandArcade(0.0, 0.0);
    return BT::NodeStatus::SUCCESS;
  }

  if (phase_ == Phase::TURNING) {
    double theta_err_rad = ghost_util::SmallestAngleDistRad(des_ang_rad, tank_model_ptr_->getWorldPose().z());
    bool angle_satisfied = std::fabs(theta_err_rad) < angle_exit_threshold_rad;

    if (angle_satisfied) {
      // Done turning: capture the drive start position and the distance to the
      // target, then publish the position target for the drive phase.
      tank_model_ptr_->driveCommandArcade(0.0, 0.0);
      start_position_ = tank_model_ptr_->getWorldPose().head<2>();
      distance_m = (Eigen::Vector2d(posX_m, posY_m) - start_position_).norm();
      phase_ = Phase::DRIVING;

      if (expected_pose_pub_) {
        geometry_msgs::msg::PoseStamped expected_pose;
        expected_pose.header.stamp = node_ptr_->get_clock()->now();
        expected_pose.header.frame_id = "map";
        expected_pose.pose.position.x = posX_m;
        expected_pose.pose.position.y = posY_m;
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

    tank_model_ptr_->driveCommandArcade(
      0.0, m_arc_turn_controller_ptr->calculateCommand(theta_err_rad, -tank_model_ptr_->getWorldTwist().z()));
    return BT::NodeStatus::RUNNING;
  }

  // Phase::DRIVING
  Eigen::Vector2d current_position_ = tank_model_ptr_->getWorldPose().head<2>();
  double dist_moved = (current_position_ - start_position_).norm();
  bool xy_satisfied = dist_moved > std::fabs(distance_m);

  if (xy_satisfied) {
    tank_model_ptr_->driveCommandArcade(0.0, 0.0);
    return BT::NodeStatus::SUCCESS;
  }

  move();
  return BT::NodeStatus::RUNNING;
}

void ActuallyMoveToPoint::move()
{
  double max_effort_percent = BT_Util::get_input<double>(this, "max_effort_percent");
  Kp = BT_Util::get_input<double>(this, "kp");
  Kd = BT_Util::get_input<double>(this, "kd");

  Eigen::Vector2d current_position_ = tank_model_ptr_->getWorldPose().head<2>();
  double dist_moved = (current_position_ - start_position_).norm();
  double dist_err = std::fabs(distance_m) - dist_moved;
  double fwd_cmd;
  if (dist_err > 0) {
    fwd_cmd = dist_err * Kp - (tank_model_ptr_->getWorldTwist().head<2>().norm()) * Kd;
  } else {
    fwd_cmd = 0;
  }

  fwd_cmd = ghost_util::clamp(fwd_cmd, -max_effort_percent, max_effort_percent);

  if (backwards) {
    fwd_cmd *= -1.0;
  }

  BT_Util::put_in_blackboard(blackboard_, "fwd_cmd", fwd_cmd);
  tank_model_ptr_->driveCommandArcade(fwd_cmd, 0.0);
}

void ActuallyMoveToPoint::onHalted()
{
  resetStatus();
}

} // namespace ghost_tank
