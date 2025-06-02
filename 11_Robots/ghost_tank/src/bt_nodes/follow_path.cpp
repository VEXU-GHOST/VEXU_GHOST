/*
 *   Copyright (c) 2025 Jake Wendling, Maxx Wilson
 *   All rights reserved.

 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */


#include "ghost_tank/bt_nodes/follow_path.hpp"
#include <ghost_util/angle_util.hpp>
#include <ghost_tank/visualization/visualization_helpers.hpp>

namespace ghost_tank
{

FollowPath::FollowPath(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  std::cout << "[FollowPath::FollowPath]" << std::endl;

  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);

  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "tank_trajectory_ptr", tank_trajectory_ptr_);

  BT_Util::get_from_blackboard(blackboard_, "distance_approach_controller_ptr", m_distance_approach_controller_ptr);
  BT_Util::get_from_blackboard(blackboard_, "steering_approach_controller_ptr", m_distance_settling_controller_ptr);
  BT_Util::get_from_blackboard(blackboard_, "distance_settling_controller_ptr", m_steering_approach_controller_ptr);
  BT_Util::get_from_blackboard(blackboard_, "steering_settling_controller_ptr", m_steering_settling_controller_ptr);

  path_viz_pub_ptr_ = node_ptr_->create_publisher<visualization_msgs::msg::MarkerArray>("/autonomy/follow_path/viz_markers", 10);
  exit_threshold_viz_pub_ptr_ = node_ptr_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/autonomy/follow_path/exit_threshold_viz", 10);
  twist_command_pub_ptr_ = node_ptr_->create_publisher<geometry_msgs::msg::TwistStamped>("/autonomy/follow_path/cmd_vel", 10);
}

BT::PortsList FollowPath::getBaseInputPorts()
{
  return  {BT::InputPort<double>("xy_exit_threshold_tiles"),
    BT::InputPort<double>("angle_exit_threshold_deg"),
    BT::InputPort<double>("lin_vel_exit_threshold_tps", 1000.0, ""),
    BT::InputPort<double>("ang_vel_exit_threshold_dps", 1000.0, ""),
    BT::InputPort<double>("xy_settling_radius_tiles"),
    BT::InputPort<double>("max_speed_linear_percent"),
    BT::InputPort<double>("max_speed_angular_percent"),
    BT::InputPort<int>("timeout_ms"),
    BT::InputPort<bool>("use_theta"),
  };
}

BT::NodeStatus FollowPath::onStart()
{
  first_loop_ = true;
  start_time_ = std::chrono::system_clock::now();
  settling_ = false;
  viz_msg_.markers.clear();

  // Get Blackboard Inputs
  xy_exit_threshold_m_ = BT_Util::get_input<double>(this, "xy_exit_threshold_tiles") * ghost_util::TILES_TO_METERS;
  angle_exit_threshold_rad_ = BT_Util::get_input<double>(this, "angle_exit_threshold_deg") * ghost_util::DEG_TO_RAD;
  lin_vel_exit_threshold_mps_ = BT_Util::get_input<double>(this, "lin_vel_exit_threshold_tps") * ghost_util::TILES_TO_METERS;
  ang_vel_exit_threshold_radps_ = BT_Util::get_input<double>(this, "ang_vel_exit_threshold_dps") * ghost_util::DEG_TO_RAD;
  max_speed_linear_percent_ = BT_Util::get_input<double>(this, "max_speed_linear_percent");
  max_speed_angular_percent_ = BT_Util::get_input<double>(this, "max_speed_angular_percent");
  timeout_ms_ = BT_Util::get_input<int>(this, "timeout_ms");
  use_theta_ = BT_Util::get_input<bool>(this, "use_theta");

  m_distance_approach_controller_ptr->reset();
  m_distance_settling_controller_ptr->reset();
  m_steering_approach_controller_ptr->reset();
  m_steering_settling_controller_ptr->reset();

  // Update local trajectory copy
  trajectory_ = *tank_trajectory_ptr_;

  goal_pose_ = Eigen::Vector3d(trajectory_.x.back(), trajectory_.y.back(), trajectory_.theta.back());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowPath::onRunning()
{
  if (checkEndConditions()) {
    tank_model_ptr_->driveCommandArcade(0.0, 0.0);
    return BT::NodeStatus::SUCCESS;
  }

  updateCurrentState();

  // Get control commands from derived class
  Eigen::Vector2d command = calculateControllerCommand();

  // Normalize to avoid saturation
  tank_model_ptr_->normalizeArcadeCommand(command);

  // Unpack and send final command to drivetrain
  fwd_command_ = command.x();
  turn_command_ = command.y();
  tank_model_ptr_->driveCommandArcade(fwd_command_, turn_command_);

  updateVisualization();

  return BT::NodeStatus::RUNNING;
}

void FollowPath::updateCurrentState()
{
  current_position_ = tank_model_ptr_->getWorldPose().head<2>();
  current_angle_ = tank_model_ptr_->getWorldPose().z();
  if (trajectory_.backwards) {
    current_angle_ = ghost_util::FlipAnglePI(current_angle_);
  }
  goal_pose_ = Eigen::Vector3d(trajectory_.x.back(), trajectory_.y.back(), trajectory_.theta.back());
}

void FollowPath::updateVisualization()
{
  viz_msg_.markers.clear();
  populateVisualizationMarkers();
  // visualization::getCircleMarker(viz_msg_, goal_pose_.head<2>(), xy_settling_radius_m_, visualization::getColorRGBA(1.0, 0.0, 0.0, 0.25), 0.0);

  publishExitThresholds();

  auto twist_msg = visualization::createTwistStampedMsg(Eigen::Vector3d(fwd_command_, 0.0, turn_command_));
  twist_command_pub_ptr_->publish(twist_msg);

  path_viz_pub_ptr_->publish(viz_msg_);
}

void FollowPath::publishExitThresholds()
{
  geometry_msgs::msg::PoseWithCovarianceStamped exit_threshold_msg;
  exit_threshold_msg.header.stamp = node_ptr_->now();
  exit_threshold_msg.header.frame_id = "map"; // Assuming "odom" is the relevant frame

  exit_threshold_msg.pose.pose.position.x = goal_pose_.x();
  exit_threshold_msg.pose.pose.position.y = goal_pose_.y();
  exit_threshold_msg.pose.pose.position.z = 0.0; // Assuming 2D planning, z is 0

  ghost_util::yawToQuaternionRad(
    goal_pose_.z(),
    exit_threshold_msg.pose.pose.orientation.w,
    exit_threshold_msg.pose.pose.orientation.x,
    exit_threshold_msg.pose.pose.orientation.y,
    exit_threshold_msg.pose.pose.orientation.z
  );

  // Set the diagonal elements for X, Y, and Yaw (Z-rotation)
  double xy_variance = std::pow(xy_exit_threshold_m_, 2);
  double angle_variance = std::pow(angle_exit_threshold_rad_, 2);

  exit_threshold_msg.pose.covariance[0] = xy_variance;
  exit_threshold_msg.pose.covariance[7] = xy_variance;
  exit_threshold_msg.pose.covariance[35] = angle_variance;

  exit_threshold_viz_pub_ptr_->publish(exit_threshold_msg);
}

bool FollowPath::checkEndConditions()
{
  double dist_err = (goal_pose_.head<2>() - tank_model_ptr_->getWorldPose().head<2>()).norm();
  double theta_err = std::fabs(ghost_util::SmallestAngleDistRad(goal_pose_.z(), tank_model_ptr_->getWorldAngleRad()));

  bool xy_satisfied = dist_err < xy_exit_threshold_m_;
  bool angle_satisfied = theta_err < angle_exit_threshold_rad_;
  bool xy_vel_satisfied = tank_model_ptr_->getWorldTwist().head<2>().norm() < lin_vel_exit_threshold_mps_;
  bool ang_vel_satisfied = std::fabs(tank_model_ptr_->getWorldTwist().z()) < ang_vel_exit_threshold_radps_;

  // Check exit conditions
  if (xy_satisfied && ang_vel_satisfied && xy_vel_satisfied) {
    bool translation_only = !use_theta_;
    if (translation_only || use_theta_ && angle_satisfied) {
      RCLCPP_INFO(node_ptr_->get_logger(), "MoveToPose: Success");
      return true;
    }
  }

  // Check timeout condition
  int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time_).count();
  if (time_elapsed > abs(timeout_ms_)) {
    RCLCPP_WARN(node_ptr_->get_logger(), "MoveToPose: Skipped");
    return true;
  }

  // Continue along the path
  return false;
}

void FollowPath::onHalted()
{
  tank_model_ptr_->driveCommandArcade(0.0, 0.0);
  resetStatus();
}

} // namespace ghost_tank
