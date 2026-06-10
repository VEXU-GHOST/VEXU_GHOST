/*
 *   Copyright (c) 2025 Karmanyaah Malhotra
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


#include <cmath>

#include "ghost_tank/bt_nodes/follow_path_controller_server.hpp"
#include <ghost_tank/visualization/visualization_helpers.hpp>

namespace ghost_tank
{

FollowPathControllerServer::FollowPathControllerServer(
  const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "planned_path_ptr", planned_path_ptr_);

  if (node_ptr_) {
    action_client_ptr_ = rclcpp_action::create_client<FollowPath>(node_ptr_, "follow_path");
    path_plan_pub_ptr_ = node_ptr_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/plan/controller", 10);
  }
}

BT::PortsList FollowPathControllerServer::providedPorts()
{
  return {
    BT::InputPort<std::string>("cmd_vel_topic", "/nav2/cmd_vel", ""),
    BT::InputPort<std::string>("controller_id", "FollowPath", ""),
    BT::InputPort<std::string>("goal_checker_id", "goal_checker", ""),
    BT::InputPort<int>("timeout_ms", 10000, ""),
    BT::InputPort<double>("kff_linear", 0.12, "static-friction voltage floor, linear"),
    BT::InputPort<double>("kff_angular", 0.20, "static-friction voltage floor, angular")
  };
}

BT::NodeStatus FollowPathControllerServer::onStart()
{
  goal_accepted_ = false;
  have_cmd_vel_ = false;
  goal_handle_.reset();
  start_time_ = std::chrono::system_clock::now();
  timeout_ms_ = BT_Util::get_input<int>(this, "timeout_ms");
  kff_linear_ = BT_Util::get_input<double>(this, "kff_linear");
  kff_angular_ = BT_Util::get_input<double>(this, "kff_angular");

  // Lazily create the cmd_vel subscription now that the input port is available.
  if (!cmd_vel_sub_ && node_ptr_) {
    std::string cmd_vel_topic = BT_Util::get_input<std::string>(this, "cmd_vel_topic");
    cmd_vel_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {this->cmdVelCallback(msg);});
  }

  if (!action_client_ptr_ || !action_client_ptr_->action_server_is_ready()) {
    RCLCPP_WARN(node_ptr_->get_logger(), "[FollowPathControllerServer] controller_server action not available, skipping");
    return BT::NodeStatus::FAILURE;
  }

  if (!planned_path_ptr_ || planned_path_ptr_->poses.empty()) {
    RCLCPP_WARN(node_ptr_->get_logger(), "[FollowPathControllerServer] no planned path available");
    return BT::NodeStatus::FAILURE;
  }

  publishPlannedPath(*planned_path_ptr_);

  FollowPath::Goal goal_msg;
  goal_msg.path = *planned_path_ptr_;
  goal_msg.controller_id = BT_Util::get_input<std::string>(this, "controller_id");
  goal_msg.goal_checker_id = BT_Util::get_input<std::string>(this, "goal_checker_id");

  // Fire the goal. Futures are polled in onRunning -- no callbacks needed.
  goal_handle_future_ = action_client_ptr_->async_send_goal(goal_msg);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowPathControllerServer::onRunning()
{
  // Timeout guard so a stuck controller can't hang the tree.
  int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now() - start_time_).count();
  if (time_elapsed > std::abs(timeout_ms_)) {
    RCLCPP_WARN(node_ptr_->get_logger(), "[FollowPathControllerServer] timed out following path");
    stopMotors();
    return BT::NodeStatus::FAILURE;
  }

  // Stage 1: wait for the goal to be accepted, then request the result.
  if (!goal_accepted_) {
    if (goal_handle_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }
    goal_handle_ = goal_handle_future_.get();
    if (!goal_handle_) {
      RCLCPP_WARN(node_ptr_->get_logger(), "[FollowPathControllerServer] controller rejected goal");
      stopMotors();
      return BT::NodeStatus::FAILURE;
    }
    result_future_ = action_client_ptr_->async_get_result(goal_handle_);
    goal_accepted_ = true;
    return BT::NodeStatus::RUNNING;
  }

  // Stage 2: relay cmd_vel while the controller drives, and watch for completion.
  if (result_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    forwardLatestCmdVel();
    return BT::NodeStatus::RUNNING;
  }

  GoalHandle::WrappedResult result = result_future_.get();
  stopMotors();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_WARN(node_ptr_->get_logger(), "[FollowPathControllerServer] controller failed to reach goal");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

void FollowPathControllerServer::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  latest_cmd_vel_ = *msg;
  have_cmd_vel_ = true;
}

void FollowPathControllerServer::forwardLatestCmdVel()
{
  if (!have_cmd_vel_) {
    return;
  }

  // Map desired chassis velocity (m/s, rad/s) to a normalized voltage fraction.
  double fwd_frac = latest_cmd_vel_.linear.x / tank_model_ptr_->getMaxBaseLinearVelocity();
  double ang_frac = latest_cmd_vel_.angular.z / tank_model_ptr_->getMaxBaseAngularVelocity();

  // Static-friction floor: driveCommandArcade sets motor *voltage* percent, and a
  // small velocity command maps to a voltage too low to break static friction --
  // the robot stalls and RPP deadlocks (it accel-limits from measured vel ~0, so
  // it never ramps up). Map any nonzero command to at least the floor voltage,
  // scaling linearly up to full at max velocity. Sign-symmetric, so reverse gets
  // the same breakaway as forward; angular uses a higher floor since rotating a
  // tank in place needs more voltage than driving straight.
  const auto with_floor = [](double frac, double floor) {
    if (std::fabs(frac) < 1.0e-4) {
      return 0.0;
    }
    double mag = std::min(std::fabs(frac), 1.0);
    return std::copysign(floor + (1.0 - floor) * mag, frac);
  };

  double fwd = with_floor(fwd_frac, kff_linear_);
  double ang = with_floor(ang_frac, kff_angular_);

  Eigen::Vector2d command(fwd, ang);
  tank_model_ptr_->normalizeArcadeCommand(command);
  tank_model_ptr_->driveCommandArcade(command.x(), command.y());
}

void FollowPathControllerServer::stopMotors()
{
  tank_model_ptr_->driveCommandArcade(0.0, 0.0);
}

void FollowPathControllerServer::publishPlannedPath(const nav_msgs::msg::Path & path)
{
  if (!path_plan_pub_ptr_ || path.poses.empty()) {
    return;
  }

  visualization_msgs::msg::MarkerArray plan_msg;

  // Clear any markers from a previous (possibly longer) plan.
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  plan_msg.markers.push_back(clear_marker);

  // Builds an orientation arrow straight from a path pose (already carries a quaternion).
  const auto append_pose_arrow =
    [&plan_msg](const geometry_msgs::msg::Pose & pose,
      const std_msgs::msg::ColorRGBA & color, const std::string & ns) {
      visualization_msgs::msg::Marker arrow;
      arrow.header.frame_id = "map";
      arrow.header.stamp = rclcpp::Clock().now();
      arrow.ns = ns;
      arrow.id = static_cast<int>(plan_msg.markers.size());
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;
      arrow.pose = pose;
      arrow.pose.position.z = visualization::MARKER_Z_OFFSET;
      arrow.scale.x = 0.20;   // shaft length
      arrow.scale.y = 0.03;   // shaft diameter
      arrow.scale.z = 0.06;   // head diameter
      arrow.color = color;
      plan_msg.markers.push_back(arrow);
    };

  const int n = static_cast<int>(path.poses.size());

  // Middle waypoints as plain points (cyan).
  for (int i = 1; i < n - 1; ++i) {
    visualization::getPointMarker(
      plan_msg,
      Eigen::Vector2d(path.poses[i].pose.position.x, path.poses[i].pose.position.y),
      visualization::getColorRGBA(0.0, 0.8, 1.0, 0.8),
      visualization::MARKER_SPHERE_DIAM,
      visualization::MARKER_Z_OFFSET,
      "controller_plan_points");
  }

  // Start pose arrow (green) and end/goal pose arrow (red).
  append_pose_arrow(
    path.poses.front().pose,
    visualization::getColorRGBA(0.0, 1.0, 0.0, 1.0),
    "controller_plan_start");
  append_pose_arrow(
    path.poses.back().pose,
    visualization::getColorRGBA(1.0, 0.0, 0.0, 1.0),
    "controller_plan_end");

  path_plan_pub_ptr_->publish(plan_msg);
}

void FollowPathControllerServer::onHalted()
{
  // Cancel the controller goal so it stops driving when the tree halts us.
  if (action_client_ptr_ && goal_handle_) {
    action_client_ptr_->async_cancel_goal(goal_handle_);
  }
  stopMotors();
  resetStatus();
}

} // namespace ghost_tank
