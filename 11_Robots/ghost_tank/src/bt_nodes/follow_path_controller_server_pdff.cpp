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

#include "ghost_tank/bt_nodes/follow_path_controller_server_pdff.hpp"
#include <ghost_tank/visualization/visualization_helpers.hpp>

namespace ghost_tank
{

FollowPathControllerServerPDFF::FollowPathControllerServerPDFF(
  const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "velocity_controller_ptr", velocity_controller_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "planned_path_ptr", planned_path_ptr_);

  if (node_ptr_) {
    action_client_ptr_ = rclcpp_action::create_client<FollowPath>(node_ptr_, "follow_path");
    path_plan_pub_ptr_ = node_ptr_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/plan/controller", 10);
  }
}

BT::PortsList FollowPathControllerServerPDFF::providedPorts()
{
  return {
    BT::InputPort<std::string>("cmd_vel_topic", "/nav2/cmd_vel", ""),
    BT::InputPort<std::string>("controller_id", "FollowPath", ""),
    BT::InputPort<std::string>("goal_checker_id", "goal_checker", ""),
    BT::InputPort<int>("timeout_ms", 10000, "")
    // PD+FF gains and static feedforward (kS) are not ports: they come from the
    // shared velocity_controller_ptr_ (robot config velocity_linear/_angular).
  };
}

BT::NodeStatus FollowPathControllerServerPDFF::onStart()
{
  goal_accepted_ = false;
  have_cmd_vel_ = false;
  goal_handle_.reset();
  start_time_ = std::chrono::system_clock::now();
  prev_time_ = start_time_;
  timeout_ms_ = BT_Util::get_input<int>(this, "timeout_ms");
  velocity_controller_ptr_->reset();

  // Lazily create the cmd_vel subscription now that the input port is available.
  if (!cmd_vel_sub_ && node_ptr_) {
    std::string cmd_vel_topic = BT_Util::get_input<std::string>(this, "cmd_vel_topic");
    cmd_vel_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {this->cmdVelCallback(msg);});
  }

  if (!action_client_ptr_ || !action_client_ptr_->action_server_is_ready()) {
    RCLCPP_WARN(node_ptr_->get_logger(), "[FollowPathControllerServerPDFF] controller_server action not available, skipping");
    return BT::NodeStatus::FAILURE;
  }

  if (!planned_path_ptr_ || planned_path_ptr_->poses.empty()) {
    RCLCPP_WARN(node_ptr_->get_logger(), "[FollowPathControllerServerPDFF] no planned path available");
    return BT::NodeStatus::FAILURE;
  }

  // Print the final heading the controller will try to settle on: the yaw of the
  // last path pose. This is the goal yaw SmacPlanner2D stamped on the path and the
  // angle RPP rotates in place to at the end (when use_rotate_to_heading is on).
  // Logged here so we can see what "final heading" actually is when it misbehaves.
  {
    const auto & q = planned_path_ptr_->poses.back().pose.orientation;
    double final_yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "[FollowPathControllerServerPDFF] final heading (goal yaw) = %.3f rad (%.1f deg)",
      final_yaw, final_yaw * 180.0 / M_PI);
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

BT::NodeStatus FollowPathControllerServerPDFF::onRunning()
{
  // Timeout guard so a stuck controller can't hang the tree.
  int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now() - start_time_).count();
  if (time_elapsed > std::abs(timeout_ms_)) {
    RCLCPP_WARN(node_ptr_->get_logger(), "[FollowPathControllerServerPDFF] timed out following path");
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
      RCLCPP_WARN(node_ptr_->get_logger(), "[FollowPathControllerServerPDFF] controller rejected goal");
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
    RCLCPP_WARN(node_ptr_->get_logger(), "[FollowPathControllerServerPDFF] controller failed to reach goal");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

void FollowPathControllerServerPDFF::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  latest_cmd_vel_ = *msg;
  have_cmd_vel_ = true;
}

void FollowPathControllerServerPDFF::forwardLatestCmdVel()
{
  if (!have_cmd_vel_) {
    return;
  }

  // Commanded and measured chassis velocity, both as a normalized fraction of
  // the chassis max velocity. Measured linear speed is the world-twist magnitude
  // and measured angular is its z component, matching FollowPathPurePursuit.
  double lin_cmd_frac = latest_cmd_vel_.linear.x / tank_model_ptr_->getMaxBaseLinearVelocity();
  double ang_cmd_frac = latest_cmd_vel_.angular.z / tank_model_ptr_->getMaxBaseAngularVelocity();
  double lin_meas_frac = tank_model_ptr_->getWorldTwist().head<2>().norm() /
    tank_model_ptr_->getMaxBaseLinearVelocity();
  double ang_meas_frac = tank_model_ptr_->getWorldTwist().z() /
    tank_model_ptr_->getMaxBaseAngularVelocity();

  // dt since the previous tick for the controller's D term. The shared
  // VelocityController owns the PD+FF gains, error-derivative history, and the
  // static feedforward (kS) (config velocity_linear/_angular), matching MoveVelocityPDFF.
  auto now = std::chrono::system_clock::now();
  double dt = std::chrono::duration<double>(now - prev_time_).count();
  prev_time_ = now;

  Eigen::Vector2d command = velocity_controller_ptr_->calculateCommand(
    lin_cmd_frac, lin_meas_frac, ang_cmd_frac, ang_meas_frac, dt);
  tank_model_ptr_->normalizeArcadeCommand(command);
  tank_model_ptr_->driveCommandArcade(command.x(), command.y());
}

void FollowPathControllerServerPDFF::stopMotors()
{
  tank_model_ptr_->driveCommandArcade(0.0, 0.0);
}

void FollowPathControllerServerPDFF::publishPlannedPath(const nav_msgs::msg::Path & path)
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

void FollowPathControllerServerPDFF::onHalted()
{
  // Cancel the controller goal so it stops driving when the tree halts us.
  if (action_client_ptr_ && goal_handle_) {
    action_client_ptr_->async_cancel_goal(goal_handle_);
  }
  stopMotors();
  resetStatus();
}

} // namespace ghost_tank
