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
    BT::InputPort<int>("timeout_ms", 10000, ""),
    BT::InputPort<double>("p_linear", 0.0, "proportional gain on linear velocity error"),
    BT::InputPort<double>("d_linear", 0.0, "derivative gain on linear velocity error"),
    BT::InputPort<double>("ff_linear", 0.10, "feedforward gain on commanded linear velocity"),
    BT::InputPort<double>("p_angular", 0.0, "proportional gain on angular velocity error"),
    BT::InputPort<double>("d_angular", 0.0, "derivative gain on angular velocity error"),
    BT::InputPort<double>("ff_angular", 0.19, "feedforward gain on commanded angular velocity"),
    BT::InputPort<double>("floor_linear", 0.0, "static-friction voltage floor, linear (0 disables)"),
    BT::InputPort<double>("floor_angular", 0.0, "static-friction voltage floor, angular (0 disables)")
  };
}

BT::NodeStatus FollowPathControllerServerPDFF::onStart()
{
  goal_accepted_ = false;
  have_cmd_vel_ = false;
  have_prev_ = false;
  goal_handle_.reset();
  start_time_ = std::chrono::system_clock::now();
  timeout_ms_ = BT_Util::get_input<int>(this, "timeout_ms");
  p_linear_ = BT_Util::get_input<double>(this, "p_linear");
  d_linear_ = BT_Util::get_input<double>(this, "d_linear");
  ff_linear_ = BT_Util::get_input<double>(this, "ff_linear");
  p_angular_ = BT_Util::get_input<double>(this, "p_angular");
  d_angular_ = BT_Util::get_input<double>(this, "d_angular");
  ff_angular_ = BT_Util::get_input<double>(this, "ff_angular");
  floor_linear_ = BT_Util::get_input<double>(this, "floor_linear");
  floor_angular_ = BT_Util::get_input<double>(this, "floor_angular");

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

  // PD on velocity error + velocity feedforward, per axis.
  double lin_err = lin_cmd_frac - lin_meas_frac;
  double ang_err = ang_cmd_frac - ang_meas_frac;

  // Derivative of error; zero on the first tick (no prior sample / dt yet).
  auto now = std::chrono::system_clock::now();
  double dt = std::chrono::duration<double>(now - prev_time_).count();
  double lin_derr = 0.0;
  double ang_derr = 0.0;
  if (have_prev_ && dt > 1.0e-6) {
    lin_derr = (lin_err - prev_lin_err_) / dt;
    ang_derr = (ang_err - prev_ang_err_) / dt;
  }
  prev_lin_err_ = lin_err;
  prev_ang_err_ = ang_err;
  prev_time_ = now;
  have_prev_ = true;

  double fwd = ff_linear_ * lin_cmd_frac + p_linear_ * lin_err + d_linear_ * lin_derr;
  double ang = ff_angular_ * ang_cmd_frac + p_angular_ * ang_err + d_angular_ * ang_derr;

  // Static-friction floor: if there is a velocity command on this axis but the
  // PD+FF output is too weak to break loose, raise the output magnitude to the
  // floor (keeping the output's own sign). A ~zero command leaves the output
  // untouched so the drive can still brake. floor == 0 disables this.
  const auto with_floor = [](double out, double cmd_frac, double floor) {
    if (floor <= 0.0 || std::fabs(cmd_frac) < 1.0e-4 || std::fabs(out) >= floor) {
      return out;
    }
    return std::copysign(floor, out != 0.0 ? out : cmd_frac);
  };
  fwd = with_floor(fwd, lin_cmd_frac, floor_linear_);
  ang = with_floor(ang, ang_cmd_frac, floor_angular_);

  Eigen::Vector2d command(fwd, ang);
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
