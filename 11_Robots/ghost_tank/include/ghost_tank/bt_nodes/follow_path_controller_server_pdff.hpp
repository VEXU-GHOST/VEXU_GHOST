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

#pragma once

#include <chrono>
#include <future>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "behaviortree_cpp/behavior_tree.h"

#include <nav2_msgs/action/follow_path.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "ghost_tank/bt_nodes/bt_util.hpp"

namespace ghost_tank
{

// Closed-loop variant of FollowPathControllerServer. Identical plumbing -- ships
// the planned path to the nav2 controller_server and relays the cmd_vel it
// publishes -- but instead of an open-loop feedforward floor it runs a per-axis
// PD controller on velocity error (commanded cmd_vel vs. the tank's measured
// world twist) plus a velocity feedforward term. Output is the normalized arcade
// command sent to the drive.
//
// Per axis: cmd = ff * cmd_frac + p * (cmd_frac - meas_frac) + d * d/dt(err)
// where *_frac is velocity normalized by the chassis max velocity.
//
// Non-blocking: onStart() sends the action goal, onRunning() polls the
// goal/result futures across ticks while continuously relaying cmd_vel.
class FollowPathControllerServerPDFF : public BT::StatefulActionNode
{
public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;

  FollowPathControllerServerPDFF(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  // Runs the per-axis PD+FF controller on the latest controller cmd_vel and
  // forwards the result to the tank drive as a normalized arcade command.
  void forwardLatestCmdVel();
  void stopMotors();
  // Publishes the path being followed on /plan/controller: start/end pose arrows
  // and points for the middle waypoints, mirroring FollowPathPurePursuit.
  void publishPlannedPath(const nav_msgs::msg::Path & path);

  std::shared_ptr<rclcpp::Node> node_ptr_;
  std::shared_ptr<TankModel> tank_model_ptr_;
  std::shared_ptr<nav_msgs::msg::Path> planned_path_ptr_;
  BT::Blackboard::Ptr blackboard_;

  // Action client to controller_server -- sending a goal starts path tracking.
  rclcpp_action::Client<FollowPath>::SharedPtr action_client_ptr_;
  // cmd_vel published by controller_server; created lazily in onStart() so the
  // topic input port is available.
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  // Debug visualization of the path being tracked.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_plan_pub_ptr_;

  // Async handles polled across ticks.
  std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
  std::shared_future<GoalHandle::WrappedResult> result_future_;
  GoalHandle::SharedPtr goal_handle_;
  bool goal_accepted_{false};

  // Latest controller velocity command, cached by the subscription callback.
  geometry_msgs::msg::Twist latest_cmd_vel_;
  bool have_cmd_vel_{false};

  // Config read on each onStart().
  std::chrono::time_point<std::chrono::system_clock> start_time_;
  int timeout_ms_{0};
  // Per-axis PD-on-velocity-error gains plus a velocity feedforward gain. All
  // act on velocities normalized to a fraction of the chassis max velocity.
  double p_linear_{0.0};
  double d_linear_{0.0};
  double ff_linear_{0.10};
  double p_angular_{0.0};
  double d_angular_{0.0};
  double ff_angular_{0.19};
  // Static-friction voltage floor: for any nonzero velocity command, the output
  // magnitude is raised to at least this fraction so the drive can break loose
  // (otherwise low commands stall and RPP deadlocks on measured vel ~0). Set to
  // 0 to disable.
  double floor_linear_{0.0};
  double floor_angular_{0.0};

  // Derivative state for the velocity-error D term, reset on each onStart().
  double prev_lin_err_{0.0};
  double prev_ang_err_{0.0};
  std::chrono::time_point<std::chrono::system_clock> prev_time_;
  bool have_prev_{false};
};

} // namespace ghost_tank
