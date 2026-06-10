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

// StatefulActionNode that drives a planned path using the nav2 controller_server
// instead of the in-house pure pursuit. It is the controller-server analogue of
// FollowPathPurePursuit, and is fully self-contained: it ships the planned
// nav_msgs::Path (handed over verbatim by GeneratePlannerPath on the
// "planned_path_ptr" blackboard entry -- no Path<->Trajectory conversion) to the
// controller_server via the FollowPath action, subscribes to the cmd_vel the
// controller publishes, and forwards that velocity to the tank drive each tick.
//
// Non-blocking: onStart() sends the action goal, onRunning() polls the
// goal/result futures across ticks while continuously relaying cmd_vel.
class FollowPathControllerServer : public BT::StatefulActionNode
{
public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;

  FollowPathControllerServer(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  // Relays the most recent controller cmd_vel to the tank drive as a normalized
  // arcade command (velocity / max velocity -> effort fraction).
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
  // Static-friction voltage floors: minimum |voltage fraction| applied for any
  // nonzero velocity command, so the controller can actually break friction and
  // move (otherwise low commands stall and RPP deadlocks on measured vel ~0).
  double kff_linear_{0.12};
  double kff_angular_{0.20};
};

} // namespace ghost_tank
