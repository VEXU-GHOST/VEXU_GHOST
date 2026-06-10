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

#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "ghost_tank/bt_nodes/bt_util.hpp"
#include "ghost_tank/control/trajectory.hpp"

namespace ghost_tank
{

// StatefulActionNode that asks the nav2 planner_server to plan a path from the
// robot's current world pose to a goal (x, y, theta) supplied by the behavior
// tree, then converts the resulting nav_msgs::Path into the
// motion_planning::Trajectory consumed by FollowPath / FollowPathPurePursuit.
// This is the planner-backed analogue of GenerateBezierPath: on SUCCESS the
// trajectory is written to the shared "tank_trajectory_ptr" blackboard entry so
// a downstream FollowPath* node drives it, same as any other path producer.
//
// The node is non-blocking: onStart() fires the ComputePathToPose action goal,
// and onRunning() polls the goal/result futures across ticks. The tree is
// ticked once per plugin cycle, so blocking here would stall the executor that
// services these futures.
class GeneratePlannerPath : public BT::StatefulActionNode
{
public:
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ComputePathToPose>;

  // If your Node has ports, you must use this constructor signature
  GeneratePlannerPath(const std::string & name, const BT::NodeConfig & config);

  // It is mandatory to define this STATIC method.
  static BT::PortsList providedPorts();

  /// Method called once, when transitioning from the state IDLE.
  /// If it returns RUNNING, this becomes an asynchronous node.
  BT::NodeStatus onStart() override;

  /// method invoked when the action is already in the RUNNING state.
  BT::NodeStatus onRunning() override;

  /// when the method halt() is called and the action is RUNNING, this method is invoked.
  /// This is a convenient place todo a cleanup, if needed.
  void onHalted() override;

private:
  // Converts the planned nav_msgs::Path into a motion_planning::Trajectory and
  // writes it to *tank_trajectory_ptr_ for the downstream follower. Returns
  // false if the path is empty.
  bool storePlannedPath(const nav_msgs::msg::Path & path);

  std::shared_ptr<rclcpp::Node> node_ptr_;
  std::shared_ptr<TankModel> tank_model_ptr_;
  std::shared_ptr<motion_planning::Trajectory> tank_trajectory_ptr_;
  // Raw planner output, stashed verbatim for the nav2 controller branch
  // (FollowPathControllerServer) so no Path<->Trajectory conversion is needed.
  std::shared_ptr<nav_msgs::msg::Path> planned_path_ptr_;
  BT::Blackboard::Ptr blackboard_;

  // Republishes the converted trajectory for RViz, matching GenerateBezierPath.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_viz_pub_ptr_;
  visualization_msgs::msg::MarkerArray viz_msg_;

  // Action client to planner_server -- calling it triggers the plan.
  rclcpp_action::Client<ComputePathToPose>::SharedPtr action_client_ptr_;

  // Async handles polled across ticks.
  std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
  std::shared_future<GoalHandle::WrappedResult> result_future_;
  bool goal_accepted_{false};

  // Config / goal state computed in onStart(), consumed when the path arrives.
  std::chrono::time_point<std::chrono::system_clock> start_time_;
  int timeout_ms_{0};
  double end_theta_rad_{0.0};  // commanded goal heading (post mirror/backwards)
  bool backwards_{false};      // drive the planned path in reverse
};

} // namespace ghost_tank
