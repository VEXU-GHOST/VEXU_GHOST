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

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/behavior_tree.h"
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "ghost_tank/bt_nodes/bt_util.hpp"

namespace ghost_tank
{

// Listens for navigation goals on /goal_pose and hands them to a planner node
// (e.g. GeneratePlannerPath) through the blackboard.
//
// The node idles (returns RUNNING) while no new pose has arrived, so a tree
// can loop on it forever without triggering a plan. When a fresh PoseStamped
// is received, it converts the map-frame pose into the tile/degree convention
// used by the other BT nodes, writes it to the output ports, and returns
// SUCCESS exactly once for that pose.
class GoalPoseListener : public BT::StatefulActionNode
{
public:
  // If your Node has ports, you must use this constructor signature
  GoalPoseListener(const std::string & name, const BT::NodeConfig & config);

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

  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

private:
  std::shared_ptr<rclcpp::Node> node_ptr_;
  BT::Blackboard::Ptr blackboard_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  geometry_msgs::msg::PoseStamped latest_goal_;
  bool has_new_goal_{false};
};

} // namespace ghost_tank
