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


#include "ghost_tank/bt_nodes/goal_pose_listener.hpp"
#include <ghost_util/angle_util.hpp>
#include <ghost_util/unit_conversion_utils.hpp>

namespace ghost_tank
{

GoalPoseListener::GoalPoseListener(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);

  if (node_ptr_) {
    goal_pose_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {this->goalPoseCallback(msg);});
  }
}

void GoalPoseListener::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // Cache the most recent goal; onRunning consumes it on the next tick.
  latest_goal_ = *msg;
  has_new_goal_ = true;
}

BT::PortsList GoalPoseListener::providedPorts()
{
  return {
    BT::OutputPort<double>("goal_x_tiles"),
    BT::OutputPort<double>("goal_y_tiles"),
    BT::OutputPort<double>("goal_theta_deg")
  };
}

BT::NodeStatus GoalPoseListener::onStart()
{
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoalPoseListener::onRunning()
{
  // Idle until a fresh pose shows up on /goal_pose.
  if (!has_new_goal_) {
    return BT::NodeStatus::RUNNING;
  }
  has_new_goal_ = false;

  // /goal_pose is map-frame meters; convert to the tile/degree convention the
  // planner nodes expect (GeneratePlannerPath converts these back to meters/radians).
  double yaw_deg = ghost_util::quaternionToYawDeg(
    latest_goal_.pose.orientation.w,
    latest_goal_.pose.orientation.x,
    latest_goal_.pose.orientation.y,
    latest_goal_.pose.orientation.z);

  setOutput("goal_x_tiles", latest_goal_.pose.position.x / ghost_util::TILES_TO_METERS);
  setOutput("goal_y_tiles", latest_goal_.pose.position.y / ghost_util::TILES_TO_METERS);
  setOutput("goal_theta_deg", yaw_deg);

  return BT::NodeStatus::SUCCESS;
}

void GoalPoseListener::onHalted()
{
  resetStatus();
}

} // namespace ghost_tank
