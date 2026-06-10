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


#include "ghost_tank/bt_nodes/generate_planner_path.hpp"
#include <ghost_util/angle_util.hpp>
#include <ghost_util/unit_conversion_utils.hpp>
#include <ghost_tank/visualization/visualization_helpers.hpp>

namespace ghost_tank
{

GeneratePlannerPath::GeneratePlannerPath(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "tank_trajectory_ptr", tank_trajectory_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "planned_path_ptr", planned_path_ptr_);

  if (node_ptr_) {
    action_client_ptr_ = rclcpp_action::create_client<ComputePathToPose>(node_ptr_, "compute_path_to_pose");
    trajectory_viz_pub_ptr_ = node_ptr_->create_publisher<visualization_msgs::msg::MarkerArray>("/autonomy/current_tank_trajectory", 10);
  }
}

BT::PortsList GeneratePlannerPath::providedPorts()
{
  return {
    BT::InputPort<double>("end_x_tiles"),
    BT::InputPort<double>("end_y_tiles"),
    BT::InputPort<double>("end_theta_deg"),
    BT::InputPort<bool>("backwards", false, ""),
    BT::InputPort<std::string>("planner_id", "GridBased", ""),
    BT::InputPort<int>("timeout_ms", 2000, "")
  };
}

BT::NodeStatus GeneratePlannerPath::onStart()
{
  goal_accepted_ = false;
  start_time_ = std::chrono::system_clock::now();
  timeout_ms_ = BT_Util::get_input<int>(this, "timeout_ms");

  if (!action_client_ptr_ || !action_client_ptr_->action_server_is_ready()) {
    RCLCPP_WARN(node_ptr_->get_logger(), "[GeneratePlannerPath] planner_server action not available, skipping");
    return BT::NodeStatus::FAILURE;
  }

  // Goal pose from BT inputs, converted to map-frame meters/radians.
  // end_theta_rad_ / backwards_ are cached as members so storePlannedPath() can
  // stamp the commanded heading and drive direction onto the returned path.
  backwards_ = BT_Util::get_input<bool>(this, "backwards");
  double end_x_m = BT_Util::get_input<double>(this, "end_x_tiles") * ghost_util::TILES_TO_METERS;
  double end_y_m = BT_Util::get_input<double>(this, "end_y_tiles") * ghost_util::TILES_TO_METERS;
  end_theta_rad_ = BT_Util::get_input<double>(this, "end_theta_deg") * ghost_util::DEG_TO_RAD;

  if (backwards_) {
    end_theta_rad_ = ghost_util::FlipAnglePI(end_theta_rad_);
  }

  // Mirror goal about center line of VEX field (matches GenerateBezierPath).
  if (BT_Util::get_from_blackboard<bool>(blackboard_, "mirrored")) {
    end_x_m = 6.0 * ghost_util::TILES_TO_METERS - end_x_m;
    end_theta_rad_ = ghost_util::WrapAngle2PI(M_PI - end_theta_rad_);
  }

  // Start pose is the robot's current world pose.
  Eigen::Vector3d start_pose = tank_model_ptr_->getWorldPose();

  ComputePathToPose::Goal goal_msg;
  goal_msg.planner_id = BT_Util::get_input<std::string>(this, "planner_id");
  goal_msg.use_start = true;

  goal_msg.start.header.frame_id = "map";
  goal_msg.start.header.stamp = node_ptr_->now();
  goal_msg.start.pose.position.x = start_pose.x();
  goal_msg.start.pose.position.y = start_pose.y();
  ghost_util::yawToQuaternionRad(
    start_pose.z(),
    goal_msg.start.pose.orientation.w,
    goal_msg.start.pose.orientation.x,
    goal_msg.start.pose.orientation.y,
    goal_msg.start.pose.orientation.z);

  goal_msg.goal.header.frame_id = "map";
  goal_msg.goal.header.stamp = node_ptr_->now();
  goal_msg.goal.pose.position.x = end_x_m;
  goal_msg.goal.pose.position.y = end_y_m;
  ghost_util::yawToQuaternionRad(
    end_theta_rad_,
    goal_msg.goal.pose.orientation.w,
    goal_msg.goal.pose.orientation.x,
    goal_msg.goal.pose.orientation.y,
    goal_msg.goal.pose.orientation.z);

  // Fire the goal. Futures are polled in onRunning -- no callbacks needed.
  goal_handle_future_ = action_client_ptr_->async_send_goal(goal_msg);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GeneratePlannerPath::onRunning()
{
  // Timeout guard so a stuck planner can't hang the tree.
  int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now() - start_time_).count();
  if (time_elapsed > std::abs(timeout_ms_)) {
    RCLCPP_WARN(node_ptr_->get_logger(), "[GeneratePlannerPath] timed out waiting for path");
    return BT::NodeStatus::FAILURE;
  }

  // Stage 1: wait for the goal to be accepted, then request the result.
  if (!goal_accepted_) {
    if (goal_handle_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }
    GoalHandle::SharedPtr goal_handle = goal_handle_future_.get();
    if (!goal_handle) {
      RCLCPP_WARN(node_ptr_->get_logger(), "[GeneratePlannerPath] planner rejected goal");
      return BT::NodeStatus::FAILURE;
    }
    result_future_ = action_client_ptr_->async_get_result(goal_handle);
    goal_accepted_ = true;
    return BT::NodeStatus::RUNNING;
  }

  // Stage 2: wait for the computed path.
  if (result_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    return BT::NodeStatus::RUNNING;
  }

  GoalHandle::WrappedResult result = result_future_.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_WARN(node_ptr_->get_logger(), "[GeneratePlannerPath] planner failed to produce a path");
    return BT::NodeStatus::FAILURE;
  }

  // Stash the raw nav_msgs::Path verbatim for the nav2 controller branch
  // (FollowPathControllerServer consumes it with no conversion). Per-pose
  // orientations are meaningless tangents (SmacPlanner2D is a 2D planner); the
  // controller is configured with use_rotate_to_heading=false so it ignores them
  // and steers by lookahead geometry.
  if (planned_path_ptr_) {
    *planned_path_ptr_ = result.result->path;
  }

  // Convert the planned path into the shared Trajectory the pure-pursuit
  // follower consumes.
  if (!storePlannedPath(result.result->path)) {
    RCLCPP_WARN(node_ptr_->get_logger(), "[GeneratePlannerPath] planner returned an empty path");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

bool GeneratePlannerPath::storePlannedPath(const nav_msgs::msg::Path & path)
{
  const auto & poses = path.poses;
  if (poses.empty() || !tank_trajectory_ptr_) {
    return false;
  }

  // The planner already runs in the true map frame on the (already
  // mirrored/back-flipped) goal, so the returned poses need no further
  // mirroring -- unlike GenerateBezierPath / LoadPathFromCSV which transform
  // their inputs before generating.
  const int num_points = static_cast<int>(poses.size());
  motion_planning::Trajectory traj(num_points);

  for (int i = 0; i < num_points; ++i) {
    traj.x[i] = poses[i].pose.position.x;
    traj.y[i] = poses[i].pose.position.y;
    // Path tangent heading; viz-only for pure pursuit (only theta.back() is read).
    traj.theta[i] = ghost_util::quaternionToYawRad(
      poses[i].pose.orientation.w,
      poses[i].pose.orientation.x,
      poses[i].pose.orientation.y,
      poses[i].pose.orientation.z);
    // Normalized parametric stamp; unused by pure pursuit but required non-empty
    // and equal-length for calculateRemainingPathLengths().
    traj.t[i] = static_cast<double>(i) / static_cast<double>(num_points);
    // omega left 0 -- a geometric global plan carries no angular velocity profile.
  }

  // Settle to the commanded goal heading, not the planner's final tangent.
  traj.theta.back() = end_theta_rad_;
  traj.backwards = backwards_;
  traj.calculateRemainingPathLengths();

  *tank_trajectory_ptr_ = traj;

  if (node_ptr_) {
    viz_msg_.markers.clear();
    visualization::getTrajectoryMsg(traj, viz_msg_, 10);
    trajectory_viz_pub_ptr_->publish(viz_msg_);
  }

  return true;
}

void GeneratePlannerPath::onHalted()
{
  resetStatus();
}

} // namespace ghost_tank
