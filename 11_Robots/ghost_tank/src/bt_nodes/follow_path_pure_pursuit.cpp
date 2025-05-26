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


#include "ghost_tank/bt_nodes/follow_path_pure_pursuit.hpp"

namespace ghost_tank
{

FollowPathPurePursuit::FollowPathPurePursuit(const std::string & name, const BT::NodeConfig & config)
: FollowPath(name, config)
{
}

BT::PortsList FollowPathPurePursuit::providedPorts()
{
  // auto input_ports = FollowPath::getBaseInputPorts();
  // input_ports.insert(BT::InputPort<double>("lookahead_distance_tiles"));
  // return input_ports;

  return  {BT::InputPort<double>("xy_exit_threshold_tiles"),
    BT::InputPort<double>("angle_exit_threshold_deg"),
    BT::InputPort<double>("lin_vel_exit_threshold_tps", 1000.0, ""),
    BT::InputPort<double>("ang_vel_exit_threshold_dps", 1000.0, ""),
    BT::InputPort<double>("max_speed_linear_percent"),
    BT::InputPort<double>("max_speed_angular_percent"),
    BT::InputPort<int>("timeout_ms"),
    BT::InputPort<bool>("use_theta"),
    BT::InputPort<bool>("backwards"),
    BT::InputPort<double>("lookahead_distance_tiles")
  };
}

BT::NodeStatus FollowPathPurePursuit::onStart()
{
  // Call base class onStart to initialize common parameters
  BT::NodeStatus status = FollowPath::onStart();
  if (status != BT::NodeStatus::RUNNING) {
    return status; // Return if base initialization failed or is not running
  }

  // Get Pure Pursuit specific parameters
  lookahead_distance_m_ = BT_Util::get_input<double>(this, "lookahead_distance_tiles") * ghost_util::TILES_TO_METERS;

  return BT::NodeStatus::RUNNING;
}

Eigen::Vector2d FollowPathPurePursuit::calculateControllerCommand()
{
  // Get current and terminal states
  Eigen::Vector2d current_pos = Eigen::Vector2d(tank_model_ptr_->getWorldPose().head<2>());
  goal_pose_ = Eigen::Vector3d(trajectory_.x.back(), trajectory_.y.back(), trajectory_.theta.back());

  // Find closest point in path
  int index = trajectory_.getIndexOfClosestPoint(current_pos);
  double dist_to_end = trajectory_.remaining_path_length[index];

  Eigen::Vector2d desired_pose;

  if (dist_to_end <= lookahead_distance_m_) {
    desired_pose = goal_pose_.head<2>();
  } else {
    auto carrot_dist_from_end = trajectory_.remaining_path_length[index] - lookahead_distance_m_;
    auto carrot_index = std::lower_bound(
      trajectory_.remaining_path_length.begin() + index,
      trajectory_.remaining_path_length.end(),
      carrot_dist_from_end) - trajectory_.remaining_path_length.begin();
    desired_pose = Eigen::Vector2d(trajectory_.x[carrot_index], trajectory_.y[carrot_index]);
  }

  // Select control strategy based on distance to target
  double dist_err = (goal_pose_.head<2>() - current_pos).norm();
  bool within_pursuit_radius = dist_err < lookahead_distance_m_;
  bool within_xy_exit_threshold = dist_err < xy_exit_threshold_m_;

  TankState current_state(dist_to_end, tank_model_ptr_->getWorldTwist().head<2>().norm(), tank_model_ptr_->getWorldPose().z(), tank_model_ptr_->getWorldTwist().z());
  TankState desired_state(0.0, 0.0, 0.0, 0.0);

  Eigen::Vector2d command;
  if (within_xy_exit_threshold || settling_) {
    command = m_settling_controller_ptr->calculateDriveCommand(current_state, desired_state, backwards_);

    // Once we start settling, never exit to avoid instability.
    settling_ = true;
  } else {
    command = m_approach_controller_ptr->calculateDriveCommand(current_state, desired_state, backwards_);
  }

  return command;
}

void FollowPathPurePursuit::populateVisualizationMarkers()
{

}


} // namespace ghost_tank {
