/*
 * Copyright (c) 2025 Jake Wendling, Maxx Wilson
 * All rights reserved.

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


#include "ghost_tank/bt_nodes/follow_path_pure_pursuit.hpp"
#include <Eigen/Geometry>

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
    BT::InputPort<double>("lookahead_distance_tiles"), // This will become the default/min lookahead
    BT::InputPort<double>("k_lookahead", 0.0, "Gain for dynamic lookahead (m/mps)"), // NEW
    BT::InputPort<double>("min_lookahead_distance_tiles", 0.0, "Minimum lookahead distance (tiles)") // NEW
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
  // The 'lookahead_distance_tiles' input will now serve as a default or base for min_lookahead_distance_m_
  // if min_lookahead_distance_tiles is not explicitly set.
  lookahead_distance_m_ = BT_Util::get_input<double>(this, "lookahead_distance_tiles") * ghost_util::TILES_TO_METERS;
  k_lookahead_ = BT_Util::get_input<double>(this, "k_lookahead");
  min_lookahead_distance_m_ = BT_Util::get_input<double>(this, "min_lookahead_distance_tiles") * ghost_util::TILES_TO_METERS;

  // If min_lookahead_distance_m_ was not provided, use the old lookahead_distance_m_ as the minimum
  if (min_lookahead_distance_m_ == 0.0) { // Assuming 0.0 is the default value if not provided
      min_lookahead_distance_m_ = lookahead_distance_m_;
  }

  return BT::NodeStatus::RUNNING;
}

Eigen::Vector2d FollowPathPurePursuit::calculateControllerCommand()
{
  // Get current robot pose and linear speed
  Eigen::Vector3d current_robot_pose = tank_model_ptr_->getWorldPose();
  Eigen::Vector2d current_pos = current_robot_pose.head<2>();
  double current_robot_theta = current_robot_pose.z();
  double current_linear_speed = tank_model_ptr_->getWorldTwist().head<2>().norm();

  // The goal_pose_ is the end of the trajectory, which is static for the path
  goal_pose_ = Eigen::Vector3d(trajectory_.x.back(), trajectory_.y.back(), trajectory_.theta.back());

  // Find closest point in path to the current robot position
  int index = trajectory_.getIndexOfClosestPoint(current_pos);
  double dist_to_end = trajectory_.remaining_path_length[index];

  // Store the projected point of the robot's current position onto the path for visualization
  projected_position_on_path_ = Eigen::Vector2d(trajectory_.x[index], trajectory_.y[index]);

  // Calculate dynamic lookahead distance
  // This will be used to find the carrot point along the path
  double dynamic_lookahead_distance = std::max(min_lookahead_distance_m_, k_lookahead_ * current_linear_speed);

  // Determine the carrot point
  if (dist_to_end <= dynamic_lookahead_distance) {
    // If remaining path is less than lookahead, target the final goal
    carrot_point_ = goal_pose_.head<2>();
  } else {
    // Find the point on the path at the dynamic_lookahead_distance from the projected point
    double carrot_dist_from_end = trajectory_.remaining_path_length[index] - dynamic_lookahead_distance;
    auto carrot_index = std::lower_bound(
      trajectory_.remaining_path_length.begin() + index,
      trajectory_.remaining_path_length.end(),
      carrot_dist_from_end,
      std::greater<double>() // Use std::greater for reverse sorted remaining_path_length
    ) - trajectory_.remaining_path_length.begin();
    carrot_point_ = Eigen::Vector2d(trajectory_.x[carrot_index], trajectory_.y[carrot_index]);
  }

  // Select control strategy based on distance to target
  double dist_err = (goal_pose_.head<2>() - current_pos).norm();
  bool within_xy_exit_threshold = dist_err < xy_exit_threshold_m_;

  Eigen::Vector2d command;

  // Check if we are in the settling phase or should transition to it
  if (within_xy_exit_threshold || settling_) {
    // Use the settling controller
    TankState current_state(dist_err, tank_model_ptr_->getWorldTwist().head<2>().norm(), tank_model_ptr_->getWorldPose().z(), tank_model_ptr_->getWorldTwist().z());
    TankState desired_state(0.0, 0.0, 0.0, 0.0); // Desired state to stop and align
    command = m_settling_controller_ptr->calculateDriveCommand(current_state, desired_state, backwards_);

    // Once we start settling, never exit to avoid instability.
    settling_ = true;
  } else {
    // Pure Pursuit Approach Phase

    // 1. Transform carrot_point_ to robot's local frame using Eigen::Rotation2D
    Eigen::Vector2d carrot_point_world = carrot_point_;
    Eigen::Vector2d robot_position_world = current_robot_pose.head<2>();
    double robot_yaw_world = current_robot_pose.z();

    // Vector from robot to carrot point in world frame
    Eigen::Vector2d vector_to_carrot_world = carrot_point_world - robot_position_world;

    // Create a 2D rotation matrix for rotation by -robot_yaw_world (to transform from world to robot frame)
    Eigen::Rotation2D<double> rotation_to_robot_frame(-robot_yaw_world);

    // Apply the rotation to get the carrot point's coordinates in the robot's local frame
    Eigen::Vector2d carrot_point_robot_frame = rotation_to_robot_frame * vector_to_carrot_world;

    double x_robot_frame = carrot_point_robot_frame.x();
    double y_robot_frame = carrot_point_robot_frame.y();

    // 2. Calculate actual lookahead distance (distance from robot to carrot point)
    double actual_lookahead_distance = (carrot_point_ - current_pos).norm();

    // Avoid division by zero if robot is on top of the carrot point
    if (actual_lookahead_distance < 1e-6) {
        command = Eigen::Vector2d(0.0, 0.0); // Stop if at the carrot point
        return command;
    }

    // 3. Calculate curvature (kappa)
    // kappa = (2 * y_robot_frame) / (Ld^2)
    double curvature = (2.0 * y_robot_frame) / (actual_lookahead_distance * actual_lookahead_distance);

    // 4. Determine desired linear velocity (can be constant or from trajectory speed profile)
    // For now, use the max_speed_linear_percent_ scaled by the tank model's max speed.
    double desired_linear_speed = max_speed_linear_percent_ * tank_model_ptr_->getMaxBaseLinearVelocity();

    // Account for backwards movement in desired linear speed
    if (backwards_) {
        desired_linear_speed *= -1.0;
    }

    // 5. Calculate desired angular velocity
    // omega = v * kappa
    double desired_angular_velocity = desired_linear_speed * curvature;

    // Set the commands directly for Pure Pursuit
    fwd_command_ = desired_linear_speed;
    turn_command_ = desired_angular_velocity;

    command = Eigen::Vector2d(fwd_command_, turn_command_);
  }

  return command;
}

void FollowPathPurePursuit::populateVisualizationMarkers()
{
  auto stamp = node_ptr_->now();

  // Marker for Projected Position On Path (Closest point to robot's current pose)
  visualization_msgs::msg::Marker projected_pos_marker;
  projected_pos_marker.header.frame_id = "map";
  projected_pos_marker.header.stamp = stamp;
  projected_pos_marker.ns = "pure_pursuit_viz";
  projected_pos_marker.id = viz_msg_.markers.size();
  projected_pos_marker.type = visualization_msgs::msg::Marker::SPHERE;
  projected_pos_marker.action = visualization_msgs::msg::Marker::ADD;
  projected_pos_marker.pose.position.x = projected_position_on_path_.x();
  projected_pos_marker.pose.position.y = projected_position_on_path_.y();
  projected_pos_marker.scale.x = 0.1;
  projected_pos_marker.scale.y = 0.1;
  projected_pos_marker.scale.z = 0.1;
  projected_pos_marker.color.a = 1.0;
  projected_pos_marker.color.r = 1.0;
  viz_msg_.markers.push_back(projected_pos_marker);

  // Marker for Lookahead Carrot Point
  visualization_msgs::msg::Marker carrot_point_marker;
  carrot_point_marker.header.frame_id = "map";
  carrot_point_marker.header.stamp = stamp;
  carrot_point_marker.ns = "pure_pursuit_viz";
  carrot_point_marker.id = viz_msg_.markers.size();
  carrot_point_marker.type = visualization_msgs::msg::Marker::SPHERE;
  carrot_point_marker.action = visualization_msgs::msg::Marker::ADD;
  carrot_point_marker.pose.position.x = carrot_point_.x();
  carrot_point_marker.pose.position.y = carrot_point_.y();
  carrot_point_marker.pose.position.z = 0.0;
  carrot_point_marker.scale.x = 0.15;
  carrot_point_marker.scale.y = 0.15;
  carrot_point_marker.scale.z = 0.15;
  carrot_point_marker.color.a = 1.0;
  carrot_point_marker.color.r = 0.0;
  carrot_point_marker.color.g = 1.0;
  carrot_point_marker.color.b = 0.0;
  viz_msg_.markers.push_back(carrot_point_marker);
}


} // namespace ghost_tank
