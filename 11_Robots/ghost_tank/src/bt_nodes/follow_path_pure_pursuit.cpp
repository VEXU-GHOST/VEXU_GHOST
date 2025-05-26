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


#include <Eigen/Geometry>
#include <cmath>
#include <limits>

#include "ghost_tank/bt_nodes/follow_path_pure_pursuit.hpp"
#include <ghost_tank/visualization/ros_helpers.hpp>

namespace ghost_tank
{

FollowPathPurePursuit::FollowPathPurePursuit(const std::string & name, const BT::NodeConfig & config)
: FollowPath(name, config)
{
}

BT::PortsList FollowPathPurePursuit::providedPorts()
{
  auto input_ports = FollowPath::getBaseInputPorts();
  input_ports.insert(BT::InputPort<double>("k_lookahead"));
  input_ports.insert(BT::InputPort<double>("min_lookahead_distance_tiles"));
  return input_ports;
}

BT::NodeStatus FollowPathPurePursuit::onStart()
{
  // Call base class onStart to initialize common parameters
  BT::NodeStatus status = FollowPath::onStart();
  if (status != BT::NodeStatus::RUNNING) {
    return status; // Return if base initialization failed or is not running
  }

  // Get Pure Pursuit specific parameters
  k_lookahead_ = BT_Util::get_input<double>(this, "k_lookahead");
  min_lookahead_distance_m_ = BT_Util::get_input<double>(this, "min_lookahead_distance_tiles") * ghost_util::TILES_TO_METERS;

  return BT::NodeStatus::RUNNING;
}

Eigen::Vector2d FollowPathPurePursuit::calculateControllerCommand()
{
  // Get current robot pose and linear speed
  current_position_ = tank_model_ptr_->getWorldPose().head<2>();
  current_robot_theta_ = tank_model_ptr_->getWorldPose().z();
  double current_linear_speed = tank_model_ptr_->getWorldTwist().head<2>().norm();

  // The goal_pose_ is the end of the trajectory, which is static for the path
  goal_pose_ = Eigen::Vector3d(trajectory_.x.back(), trajectory_.y.back(), trajectory_.theta.back());

  // Find closest point in path to the current robot position
  int closest_point_index = trajectory_.getIndexOfClosestPoint(current_position_);

  // Store the projected point of the robot's current position onto the path for visualization
  projected_position_on_path_ = Eigen::Vector2d(trajectory_.x[closest_point_index], trajectory_.y[closest_point_index]);

  // Calculate dynamic lookahead distance
  dynamic_lookahead_distance_ = std::max(min_lookahead_distance_m_, k_lookahead_ * current_linear_speed);

  // --- Determine the carrot point using the pursuit radius method ---
  carrot_point_ = goal_pose_.head<2>(); // Default to goal in case no valid point is found
  double min_remaining_path_length_for_carrot = std::numeric_limits<double>::max();

  // Iterate through path segments starting from the closest point
  // We need at least two points for a segment, so loop up to size() - 1
  for (int i = closest_point_index; i < trajectory_.x.size() - 1; ++i) {
    Eigen::Vector2d p1(trajectory_.x[i], trajectory_.y[i]);
    Eigen::Vector2d p2(trajectory_.x[i+1], trajectory_.y[i+1]);

    // Vector representing the line segment
    Eigen::Vector2d segment_vec = p2 - p1;

    // Vector from robot to p1 (relative to robot's current position)
    Eigen::Vector2d robot_to_p1 = p1 - current_position_;

    // Coefficients for the quadratic equation At^2 + Bt + C = 0
    // This finds intersections of a circle (robot center, lookahead_distance radius) with a line segment
    double A = segment_vec.dot(segment_vec);
    double B = 2 * robot_to_p1.dot(segment_vec);
    double C = robot_to_p1.dot(robot_to_p1) - dynamic_lookahead_distance_ * dynamic_lookahead_distance_;

    double discriminant = B * B - 4 * A * C;

    if (discriminant < 0) {
      // No real intersection points for this segment (circle does not intersect line)
      continue;
    }

    double sqrt_discriminant = std::sqrt(discriminant);
    double t_values[] = {
      (-B + sqrt_discriminant) / (2 * A),
      (-B - sqrt_discriminant) / (2 * A)
    };

    // Check both potential intersection points
    for (double t : t_values) {
      // Ensure the intersection point lies within the segment [0, 1]
      if (t >= 0.0 && t <= 1.0) {
        Eigen::Vector2d intersection_point = p1 + t * segment_vec;

        // Linearly interpolate the remaining path length for the intersection point.
        // We're looking for the point that is *furthest along the path*,
        // which corresponds to the *smallest* remaining_path_length value.
        double interpolated_remaining_length =
            trajectory_.remaining_path_length[i] * (1.0 - t) +
            trajectory_.remaining_path_length[i+1] * t;

        // If this intersection point is further along the path than the current best candidate,
        // update the carrot point.
        if (interpolated_remaining_length < min_remaining_path_length_for_carrot) {
          min_remaining_path_length_for_carrot = interpolated_remaining_length;
          carrot_point_ = intersection_point;
        }
      }
    }
  }

  // --- End carrot point determination ---

  // Select control strategy based on distance to target
  double dist_err = (goal_pose_.head<2>() - current_position_).norm();
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
    curvature_ = 0.0;
  } else {
    // Pure Pursuit Approach Phase

    // 1. Transform carrot_point_ to robot's local frame using Eigen::Rotation2D
    Eigen::Vector2d carrot_point_world = carrot_point_;
    Eigen::Vector2d robot_position_world = current_position_;
    double robot_yaw_world = current_robot_theta_;

    // Vector from robot to carrot point in world frame
    Eigen::Vector2d vector_to_carrot_world = carrot_point_world - robot_position_world;

    // Create a 2D rotation matrix for rotation by -robot_yaw_world (to transform from world to robot frame)
    Eigen::Rotation2D<double> rotation_to_robot_frame(-robot_yaw_world);

    // Apply the rotation to get the carrot point's coordinates in the robot's local frame
    Eigen::Vector2d carrot_point_robot_frame = rotation_to_robot_frame * vector_to_carrot_world;

    double x_robot_frame = carrot_point_robot_frame.x();
    double y_robot_frame = carrot_point_robot_frame.y();

    // 2. Calculate actual lookahead distance (distance from robot to carrot point)
    double actual_lookahead_distance = (carrot_point_ - current_position_).norm();

    // Avoid division by zero if robot is on top of the carrot point
    if (actual_lookahead_distance < 1e-6) {
      command = Eigen::Vector2d(0.0, 0.0);   // Stop if at the carrot point
      curvature_ = 0.0;
      // Since we want zero output for testing, ensure this path returns zeros
      return Eigen::Vector2d(0.0, 0.0);
    }

    // 3. Calculate curvature (kappa)
    // kappa = (2 * y_robot_frame) / (Ld^2)
    curvature_ = (2.0 * y_robot_frame) / (actual_lookahead_distance * actual_lookahead_distance);

    // 4. Determine desired linear velocity (can be constant or from trajectory speed profile)
    // For now, use the max_speed_linear_percent_ scaled by the tank model's max speed.
    double desired_linear_speed = max_speed_linear_percent_ * tank_model_ptr_->getMaxBaseLinearVelocity();

    // Account for backwards movement in desired linear speed
    if (backwards_) {
      desired_linear_speed *= -1.0;
    }

    // 5. Calculate desired angular velocity
    // omega = v * kappa
    double desired_angular_velocity = desired_linear_speed * curvature_;

    // Set the commands directly for Pure Pursuit
    fwd_command_ = desired_linear_speed;
    turn_command_ = desired_angular_velocity;

    // Assign calculated command, but then override to zero for safe testing
    command = Eigen::Vector2d(fwd_command_, turn_command_);
  }

  // Always return zero command for safe testing, as requested.
  return Eigen::Vector2d(0.0, 0.0);
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
  projected_pos_marker.pose.position.z = MARKER_Z_OFFSET;
  projected_pos_marker.scale.x = PROJ_POINT_MARKER_DIAM;
  projected_pos_marker.scale.y = PROJ_POINT_MARKER_DIAM;
  projected_pos_marker.scale.z = PROJ_POINT_MARKER_DIAM;
  projected_pos_marker.color.a = 1.0;
  projected_pos_marker.color.r = 1.0;
  projected_pos_marker.color.g = 1.0;
  projected_pos_marker.color.b = 1.0;
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
  carrot_point_marker.pose.position.z = MARKER_Z_OFFSET;
  carrot_point_marker.scale.x = CARROT_POINT_MARKER_DIAM;
  carrot_point_marker.scale.y = CARROT_POINT_MARKER_DIAM;
  carrot_point_marker.scale.z = CARROT_POINT_MARKER_DIAM;
  carrot_point_marker.color.a = 1.0;
  carrot_point_marker.color.r = 1.0;
  carrot_point_marker.color.g = 0.5;
  carrot_point_marker.color.b = 0.0;
  viz_msg_.markers.push_back(carrot_point_marker);

  // --- New: Marker for Pursuit Radius Circle ---
  visualization_msgs::msg::Marker pursuit_radius_circle_marker;
  pursuit_radius_circle_marker.header.frame_id = "map";
  pursuit_radius_circle_marker.header.stamp = stamp;
  pursuit_radius_circle_marker.ns = "pure_pursuit_viz";
  pursuit_radius_circle_marker.id = viz_msg_.markers.size();
  pursuit_radius_circle_marker.type = visualization_msgs::msg::Marker::SPHERE; // SPHERE type can render as a circle if Z scale is small
  pursuit_radius_circle_marker.action = visualization_msgs::msg::Marker::ADD;

  // Center the circle at the robot's current position
  pursuit_radius_circle_marker.pose.position.x = current_position_.x();
  pursuit_radius_circle_marker.pose.position.y = current_position_.y();
  pursuit_radius_circle_marker.pose.position.z = MARKER_Z_OFFSET; // Keep it above the map for visibility

  // Scale the sphere to represent the circle's diameter
  // Scale.x and scale.y control the diameter in XY plane
  pursuit_radius_circle_marker.scale.x = dynamic_lookahead_distance_ * 2.0;
  pursuit_radius_circle_marker.scale.y = dynamic_lookahead_distance_ * 2.0;
  pursuit_radius_circle_marker.scale.z = 0.01; // Make Z very small to appear as a flat circle

  pursuit_radius_circle_marker.color.a = 0.3; // Semi-transparent
  pursuit_radius_circle_marker.color.r = 0.0;
  pursuit_radius_circle_marker.color.g = 0.0;
  pursuit_radius_circle_marker.color.b = 1.0; // Blue color
  viz_msg_.markers.push_back(pursuit_radius_circle_marker);
  // --- End new marker ---

  ghost_tank::visualization::getArcOrLineMarker(viz_msg_, current_position_, current_robot_theta_, carrot_point_, curvature_);
}


} // namespace ghost_tank