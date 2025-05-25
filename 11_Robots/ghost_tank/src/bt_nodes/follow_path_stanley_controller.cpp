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


#include "ghost_tank/bt_nodes/follow_path_stanley_controller.hpp"
#include <cmath>
#include <algorithm>

namespace ghost_tank
{

// Add a constant for the Stanley gain
// This can be tuned via a blackboard input port if desired for more flexibility
constexpr double K_STANLEY_GAIN = 1.0; // A common starting value, will likely need tuning

FollowPathStanleyController::FollowPathStanleyController(const std::string & name, const BT::NodeConfig & config)
: FollowPath(name, config)
{
}

BT::PortsList FollowPathStanleyController::providedPorts()
{
  auto input_ports = getBaseInputPorts();
  // We can add specific Stanley controller parameters here if needed, e.g., K_STANLEY_GAIN
  return input_ports;
}

Eigen::Vector2d FollowPathStanleyController::calculateControllerCommand()
{
  // Get current vehicle pose and twist from the TankModel
  Eigen::Vector3d current_pose_world = tank_model_ptr_->getWorldPose();
  Eigen::Vector3d current_twist_world = tank_model_ptr_->getWorldTwist(); // x, y, theta_dot

  // Vehicle's current position (x, y) and heading (theta)
  double current_x = current_pose_world.x();
  double current_y = current_pose_world.y();
  double current_theta = current_pose_world.z(); // Radians

  // Vehicle's current forward speed (longitudinal velocity)
  // Using Eigen's norm() for the 2D velocity vector
  double current_linear_speed = current_twist_world.head<2>().norm();

  // Handle the case where speed is very low to avoid division by zero or large errors
  if (current_linear_speed < 0.1) { // Threshold, e.g., 0.1 m/s
    current_linear_speed = 0.1;   // Set a minimum speed to prevent division by zero
    // If the robot is stopped, we might want to just rotate towards the path
    // or stop moving if near the goal. This simple fix avoids NaN.
  }

  // --- 1. Find the closest point on the trajectory to the vehicle's reference point ---
  // Using the center of the robot's position as the reference point for simplicity,
  // since getWheelBase() is not available.
  Eigen::Vector2d robot_center_pos(current_x, current_y);

  // Find the index of the closest point on the trajectory to the robot's center
  // For simplicity, let's iterate through the trajectory.
  // A more efficient way would be to use a k-d tree or specialized path searching algorithm.

  double min_dist = std::numeric_limits<double>::max();
  int closest_idx = 0;
  for (int i = 0; i < trajectory_.size(); ++i) {
    // Create an Eigen::Vector2d for the trajectory point
    Eigen::Vector2d trajectory_point(trajectory_.x[i], trajectory_.y[i]);

    // Calculate the squared distance using Eigen's norm_sq()
    double dist = (trajectory_point - robot_center_pos).norm();

    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  // Get the closest point on the path and its tangent angle
  double path_x_closest = trajectory_.x[closest_idx];
  double path_y_closest = trajectory_.y[closest_idx];
  double path_theta_closest = trajectory_.theta[closest_idx]; // Tangent angle of the path at closest point

  // --- 2. Calculate Heading Error (psi) ---
  // The difference between vehicle's current heading and the path's heading at the closest point
  double psi_error = ghost_util::SmallestAngleDistRad(path_theta_closest, current_theta);

  // --- 3. Calculate Cross-Track Error (e) ---
  // The signed distance from the robot's center to the path.
  // Positive 'e' means the robot's center is to the left of the path (assuming path_theta_closest is forward).
  // This is calculated as the projection of the vector from path_closest_point to robot_center_pos
  // onto the normal to the path at path_closest_point.

  // Vector from closest point on path to robot center
  double vec_x = robot_center_pos.x() - path_x_closest;
  double vec_y = robot_center_pos.y() - path_y_closest;

  // Normal vector to the path (path_theta_closest + 90 degrees for left-side normal)
  double path_normal_x = -std::sin(path_theta_closest);
  double path_normal_y = std::cos(path_theta_closest);

  // Cross-track error is the dot product of the vector from path_closest_point to robot_center_pos
  // with the normal vector to the path at path_closest_point.
  double cross_track_error = (vec_x * path_normal_x) + (vec_y * path_normal_y);

  // --- 4. Apply Stanley Formula for Steering Command ---
  // The Stanley controller's steering output is typically a desired heading rate or steering angle.
  // For a differential drive robot (like a tank), this directly translates to the turn command.
  double desired_steering_angle = psi_error + std::atan2(K_STANLEY_GAIN * cross_track_error, current_linear_speed);

  // The 'fwd_command_' is the desired linear velocity.
  // We can set this to a constant or a percentage of max speed.
  double desired_fwd_command = max_speed_linear_percent_; // Use the configured max linear speed percentage

  // Package as Eigen::Vector2d (x = forward, y = turn)
  return {desired_fwd_command, desired_steering_angle};
}

} // namespace ghost_tank
