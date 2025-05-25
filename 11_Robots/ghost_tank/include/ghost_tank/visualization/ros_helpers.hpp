/*
 * Copyright (c) 2024 Maxx Wilson
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
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>

#include <ghost_util/angle_util.hpp>
#include <ghost_tank/control/trajectory.hpp>

namespace ghost_tank
{

namespace visualization
{

constexpr double TRAJECTORY_SPHERE_DIAMETER = 0.02;
constexpr double TRAJECTORY_ARROW_LENGTH = 0.075;
constexpr double TRAJECTORY_ARROW_SHAFT_DIAMETER = 0.015;
constexpr double TRAJECTORY_ARROW_HEAD_DIAMETER = 0.025;
constexpr float TRAJECTORY_SPHERE_ALPHA = 0.5f;
constexpr float TRAJECTORY_ARROW_ALPHA = 1.0f;
constexpr double TRAJECTORY_ARROW_Z_OFFSET = 0.1;

/**
 * @brief Populates a visualization_msgs::msg::MarkerArray with trajectory points as spheres and arrows.
 *
 * This function converts a Trajectory object into a ROS MarkerArray message suitable
 * for visualization in rviz. It generates a list of spheres representing each
 * (x,y) point in the trajectory, colored based on remaining path length (blue to red).
 * If remaining_path_length is not calculated, all spheres will be blue.
 * It also generates a specified number of arrows indicating the orientation (theta),
 * evenly spaced along the trajectory, starting from the end.
 *
 * NOTE: This function *appends* markers to the provided viz_msg.
 * The caller is responsible for clearing viz_msg.markers if a fresh set of markers is desired.
 * Marker IDs are generated based on the current size of viz_msg.markers to ensure uniqueness.
 *
 * @param path The input Trajectory object containing the x, y, and theta data,
 * and optionally, the pre-calculated remaining_path_length.
 * @param viz_msg The output MarkerArray message that will be populated.
 * @param num_arrows The desired number of arrows to place along the trajectory.
 * The arrows will be evenly spaced, with the last arrow always at the end of the trajectory.
 * Defaults to 5 if not specified.
 */
void getTrajectoryMsg(
  const motion_planning::Trajectory & path,
  visualization_msgs::msg::MarkerArray & viz_msg,
  int num_arrows = 5)
{
  // Get current ROS time once for all markers in this array
  const auto current_ros_time = rclcpp::Clock().now();

  // Ensure there are points in the trajectory to visualize at all
  if (path.size() == 0) {
    RCLCPP_WARN(rclcpp::get_logger("getTrajectoryMsg"), "Trajectory is empty. Skipping visualization.");
    return;
  }

  // Determine if remaining_path_length is available for coloring
  bool use_gradient_coloring = !path.remaining_path_length.empty() && path.remaining_path_length[0] != 0.0;
  if (!use_gradient_coloring) {
    RCLCPP_WARN(rclcpp::get_logger("getTrajectoryMsg"), "remaining_path_length is not calculated. Defaulting to all blue spheres.");
  }

  // Marker for all trajectory points as spheres
  visualization_msgs::msg::Marker sphere_list_marker;
  sphere_list_marker.header.frame_id = "map";
  sphere_list_marker.header.stamp = current_ros_time;
  sphere_list_marker.ns = "trajectory_points";
  sphere_list_marker.id = viz_msg.markers.size();
  sphere_list_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  sphere_list_marker.action = visualization_msgs::msg::Marker::ADD;

  // Set scale for individual spheres (x, y, z represent diameter)
  sphere_list_marker.scale.x = TRAJECTORY_SPHERE_DIAMETER;
  sphere_list_marker.scale.y = TRAJECTORY_SPHERE_DIAMETER;
  sphere_list_marker.scale.z = TRAJECTORY_SPHERE_DIAMETER;

  // Get the total path length for gradient calculation (if applicable)
  double total_path_length = 0.0;
  if (use_gradient_coloring) {
    total_path_length = path.remaining_path_length[0];
  }

  // Populate the points and colors for the sphere list
  for (int i = 0; i < path.size(); ++i) {
    geometry_msgs::msg::Point p;
    p.x = path.x[i];
    p.y = path.y[i];
    p.z = 0.0; // Assuming 2D trajectory on the ground plane
    sphere_list_marker.points.push_back(p);

    std_msgs::msg::ColorRGBA point_color;
    if (use_gradient_coloring && total_path_length > 0.0) {
      // Calculate normalized distance for color interpolation
      // normalized_dist = 0 at start (full length), normalized_dist = 1 at end (zero length)
      double normalized_dist = 1.0 - (path.remaining_path_length[i] / total_path_length);

      // Interpolate color from blue (0,0,1) to red (1,0,0)
      point_color.r = static_cast<float>(normalized_dist);      // R: 0 (blue) -> 1 (red)
      point_color.g = 0.0f;                                     // G: always 0
      point_color.b = static_cast<float>(1.0 - normalized_dist); // B: 1 (blue) -> 0 (red)
    } else {
      // Default to blue if gradient coloring is not used or total_path_length is zero
      point_color.r = 0.0f;
      point_color.g = 0.0f;
      point_color.b = 1.0f;
    }
    point_color.a = TRAJECTORY_SPHERE_ALPHA;
    sphere_list_marker.colors.push_back(point_color);
  }

  viz_msg.markers.push_back(sphere_list_marker);


  // Markers for arrows at specific intervals
  // Start arrow IDs from the current size of the marker array to ensure uniqueness
  int arrow_id_counter = viz_msg.markers.size();

  // Only generate arrows if num_arrows is greater than 0 and there are points in the path
  if (num_arrows > 0 && path.size() > 0) {
    // Ensure we don't try to place more arrows than there are points
    int actual_num_arrows = std::min(num_arrows, path.size());

    // Calculate the step interval for even distribution
    // If only one arrow, it's at the end. Otherwise, distribute over (actual_num_arrows - 1) intervals.

    double step_interval = 0.0;
    if (actual_num_arrows > 1) {
      step_interval = static_cast<double>(path.size() - 1) / static_cast<double>(actual_num_arrows - 1);
    }

    for (int k = 0; k < actual_num_arrows; ++k) {
      // Calculate the index for the current arrow, starting from the end (k=0) and moving towards the beginning.
      int i = static_cast<int>(path.size() - 1 - k * step_interval);

      // Ensure the index is valid (should be within bounds due to calculation, but for safety)
      if (i < 0) {
        i = 0;
      }
      if (i >= path.size()) { // Should not happen with current logic, but good for robustness
        i = path.size() - 1;
      }

      visualization_msgs::msg::Marker arrow_marker;
      arrow_marker.header.frame_id = "map"; // Same frame as spheres
      arrow_marker.header.stamp = current_ros_time; // Use the single acquired ROS time
      arrow_marker.ns = "trajectory_arrows"; // Namespace for arrow markers
      arrow_marker.id = arrow_id_counter++; // Unique ID for each arrow within this segment
      arrow_marker.type = visualization_msgs::msg::Marker::ARROW; // Type is an arrow
      arrow_marker.action = visualization_msgs::msg::Marker::ADD; // Add the marker

      // Set position of the arrow, elevated slightly in Z
      arrow_marker.pose.position.x = path.x[i];
      arrow_marker.pose.position.y = path.y[i];
      arrow_marker.pose.position.z = TRAJECTORY_ARROW_Z_OFFSET; // Elevated Z for visibility

      // Set orientation of the arrow based on theta using custom utility
      ghost_util::yawToQuaternionRad(
        path.theta[i],
        arrow_marker.pose.orientation.w,
        arrow_marker.pose.orientation.x,
        arrow_marker.pose.orientation.y,
        arrow_marker.pose.orientation.z);

      // Set scale for the arrow (x: total length, y: shaft diameter, z: head diameter)
      arrow_marker.scale.x = TRAJECTORY_ARROW_LENGTH;
      arrow_marker.scale.y = TRAJECTORY_ARROW_SHAFT_DIAMETER;
      arrow_marker.scale.z = TRAJECTORY_ARROW_HEAD_DIAMETER;

      // Set color for the arrows (RGBA) - semi-transparent green
      arrow_marker.color.r = 0.0f; // No Red
      arrow_marker.color.g = 1.0f; // Full Green
      arrow_marker.color.b = 0.0f; // No Blue
      arrow_marker.color.a = TRAJECTORY_ARROW_ALPHA; // Alpha (transparency)

      viz_msg.markers.push_back(arrow_marker);
    }
  }
}

} //namespace visualization
} //namespace ghost_tank
