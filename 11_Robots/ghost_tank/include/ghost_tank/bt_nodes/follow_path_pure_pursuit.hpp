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
#pragma once

#include <ghost_tank/bt_nodes/follow_path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Geometry>

#include <math/line2d.h>
#include <math/geometry.h>
#include <math/math_util.h>
namespace ghost_tank
{

class FollowPathPurePursuit : public FollowPath
{
public:
  static constexpr double CARROT_POINT_MARKER_DIAM = 0.025;
  static constexpr double PROJ_POINT_MARKER_DIAM = 0.025;
  static constexpr double MARKER_Z_OFFSET = 0.05;

  FollowPathPurePursuit(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;

  Eigen::Vector2d calculateControllerCommand() override;

  void populateVisualizationMarkers() override;

protected:
  Eigen::Vector2d calculateCarrotPoint() const;
  Eigen::Vector2d calculatePurePursuitDriveCommand(bool use_settling_controller);
  void visualizeSettlingError();
  void visualizeDeccelerationZone();

private:
  /**
   * @brief Calculates the maximum kinematically feasible linear and angular velocities
   * for a given curvature, ensuring no wheel exceeds its maximum speed.
   * This method assumes the relationship omega = V * curvature.
   *
   * @param desired_linear_vel_unconstrained The ideal linear velocity before kinematic limiting.
   * @param curvature The path curvature (1/radius).
   * @param max_single_wheel_linear_vel The maximum linear velocity of a single wheel.
   * @param half_track_width_meters The distance from the robot's center to a wheel's center.
   * @return Eigen::Vector2d containing (constrained_linear_vel, constrained_angular_vel).
   */
  Eigen::Vector2d calculateKinematicallyFeasibleVelocities(
    double des_lin_vel_unconstrained,
    double curvature,
    double max_wheel_lin_vel,
    double wheel_dist_m) const;

  double k_lookahead_{0.0};
  double min_pursuit_radius_{0.0};
  double dynamic_pursuit_radius_{0.0};

  int closest_point_index_{0};

  Eigen::Vector2d projected_position_on_path_;
  Eigen::Vector2d carrot_point_;
  double curvature_{0.0};

  double settling_alignment_error_{0.0};
  double dist_to_goal_{0.0};

  double deceleration_start_distance_m_;
  double deceleration_exponent_;
  double min_approach_velocity_mps_;
};

} // namespace ghost_tank
