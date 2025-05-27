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
#include <ghost_tank/visualization/visualization_helpers.hpp>

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
  input_ports.insert(BT::InputPort<double>("min_pursuit_radius_tiles"));
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
  min_pursuit_radius_ = BT_Util::get_input<double>(this, "min_pursuit_radius_tiles") * ghost_util::TILES_TO_METERS;

  // closest_point_index_ is updated in calculateControllerCommand, no need to initialize here.

  return BT::NodeStatus::RUNNING;
}

Eigen::Vector2d FollowPathPurePursuit::calculateCarrotPoint() const
{
  // Default carrot point to the goal in case no valid intersection is found on the path.
  Eigen::Vector2d carrot_point = goal_pose_.head<2>();
  double min_remaining_path_length_for_carrot = std::numeric_limits<double>::max();
  bool carrot_point_found_on_path = false; // Flag to track if a valid carrot point on path was found

  // Iterate through path segments starting from the closest point to find intersections.
  // We need at least two points for a segment, so loop up to size() - 1.
  for (int i = closest_point_index_; i < trajectory_.x.size() - 1; ++i) {
    Eigen::Vector2d p1(trajectory_.x[i], trajectory_.y[i]);
    Eigen::Vector2d p2(trajectory_.x[i + 1], trajectory_.y[i + 1]);

    Eigen::Vector2d intersection1, intersection2;
    // Find intersections of the robot's pursuit circle with the current path segment.
    int num_intersections = geometry::CircleLineIntersection(
      current_position_,          // Circle center (robot's current position)
      dynamic_pursuit_radius_,    // Circle radius (lookahead distance)
      p1, p2,                     // Line segment points
      &intersection1, &intersection2);   // Output intersection points

    for (int j = 0; j < num_intersections; ++j) {
      Eigen::Vector2d current_intersection = (j == 0) ? intersection1 : intersection2;

      // Calculate 't' value for linear interpolation of remaining path length.
      double segment_length = (p2 - p1).norm();
      double t = 0.0;
      if (segment_length > geometry::kEpsilon) { // Avoid division by zero
        t = (current_intersection - p1).norm() / segment_length;
        t = math_util::Clamp(t, 0.0, 1.0); // Ensure t is within [0,1] range due to numerical inaccuracies
      }

      // Linearly interpolate the remaining path length for the intersection point.
      // We're looking for the point that is *furthest along the path*,
      // which corresponds to the *smallest* remaining_path_length value.
      double interpolated_remaining_length =
        trajectory_.remaining_path_length[i] * (1.0 - t) +
        trajectory_.remaining_path_length[i + 1] * t;

      // Transform the intersection point to the robot's local frame to check if it's forward.
      Eigen::Vector2d vector_to_intersection_world = current_intersection - current_position_;
      Eigen::Rotation2D<double> rotation_to_robot_frame(-current_angle_);
      Eigen::Vector2d intersection_point_robot_frame = rotation_to_robot_frame * vector_to_intersection_world;

      // Only consider points that are ahead of the robot (positive x in robot frame).
      // If a valid intersection is found that is further along the path than the current best, update.
      if (intersection_point_robot_frame.x() >= 0.0) {
        if (interpolated_remaining_length < min_remaining_path_length_for_carrot) {
          min_remaining_path_length_for_carrot = interpolated_remaining_length;
          carrot_point = current_intersection;
          carrot_point_found_on_path = true;
        }
      }
    }
  }

  // If no valid intersection point was found on the path segments within the lookahead distance
  // (e.g., all segments are too far or behind the robot),
  // then the carrot point defaults to the closest point on the path.
  if (!carrot_point_found_on_path) {
    carrot_point = projected_position_on_path_;
  }

  return carrot_point;
}

Eigen::Vector2d FollowPathPurePursuit::calculateKinematicallyFeasibleVelocities(
  double des_lin_vel_unconstrained,
  double curvature,
  double max_wheel_lin_vel,
  double wheel_dist_m) const
{
  // Calculate wheel velocities based on unconstrained linear velocity
  double left_vel_unconstrained = des_lin_vel_unconstrained * (1 - curvature * wheel_dist_m);
  double right_vel_unconstrained = des_lin_vel_unconstrained * (1 + curvature * wheel_dist_m);
  double max_wheel_speed_unconstrained = std::max(std::fabs(left_vel_unconstrained), std::fabs(right_vel_unconstrained));

  // Exit early if limits are satisfied
  if (max_wheel_speed_unconstrained <= max_wheel_lin_vel) {
    return Eigen::Vector2d(des_lin_vel_unconstrained, des_lin_vel_unconstrained * curvature);
  }

  double scaling_factor = max_wheel_lin_vel / max_wheel_speed_unconstrained;

  // Calculate the maximum linear velocity kinematically possible for this curvature.
  double constrained_linear_vel = des_lin_vel_unconstrained * scaling_factor;

  return Eigen::Vector2d(constrained_linear_vel, constrained_linear_vel * curvature);
}


Eigen::Vector2d FollowPathPurePursuit::calculatePurePursuitDriveCommand(bool use_settling_controller)
{
  // Transform carrot_point_ to robot's local frame
  Eigen::Vector2d vector_to_carrot_world = carrot_point_ - current_position_;
  Eigen::Vector2d carrot_point_robot_frame = Eigen::Rotation2D<double>(-current_angle_) * vector_to_carrot_world;

  // Calculate actual lookahead distance (distance from robot to carrot point)
  double dist_to_carrot = vector_to_carrot_world.norm();

  // Avoid division by zero if robot is on top of the carrot point
  if (dist_to_carrot < 1.0e-6) {
    curvature_ = 0.0;
    return Eigen::Vector2d(0.0, 0.0);
  }

  // Calculate curvature
  curvature_ = (2.0 * carrot_point_robot_frame.y()) / (dist_to_carrot * dist_to_carrot);

  // Calculate desired linear velocity (unconstrained by kinematic limits initially)
  double des_lin_vel_unconstrained = max_speed_linear_percent_ * tank_model_ptr_->getMaxBaseLinearVelocity();

  // Get kinematically feasible base velocities (this method handles the conditional limiting)
  Eigen::Vector2d vel_cmd = calculateKinematicallyFeasibleVelocities(
    des_lin_vel_unconstrained,
    curvature_,
    tank_model_ptr_->getMaxBaseLinearVelocity(),
    tank_model_ptr_->getWheelDistMeters()
  );

  // Select Controller
  ghost_control::PIDController * distance_controller_ptr;
  ghost_control::PIDController * steering_controller_ptr;

  if (use_settling_controller) {
    distance_controller_ptr = m_distance_settling_controller_ptr.get();
    steering_controller_ptr = m_steering_settling_controller_ptr.get();
  } else {
    distance_controller_ptr = m_distance_approach_controller_ptr.get();
    steering_controller_ptr = m_steering_approach_controller_ptr.get();
  }

  // Calculate Commands
  double fwd_cmd = distance_controller_ptr->calculateCommand(dist_to_goal_, -tank_model_ptr_->getWorldTwist().head<2>().norm());

  double angle_error = ghost_util::SmallestAngleDistRad(trajectory_.theta[closest_point_index_], current_angle_);
  double ang_vel_error = vel_cmd.y() - tank_model_ptr_->getWorldTwist().z();
  double ang_cmd = steering_controller_ptr->calculateCommand(angle_error, ang_vel_error);

  if (backwards_) {
    fwd_cmd *= -1.0;
  }

  return Eigen::Vector2d(fwd_cmd, ang_cmd);
}


Eigen::Vector2d FollowPathPurePursuit::calculateControllerCommand()
{
  // Update pure pursuit specific data
  dynamic_pursuit_radius_ = std::max(min_pursuit_radius_, k_lookahead_ * tank_model_ptr_->getWorldTwist().head<2>().norm());
  closest_point_index_ = trajectory_.getIndexOfClosestPoint(current_position_);
  projected_position_on_path_ = Eigen::Vector2d(trajectory_.x[closest_point_index_], trajectory_.y[closest_point_index_]);

  dist_to_goal_ = (goal_pose_.head<2>() - current_position_).norm();

  bool within_pursuit_radius = dist_to_goal_ <= dynamic_pursuit_radius_;

  Eigen::Vector2d robot_to_goal_vector = goal_pose_.head<2>() - current_position_;
  Eigen::Vector2d goal_in_robot_frame = Eigen::Rotation2D<double>(-current_angle_) * robot_to_goal_vector;
  bool goal_is_behind_robot = goal_in_robot_frame.x() < 0.0; // Goal has negative x in robot frame

  if (within_pursuit_radius) {
    carrot_point_ = goal_pose_.head<2>();
  } else {
    carrot_point_ = calculateCarrotPoint();
  }

  Eigen::Vector2d command;
  if (dist_to_goal_ < xy_exit_threshold_m_ || settling_ || (within_pursuit_radius && goal_is_behind_robot)) {
    // Use the settling controller
    double alignment_angle = ghost_util::SmallestAngleDistRad(atan2(robot_to_goal_vector.y(), robot_to_goal_vector.x()), current_angle_);
    settling_alignment_error_ = dist_to_goal_ * cos(alignment_angle);
    command.x() = m_distance_settling_controller_ptr->calculateCommand(settling_alignment_error_, -tank_model_ptr_->getWorldTwist().head<2>().norm());

    double angle_error = ghost_util::SmallestAngleDistRad(goal_pose_.z(), current_angle_);
    command.y() = m_steering_settling_controller_ptr->calculateCommand(angle_error, -tank_model_ptr_->getWorldTwist().z());

    // Once we start settling, never exit to avoid instability.
    settling_ = true;
    curvature_ = 0.0;
  } else {
    // Use approach controller with Pure Pursuit
    command = calculatePurePursuitDriveCommand(dist_to_goal_ < xy_settling_radius_m_);
  }

  return command;
}

void FollowPathPurePursuit::visualizeSettlingError()
{
  Eigen::Vector2d alignment_point_world = current_position_ + settling_alignment_error_ * Eigen::Vector2d(std::cos(current_angle_), std::sin(current_angle_));

  // Draw the red line (longitudinal alignment distance)
  visualization::getLineMarker(
    viz_msg_,
    current_position_,
    alignment_point_world,
    0.25 * visualization::MARKER_Z_OFFSET,
    visualization::getColorRGBA(1.0, 0.0, 0.0, 1.0)
  );

  // Draw the blue line (lateral ignored distance)
  visualization::getLineMarker(
    viz_msg_,
    alignment_point_world,
    goal_pose_.head<2>(),
    0.25 * visualization::MARKER_Z_OFFSET,
    visualization::getColorRGBA(0.0, 0.0, 1.0, 1.0)
  );
}

void FollowPathPurePursuit::populateVisualizationMarkers()
{
  FollowPath::populateVisualizationMarkers();
  visualization::getPointMarker(viz_msg_, projected_position_on_path_, visualization::getColorRGBA(1.0, 1.0, 1.0, 1.0), 0.5 * visualization::MARKER_Z_OFFSET);
  visualization::getPointMarker(viz_msg_, carrot_point_, visualization::getColorRGBA(1.0, 0.5, 0.0, 1.0), 0.5 * visualization::MARKER_Z_OFFSET);
  visualization::getCircleMarker(viz_msg_, current_position_, dynamic_pursuit_radius_, visualization::getColorRGBA(0.0, 0.0, 1.0, 0.3), 0.0);
  if (!settling_) {
    ghost_tank::visualization::getArcOrLineMarker(viz_msg_, current_position_, current_angle_, carrot_point_, curvature_, 0.5 * visualization::MARKER_Z_OFFSET);
  } else {
    visualizeSettlingError();
  }
}

} // namespace ghost_tank
