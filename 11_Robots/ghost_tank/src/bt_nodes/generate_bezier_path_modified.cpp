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


#include "ghost_tank/bt_nodes/generate_bezier_path_modified.hpp"
#include <ghost_tank/visualization/visualization_helpers.hpp>

namespace ghost_tank
{

using motion_planning::TRAJECTORY_STRING_ENUM_MAP;

GenerateBezierPath_modified::GenerateBezierPath_modified(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "tank_trajectory_ptr", tank_trajectory_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);

  if (node_ptr_) {
    trajectory_viz_pub_ptr_ = node_ptr_->create_publisher<visualization_msgs::msg::MarkerArray>("/autonomy/current_tank_trajectory", 10);
  }
}

BT::PortsList GenerateBezierPath_modified::providedPorts()
{
  return {
   // BT::InputPort<double>("cv_x"),
   // BT::InputPort<double>("cv_y"),
    BT::InputPort<double>("end_theta_deg"),
    BT::InputPort<double>("lead_tiles"),
    BT::InputPort<bool>("backwards", false, ""),
    BT::InputPort<std::string>("type", "CUBIC_BEZIER", ""),
    BT::InputPort<int>("num_points", 250, "")
  };
}

BT::NodeStatus GenerateBezierPath_modified::onStart()
{
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GenerateBezierPath_modified::onRunning()
{
  double num_points = BT_Util::get_input<int>(this, "num_points");
  motion_planning::Trajectory traj(num_points);
  
  // Determine trajectory type
  std::string type = BT_Util::get_input<std::string>(this, "type");
  if (TRAJECTORY_STRING_ENUM_MAP.count(type) == 0) {
    std::cout << "[GenerateBezierPath] WARNING: trajectory type \"" << type << "\" not recognized! Defaulting to CUBIC_BEZIER..." << std::endl;
    type = "CUBIC_BEZIER";
  }
  auto traj_type = TRAJECTORY_STRING_ENUM_MAP.at(type);
  
  // 1. Get Current World Context
  Eigen::Vector2d start_pos = tank_model_ptr_->getWorldPose().head<2>();
  double start_angle = tank_model_ptr_->getWorldPose()[2];
  
  // 2. Pull CV relative meters from blackboard
  double cv_fwd = blackboard_->get<double>("cv_y");  // Forward in robot frame
  double cv_lat = blackboard_->get<double>("cv_x");  // Lateral in robot frame
  
  // 3. Transform Relative CV Target to World Coordinates
  Eigen::Rotation2D<double> R(start_angle);
  Eigen::Vector2d cv_rel(cv_fwd, cv_lat);
  Eigen::Vector2d target_world = R * cv_rel + start_pos;
  
  // 4. Calculate Final Heading (pointing at target)
  double end_theta_rad = atan2(target_world.y() - start_pos.y(), 
                               target_world.x() - start_pos.x());
  
  // 5. Get Lead Distance
  double lead_m = BT_Util::get_input<double>(this, "lead_tiles") * ghost_util::TILES_TO_METERS;
  
  // 6. Handle Backwards Driving (flip angles BEFORE mirroring)
  bool backwards = BT_Util::get_input<bool>(this, "backwards");
  if (backwards) {
    start_angle = ghost_util::FlipAnglePI(start_angle);
    end_theta_rad = ghost_util::FlipAnglePI(end_theta_rad);
  }
  
  // 7. Mirror path about center line of VEX field (for red/blue alliance)
  if (BT_Util::get_from_blackboard<bool>(blackboard_, "mirrored")) {
    // Mirror x-coordinate about field centerline (6 tiles = 3.048m)
    target_world.x() = 6.0 * ghost_util::TILES_TO_METERS - target_world.x();
    // Mirror heading angle
    end_theta_rad = ghost_util::WrapAngle2PI(M_PI - end_theta_rad);
  }
  
  // 8. Generate the World-Frame Trajectory
  switch (traj_type) {
    case motion_planning::trajectory_type_e::CUBIC_BEZIER:
      traj = motion_planning::generateCubicBezierCurve(
        start_pos, start_angle, target_world, end_theta_rad, lead_m, num_points);
      break;
    case motion_planning::trajectory_type_e::QUADRATIC_BEZIER:
      traj = motion_planning::generateQuadraticBezierCurve(
        start_pos, target_world, end_theta_rad, lead_m, num_points);
      break;
  }
  
  traj.calculateRemainingPathLengths();
  
  // 9. Visualize in RViz
  if (node_ptr_) {
    viz_msg_.markers.clear();
    visualization::getTrajectoryMsg(traj, viz_msg_, 10);
    trajectory_viz_pub_ptr_->publish(viz_msg_);
  }
  
  // 10. Set backwards flag for Pure Pursuit
  traj.backwards = backwards;
  *tank_trajectory_ptr_ = traj;
  RCLCPP_INFO(node_ptr_->get_logger(), "Bezier: Completed");
  return BT::NodeStatus::SUCCESS;
}

void GenerateBezierPath_modified::onHalted()
{
  resetStatus();
}

} // namespace ghost_tank {
