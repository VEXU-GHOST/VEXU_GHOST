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


#include "ghost_tank/bt_nodes/generate_bezier_path.hpp"

namespace ghost_tank
{

using motion_planning::TRAJECTORY_STRING_ENUM_MAP;

GenerateBezierPath::GenerateBezierPath(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "tank_trajectory_ptr", tank_trajectory_ptr_);
}

BT::PortsList GenerateBezierPath::providedPorts()
{
  return {
    BT::InputPort<double>("end_x_tiles"),
    BT::InputPort<double>("end_y_tiles"),
    BT::InputPort<double>("end_theta_deg"),
    BT::InputPort<double>("lead"),
    BT::InputPort<bool>("backwards", false, ""),
    BT::InputPort<std::string>("type", "CUBIC_BEZIER", ""),
    BT::InputPort<int>("num_points", 250, "")
  };
}

BT::NodeStatus GenerateBezierPath::onStart()
{
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GenerateBezierPath::onRunning()
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

  // Get trajectory input data
  Eigen::Vector2d start_pos = tank_model_ptr_->getWorldPose().head<2>();
  double start_angle = tank_model_ptr_->getWorldPose()[2];
  double end_x_m = BT_Util::get_input<int>(this, "end_x_tiles") * ghost_util::TILES_TO_METERS;
  double end_y_m = BT_Util::get_input<int>(this, "end_y_tiles") * ghost_util::TILES_TO_METERS;
  double end_theta_rad = BT_Util::get_input<int>(this, "end_theta_deg") * ghost_util::DEG_TO_RAD;
  double lead_m = BT_Util::get_input<int>(this, "lead_tiles") * ghost_util::TILES_TO_METERS;

  if (BT_Util::get_input<bool>(this, "backwards")) {
    start_angle = ghost_util::FlipAnglePI(start_angle);
  }

  // Mirror path about center line of VEX field
  if (BT_Util::get_from_blackboard<bool>(blackboard_, "mirrored")) {
    end_x_m = 6.0 * ghost_util::TILES_TO_METERS - end_x_m;
    start_angle = ghost_util::WrapAngle2PI(M_PI - start_angle);
  }

  // Generate Trajectory
  switch (traj_type) {
    case motion_planning::trajectory_type_e::CUBIC_BEZIER:
      traj = motion_planning::generateCubicBezierCurve(start_pos, start_angle, Eigen::Vector2d(end_x_m, end_y_m), end_theta_rad, lead_m, num_points);
      break;
    case motion_planning::trajectory_type_e::QUADRATIC_BEZIER:
      traj = motion_planning::generateQuadraticBezierCurve(start_pos, Eigen::Vector2d(end_x_m, end_y_m), end_theta_rad, lead_m, num_points);
      break;
  }

  traj.calculateRemainingPathLengths();

  *tank_trajectory_ptr_ = traj;
  return BT::NodeStatus::SUCCESS;
}

void GenerateBezierPath::onHalted()
{
  resetStatus();
}

} // namespace ghost_tank {
