/*
 *   Copyright (c) 2024 Jake Wendling
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

#include "ghost_tank/bt_nodes/moveToPoseBezier.hpp"

using std::placeholders::_1;

namespace ghost_tank
{

// If your Node has ports, you must use this constructor signature
MoveToPoseBezier::MoveToPoseBezier(const std::string & name, const BT::NodeConfig & config)
: MoveToPose::MoveToPose(name, config)
{
  std::cout << "[MoveToPoseBezier::MoveToPoseBezier]" << std::endl;

  bezier_ = std::make_shared<BezierCurve>();
}

// It is mandatory to define this STATIC method.
BT::PortsList MoveToPoseBezier::providedPorts()
{
  return {
    BT::InputPort<double>("posX_tiles"),
    BT::InputPort<double>("posY_tiles"),
    BT::InputPort<double>("theta_deg"),
    BT::InputPort<double>("search_radius_tiles"),
    BT::InputPort<double>("lead"),
    BT::InputPort<double>("xy_exit_threshold_tiles"),
    BT::InputPort<double>("angle_exit_threshold_deg"),
    BT::InputPort<double>("lin_vel_exit_threshold_tps", 1000.0, ""),
    BT::InputPort<double>("ang_vel_exit_threshold_dps", 1000.0, ""),
    BT::InputPort<double>("max_speed_linear_percent"),
    BT::InputPort<double>("max_speed_angular_percent"),
    BT::InputPort<int>("timeout_ms"),
    BT::InputPort<bool>("use_theta"),
    BT::InputPort<bool>("backwards"),
  };
}

void MoveToPoseBezier::GetBlackboardData(){
  // Get blackboard data
  posX_m = BT_Util::get_input<double>(this, "posX_tiles") * tile_to_meters;
  posY_m = BT_Util::get_input<double>(this, "posY_tiles") * tile_to_meters;
  theta_rad = BT_Util::get_input<double>(this, "theta_deg") * ghost_util::DEG_TO_RAD;
  xy_exit_threshold_m = BT_Util::get_input<double>(this, "xy_exit_threshold_tiles", 0.1) * tile_to_meters;
  angle_exit_threshold_rad = BT_Util::get_input<double>(this, "angle_exit_threshold_deg", 5.0) * ghost_util::DEG_TO_RAD;
  lin_vel_exit_threshold_mps = BT_Util::get_input<double>(this, "lin_vel_exit_threshold_tps", 100.0) * tile_to_meters;
  ang_vel_exit_threshold_radps = BT_Util::get_input<double>(this, "ang_vel_exit_threshold_dps", 1000.0) * ghost_util::DEG_TO_RAD;
  timeout_ms = BT_Util::get_input<int>(this, "timeout_ms");
  use_theta = BT_Util::get_input<bool>(this, "use_theta", true);
  backwards = BT_Util::get_input<bool>(this, "backwards", false);
  search_radius = BT_Util::get_input<double>(this, "search_radius_tiles", 0.3) * tile_to_meters;
  lead = BT_Util::get_input<double>(this, "lead", 1.0);
  max_speed_linear_percent = BT_Util::get_input<double>(this, "max_speed_linear_percent", 1.0);
  max_speed_angular_percent = BT_Util::get_input<double>(this, "max_speed_angular_percent", 1.0);

  // Invert commands when mirrored
  if (BT_Util::get_from_blackboard<bool>(blackboard_, "mirrored")) {
    posX_m = 6.0 * tile_to_meters - posX_m;
    theta_rad = ghost_util::WrapAngle2PI(M_PI - theta_rad);
  }
}

void MoveToPoseBezier::FirstLoop(){
  RCLCPP_INFO(node_ptr_->get_logger(), "MoveToPoseBezier: Started");
  RCLCPP_INFO(node_ptr_->get_logger(), "posX_m: %f", posX_m);
  RCLCPP_INFO(node_ptr_->get_logger(), "posY_m: %f", posY_m);
  RCLCPP_INFO(node_ptr_->get_logger(), "theta_rad: %f", theta_rad);
}

void MoveToPoseBezier::GeneratePath()
{
  bezier_->set_lead(lead);
  double end_angle_rad = theta_rad;
  Eigen::Vector3d curr_world_pose = Eigen::Vector3d(tank_model_ptr_->getWorldPose());

  if (backwards) {
    end_angle_rad = ghost_util::FlipAnglePI(end_angle_rad);
    curr_world_pose.z() = ghost_util::FlipAnglePI(curr_world_pose.z());
  }
  bezier_->set_end_point(posX_m, posY_m, end_angle_rad);
  bezier_->map_curve(curr_world_pose);

  auto points = bezier_->get_points();
  std::vector<double> x_trajectory;
  std::vector<double> y_trajectory;
  std::vector<double> theta_trajectory;
  std::vector<double> time_vector;

  for (const auto & vec : points) {
    x_trajectory.push_back(vec.x());
    y_trajectory.push_back(vec.y());
    theta_trajectory.push_back(theta_rad);
  }
  int num_points = 250;
  for (int i = 0; i <= num_points; i++) {
    time_vector.push_back(i / static_cast<double>(num_points));
  }

  robot_trajectory_.x_trajectory.position_vector = x_trajectory;
  robot_trajectory_.y_trajectory.position_vector = y_trajectory;
  robot_trajectory_.theta_trajectory.position_vector = theta_trajectory;
  robot_trajectory_.x_trajectory.threshold = xy_exit_threshold_m;
  robot_trajectory_.y_trajectory.threshold = xy_exit_threshold_m;
  robot_trajectory_.theta_trajectory.threshold = angle_exit_threshold_rad;
  robot_trajectory_.x_trajectory.time_vector = time_vector;
  robot_trajectory_.y_trajectory.time_vector = time_vector;
  robot_trajectory_.theta_trajectory.time_vector = time_vector;
}

}  // namespace ghost_tank
