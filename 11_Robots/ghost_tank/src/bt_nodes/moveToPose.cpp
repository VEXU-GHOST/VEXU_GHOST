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

#include "ghost_tank/bt_nodes/moveToPose.hpp"
#include "ghost_tank/control/tank_pid_controller.hpp"

using std::placeholders::_1;

namespace ghost_tank
{

// If your Node has ports, you must use this constructor signature
MoveToPose::MoveToPose(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  // std::cout << "[MoveToPose::MoveToPose]" << std::endl;

  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "pd_control_ptr", pd_control_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "pd_control_threshold_ptr", pd_control_threshold_ptr_);

  curr_angle_pub = node_ptr_->create_publisher<std_msgs::msg::Float64>("/test/curr_angle", 10);
  des_angle_pub = node_ptr_->create_publisher<std_msgs::msg::Float64>("/test/des_angle", 10);
  fwd_cmd_pub = node_ptr_->create_publisher<std_msgs::msg::Float64>("/test/fwd_cmd", 10);
  turn_cmd_pub = node_ptr_->create_publisher<std_msgs::msg::Float64>("/test/turn_cmd", 10);
  left_cmd_pub = node_ptr_->create_publisher<std_msgs::msg::Float64>("/test/left_cmd", 10);
  right_cmd_pub = node_ptr_->create_publisher<std_msgs::msg::Float64>("/test/right_cmd", 10);

  first_loop_ = true;
}

/// Method called once, when transitioning from the state IDLE.
/// If it returns RUNNING, this becomes an asynchronous node.
BT::NodeStatus MoveToPose::onStart()
{
  first_loop_ = true;
  settling_ = false;
  return BT::NodeStatus::RUNNING;
}

/// when the method halt() is called and the action is RUNNING, this method is invoked.
/// This is a convenient place todo a cleanup, if needed.
void MoveToPose::onHalted()
{
  resetStatus();
}

BT::NodeStatus MoveToPose::onRunning()
{
  // Get blackboard data
  GetBlackboardData();

  // First control cycle
  if (first_loop_) {
    RCLCPP_INFO(node_ptr_->get_logger(), "MoveToPose: Started");
    FirstLoop();
    start_time_ = std::chrono::system_clock::now();
    first_loop_ = false;
    GeneratePath();
    publishTrajectoryVisualization();
    // RCLCPP_INFO(node_ptr_->get_logger(), "posX_m: %f", posX_m);
    // RCLCPP_INFO(node_ptr_->get_logger(), "posY_m: %f", posY_m);
    // RCLCPP_INFO(node_ptr_->get_logger(), "theta_rad: %f", theta_rad);
    settling_ = false;
  }

  // // Calculate end pose error for exit conditions
  // GetFinalPose();

  if (CheckEndConditions(final_pose_)){
    return BT::NodeStatus::SUCCESS;
  }

  // Run control
  PurePursuit();

  return BT::NodeStatus::RUNNING;
}

bool MoveToPose::CheckEndConditions(Eigen::Vector3d des_pos){
  double dist_err = (des_pos.head<2>() - tank_model_ptr_->getWorldPose().head<2>()).norm();
  double theta_err = std::fabs(ghost_util::SmallestAngleDistRad(des_pos.z(), tank_model_ptr_->getWorldAngleRad()));

  bool xy_satisfied = dist_err < xy_exit_threshold_m;
  bool angle_satisfied = theta_err < angle_exit_threshold_rad;
  bool xy_vel_satisfied = tank_model_ptr_->getWorldTwist().head<2>().norm() < lin_vel_exit_threshold_mps;
  bool ang_vel_satisfied = std::fabs(tank_model_ptr_->getWorldTwist().z()) < ang_vel_exit_threshold_radps;

  // Check exit conditions
  if (xy_satisfied && angle_satisfied && xy_vel_satisfied) {
    bool translation_only = !use_theta;
    if (translation_only || use_theta && ang_vel_satisfied) {
      RCLCPP_INFO(node_ptr_->get_logger(), "MoveToPose: Success");
      return true;
    }
  }

  // Check timeout condition
  int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time_).count();
  if (time_elapsed > abs(timeout_ms)) {
    RCLCPP_WARN(node_ptr_->get_logger(), "MoveToPose: Skipped");
    return true;
  }

  // Continue along the path
  return false;
}

void MoveToPose::PurePursuit()
{
  // Get current state
  Eigen::Vector2d current_pos = Eigen::Vector2d(tank_model_ptr_->getWorldPose().head<2>());

  // Get desired trajectory
  auto x_trajectory = robot_trajectory_.x_trajectory.position_vector;
  auto y_trajectory = robot_trajectory_.y_trajectory.position_vector;
  auto theta_trajectory = robot_trajectory_.theta_trajectory.position_vector;

  // Load terminal pose from trajectory
  final_pose_ = Eigen::Vector3d(x_trajectory.back(), y_trajectory.back(), theta_trajectory.back());
  // Find intersection of path and pursuit radius
  Eigen::Vector3d desired_pose;
  past_index_ = x_trajectory.size() - 1; // Initialize to end so if we are way off the path (no points inside pursuit radius), we go straight to final pose
  for (int i = 0; i < x_trajectory.size(); ++i) {
    double distance = (current_pos - Eigen::Vector2d(x_trajectory[i], y_trajectory[i])).norm();
    if (distance < search_radius) {
      past_index_ = i;
    }
  }
  desired_pose = Eigen::Vector3d(x_trajectory[past_index_], y_trajectory[past_index_], 0.0);
  BT_Util::put_in_blackboard(blackboard_, "desired_pose", desired_pose);

  // Select control strategy based on distance to target
  double dist_err = (final_pose_.head<2>() - current_pos).norm();
  bool within_pursuit_radius = dist_err < search_radius;
  bool within_xy_exit_threshold = dist_err < xy_exit_threshold_m;

  Eigen::Vector2d command;
  if (within_xy_exit_threshold || settling_) {
    // We are within xy_exit_threshold, switch to pure angle control
    // command = pd_control_threshold_ptr_->theta_pid(tank_model_ptr_->getWorldPose(), tank_model_ptr_->getWorldTwist(), final_pose_);

    // Once we start settling, never exit to avoid instability.
    settling_ = true;
  } else {
    // Chase the carrot. If within pursuit radius, ignore lateral error in xy control.
    // command = pd_control_ptr_->tank_pid(tank_model_ptr_->getWorldPose(), tank_model_ptr_->getWorldTwist(), desired_pose, final_pose_, backwards, within_pursuit_radius);
  }

  // Clamp steering and lateral thrust to bounds
  auto fwd_cmd = ghost_util::clamp(command[0], -max_speed_linear_percent, max_speed_linear_percent);
  auto turn_cmd = ghost_util::clamp(command[1], -max_speed_angular_percent, max_speed_angular_percent);

  // Normalize to avoid output saturation.
  double left_cmd = fwd_cmd - turn_cmd;
  double right_cmd = fwd_cmd + turn_cmd;

  // Scale commands so that max command equals full thrust
  double normalizer = 1.0 / std::max(1.0, std::max(std::fabs(left_cmd), std::fabs(right_cmd)));
  // double normalizer = 1.0;
  fwd_cmd *= normalizer;
  turn_cmd *= normalizer;

  publishDrivetrainCommands(fwd_cmd, turn_cmd);

  BT_Util::put_in_blackboard(blackboard_, "fwd_cmd", fwd_cmd);
  BT_Util::put_in_blackboard(blackboard_, "turn_cmd", turn_cmd);

  tank_model_ptr_->driveCommand(fwd_cmd, turn_cmd);
}

void MoveToPose::publishDrivetrainCommands(double fwd_cmd, double turn_cmd)
{
  double left_cmd = fwd_cmd - turn_cmd;
  double right_cmd = fwd_cmd + turn_cmd;

  std_msgs::msg::Float64 fwd_cmd_msg;
  fwd_cmd_msg.data = fwd_cmd;
  fwd_cmd_pub->publish(fwd_cmd_msg);

  std_msgs::msg::Float64 turn_cmd_msg;
  turn_cmd_msg.data = turn_cmd;
  turn_cmd_pub->publish(turn_cmd_msg);

  std_msgs::msg::Float64 left_cmd_msg;
  left_cmd_msg.data = left_cmd;
  left_cmd_pub->publish(left_cmd_msg);

  std_msgs::msg::Float64 right_cmd_msg;
  right_cmd_msg.data = right_cmd;
  right_cmd_pub->publish(right_cmd_msg);
}

void MoveToPose::publishTrajectoryVisualization()
{
  std_msgs::msg::Float64 curr_angle_msg;
  curr_angle_msg.data = tank_model_ptr_->getWorldAngleRad();
  curr_angle_pub->publish(curr_angle_msg);

  std_msgs::msg::Float64 des_angle_msg;
  des_angle_msg.data = final_pose_.z();
  des_angle_pub->publish(des_angle_msg);

  visualization_msgs::msg::MarkerArray msg{};

  double search_radius = BT_Util::get_input<double>(this, "search_radius_tiles") * tile_to_meters;

  Eigen::Vector3d desired_pose;
  BT_Util::get_from_blackboard(blackboard_, "desired_pose", desired_pose);

  visualization_msgs::msg::Marker search_radius_marker{};
  search_radius_marker.header.frame_id = "base_link";
  search_radius_marker.header.stamp = node_ptr_->get_clock()->now();
  search_radius_marker.id = 1;
  search_radius_marker.type = 3;          // cylinder type
  search_radius_marker.action = 0;
  search_radius_marker.scale.x = 2 * search_radius;
  search_radius_marker.scale.y = 2 * search_radius;
  search_radius_marker.scale.z = 0.01;
  search_radius_marker.color.b = 1.0;
  search_radius_marker.color.a = 0.3;

  visualization_msgs::msg::Marker carrot{};
  carrot.header.frame_id = "map";
  carrot.header.stamp = node_ptr_->get_clock()->now();
  carrot.id = 2;
  carrot.type = 4;          // line type
  carrot.action = 0;
  carrot.pose.position.z = 0.01;
  carrot.scale.x = 0.01;
  carrot.scale.y = 1.0;
  carrot.scale.z = 1.0;
  carrot.color.r = 1.0;
  carrot.color.b = 1.0;
  carrot.color.a = 0.5;
  geometry_msgs::msg::Point p_robot;
  p_robot.x = tank_model_ptr_->getWorldPose().x();
  p_robot.y = tank_model_ptr_->getWorldPose().y();
  p_robot.z = 0.0;
  geometry_msgs::msg::Point p_carrot;
  p_carrot.set__x(desired_pose.x());
  p_carrot.set__y(desired_pose.y());
  p_carrot.z = 0.0;
  carrot.points.push_back(p_robot);
  carrot.points.push_back(p_carrot);

  visualization_msgs::msg::Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = node_ptr_->get_clock()->now();
  marker.id = 0;
  marker.type = 8;          // points type
  marker.action = 0;
  marker.scale.x = 0.03;
  marker.scale.y = 0.03;
  marker.scale.z = 0.1;
  marker.color.g = 1.0;
  marker.color.a = 0.5;

  for (int i = 0; i < robot_trajectory_.x_trajectory.position_vector.size(); i += 5) {
    geometry_msgs::msg::Point p;
    p.x = robot_trajectory_.x_trajectory.position_vector[i];
    p.y = robot_trajectory_.y_trajectory.position_vector[i];
    p.z = 0.0;
    marker.points.push_back(p);
  }

  visualization_msgs::msg::Marker end_marker{};
  end_marker.header.frame_id = "map";
  end_marker.header.stamp = node_ptr_->get_clock()->now();
  end_marker.id = 4;
  end_marker.type = 0;          // arrow type
  end_marker.action = 0;
  end_marker.pose.position.x = final_pose_.x();
  end_marker.pose.position.y = final_pose_.y();
  ghost_util::yawToQuaternionRad(
    final_pose_.z(), end_marker.pose.orientation.w, end_marker.pose.orientation.x,
    end_marker.pose.orientation.y, end_marker.pose.orientation.z);
  end_marker.scale.x = 0.1;
  end_marker.scale.y = 0.025;
  end_marker.scale.z = 0.025;
  end_marker.color.r = 1.0;
  end_marker.color.a = 1.0;

  msg.markers.push_back(search_radius_marker);
  msg.markers.push_back(carrot);
  msg.markers.push_back(marker);
  msg.markers.push_back(end_marker);

  BT_Util::get_from_blackboard(blackboard_, "trajectory_viz_pub", trajectory_viz_pub_);
  trajectory_viz_pub_->publish(msg);
}

}  // namespace ghost_tank
