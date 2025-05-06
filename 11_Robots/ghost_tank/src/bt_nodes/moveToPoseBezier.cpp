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
 #include "ghost_tank/pdcontrol.hpp"

using std::placeholders::_1;

namespace ghost_tank
{

// If your Node has ports, you must use this constructor signature
MoveToPoseBezier::MoveToPoseBezier(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  // std::cout << "[MoveToPoseBezier::MoveToPoseBezier]" << std::endl;

  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "pd_control_ptr", pd_control_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "pd_control_threshold_ptr", pd_control_threshold_ptr_);

  if (!node_ptr_->has_parameter("behavior_tree.trajectory_topic")) {
    node_ptr_->declare_parameter(
      "behavior_tree.trajectory_topic",
      "/motion_planner/trajectory");
  }
  std::string bezier_planner_topic =
    node_ptr_->get_parameter("behavior_tree.trajectory_topic").as_string();

  trajectory_pub_ = node_ptr_->create_publisher<ghost_msgs::msg::RobotTrajectory>(
    bezier_planner_topic,
    10);

  curr_angle_pub = node_ptr_->create_publisher<std_msgs::msg::Float64>("/test/curr_angle", 10);
  des_angle_pub = node_ptr_->create_publisher<std_msgs::msg::Float64>("/test/des_angle", 10);

  first_loop_ = true;

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

/// Method called once, when transitioning from the state IDLE.
/// If it returns RUNNING, this becomes an asynchronous node.
BT::NodeStatus MoveToPoseBezier::onStart()
{
  first_loop_ = true;
  settling_ = false;
  // plan_time_ = std::chrono::();
  return BT::NodeStatus::RUNNING;
}

/// when the method halt() is called and the action is RUNNING, this method is invoked.
/// This is a convenient place todo a cleanup, if needed.
void MoveToPoseBezier::onHalted()
{
  resetStatus();
}

BT::NodeStatus MoveToPoseBezier::onRunning()
{
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

  // First control cycle
  if (first_loop_) {
    RCLCPP_INFO(node_ptr_->get_logger(), "MoveToPoseBezier: Started");
    start_time_ = std::chrono::system_clock::now();
    first_loop_ = false;
    GeneratePath();
    RCLCPP_INFO(node_ptr_->get_logger(), "posX_m: %f", posX_m);
    RCLCPP_INFO(node_ptr_->get_logger(), "posY_m: %f", posY_m);
    RCLCPP_INFO(node_ptr_->get_logger(), "theta_rad: %f", theta_rad);
    settling_ = false;
  }

  // Calculate end pose error for exit conditions
  Eigen::Vector2d des_pos = Eigen::Vector2d(posX_m, posY_m);

  double dist_err = (des_pos - tank_model_ptr_->getWorldPose().head<2>()).norm();
  double theta_err = std::fabs(ghost_util::SmallestAngleDistRad(theta_rad, tank_model_ptr_->getWorldAngleRad()));

  bool xy_satisfied = dist_err < xy_exit_threshold_m;
  bool angle_satisfied = theta_err < angle_exit_threshold_rad;
  bool xy_vel_satisfied = tank_model_ptr_->getWorldTwist().head<2>().norm() < lin_vel_exit_threshold_mps;
  bool ang_vel_satisfied = std::fabs(tank_model_ptr_->getWorldTwist().z()) < ang_vel_exit_threshold_radps;

  // Check exit conditions
  if (xy_satisfied && angle_satisfied && xy_vel_satisfied) {
    bool translation_only = !use_theta;
    if (translation_only || use_theta && ang_vel_satisfied) {
      RCLCPP_INFO(node_ptr_->get_logger(), "MoveToPoseBezier: Success");
      // tank_model_ptr_->driveCommand(0.0, 0.0);
      return BT::NodeStatus::SUCCESS;
    }
  }

  // Check timeout condition
  int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time_).count();
  if (time_elapsed > abs(timeout_ms)) {
    RCLCPP_WARN(node_ptr_->get_logger(), "MoveToPoseBezier: Skipped");
    // tank_model_ptr_->driveCommand(0.0, 0.0);
    return BT::NodeStatus::SUCCESS;
  }

  // Run control
  PurePursuit();
  publishTrajectoryVisualization();

  return BT::NodeStatus::RUNNING;
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

void MoveToPoseBezier::PurePursuit()
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
  past_index_ = x_trajectory.size()-1; // Initialize to end so if we are way off the path (no points inside pursuit radius), we go straight to final pose
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
    command = pd_control_threshold_ptr_->theta_pid(tank_model_ptr_->getWorldPose(), tank_model_ptr_->getWorldTwist(), final_pose_);
    
    // Once we start settling, never exit to avoid instability.
    settling_ = true;
  } else {
    // Chase the carrot. If within pursuit radius, ignore lateral error in xy control.
    command = pd_control_ptr_->tank_pid(tank_model_ptr_->getWorldPose(), tank_model_ptr_->getWorldTwist(), desired_pose, final_pose_, backwards, within_pursuit_radius);
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
  left_cmd *= normalizer;
  right_cmd *= normalizer;

  BT_Util::put_in_blackboard(blackboard_, "fwd_cmd", fwd_cmd);
  BT_Util::put_in_blackboard(blackboard_, "turn_cmd", turn_cmd);

  tank_model_ptr_->driveCommand(fwd_cmd, turn_cmd);
}

void MoveToPoseBezier::publishTrajectoryVisualization()
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
  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 0.1;
  marker.color.r = 1.0;
  marker.color.a = 0.5;

  for (int i = 0; i < robot_trajectory_.x_trajectory.position_vector.size(); i += 25) {
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
