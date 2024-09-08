/*
 *   Copyright (c) 2024 Maxx Wilson
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

#include <ghost_planners/robot_trajectory.hpp>
#include <ghost_ros_interfaces/competition/v5_robot_base.hpp>
#include <ghost_ros_interfaces/msg_helpers/msg_helpers.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <ghost_msgs/msg/drivetrain_command.hpp>
#include <ghost_msgs/msg/robot_trajectory.hpp>
#include <ghost_msgs/srv/start_recorder.hpp>
#include <ghost_msgs/srv/stop_recorder.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <ghost_tank/tank_tree.hpp>

namespace ghost_tank
{

class tankRobotPlugin : public ghost_ros_interfaces::V5RobotBase
{
public:
  tankRobotPlugin();

  void initialize() override;
  void disabled() override;
  void autonomous(double current_time) override;
  void teleop(double current_time) override;
  void onNewSensorData() override;

protected:
  // Publishers
  void publishVisualization();
  void publishOdometry();
  void publishBaseTwist();
  void publishTrajectoryVisualization();
  void resetPose(double x, double y, double theta);

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_tank_viz_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_trajectory_viz_pub;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_base_twist_cmd_pub;


  // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_cur_pos_pub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_des_vel_pub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_cur_vel_pub;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_des_pos_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_set_pose_publisher;

  void publishDesiredTwist(double des_vel_x, double des_vel_y, double des_theta_vel);
  void publishCurrentTwist(double curr_vel_x, double curr_vel_y, double des_vel_theta);
  void publishDesiredPose(double des_x, double des_y, double des_theta);

  // Subscribers
  void worldOdometryUpdateCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void worldOdometryUpdateCallbackBackup(const nav_msgs::msg::Odometry::SharedPtr msg);
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_robot_pose_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_robot_backup_pose_sub;

  // Service Clients
  rclcpp::Client<ghost_msgs::srv::StartRecorder>::SharedPtr m_start_recorder_client;
  rclcpp::Client<ghost_msgs::srv::StopRecorder>::SharedPtr m_stop_recorder_client;

  // tank Model
  void updateDrivetrainMotors();
  std::shared_ptr<tankModel> m_tank_model_ptr;

  // Autonomy
  std::string bt_path_;
  std::shared_ptr<tankTree> bt_;

  // Motion Planner
  double m_move_to_pose_kp_xy = 0.0;
  double m_move_to_pose_kd_xy = 0.0;
  double m_move_to_pose_kp_theta = 0.0;
  double m_move_to_pose_kd_theta = 0.0;

  // Odometry
  Eigen::Vector2d m_last_odom_loc = Eigen::Vector2d::Zero();
  double m_last_odom_angle = 0.0;

  Eigen::Vector2d m_curr_odom_loc = Eigen::Vector2d::Zero();
  double m_curr_odom_angle = 0.0;

  Eigen::Vector3d m_curr_odom_std = Eigen::Vector3d::Zero();
  Eigen::Vector3d m_curr_odom_cov = Eigen::Vector3d::Zero();
  double m_k1 = 0.0;
  double m_k2 = 0.0;
  double m_k3 = 0.0;
  double m_k4 = 0.0;
  double m_k5 = 0.0;
  double m_k6 = 0.0;
  double m_k7 = 0.0;
  double m_k8 = 0.0;
  double m_k9 = 0.0;

  // Pose Reset Covariances
  double m_init_sigma_x = 0.2;              // 99% within +-24" (two tiles)
  double m_init_sigma_y = 0.2;              // 99% within +-24" (two tiles)
  double m_init_sigma_theta = 0.35;         // 99% within 60 degrees
  double m_init_world_x = 0.0;
  double m_init_world_y = 0.0;
  double m_init_world_theta = 0.0;
  bool m_use_backup_estimator = false;
  double m_intake_setpoint = 7.0;

  // Digital IO
  std::vector<bool> m_digital_io;
  std::unordered_map<std::string, size_t> m_digital_io_name_map;

  // Claw
  bool m_claw_btn_pressed = false;
  bool m_claw_open = true;
  bool claw_auto_extended = false;

  // Tail
  bool m_tail_btn_pressed = false;
  bool m_tail_down = false;

  // Climb Mode
  bool m_climb_mode = false;

  // Stick Mode
  bool m_tail_mode = false;

  // Bag Recorder
  bool m_recording_btn_pressed = false;
  bool m_recording = false;

  // Field vs Robot Oriented Control
  bool m_toggle_tank_field_control_btn_pressed = false;

  // Angle vs Velocity Control
  bool m_toggle_tank_angle_control_btn_pressed = false;
  double m_angle_target = 0.0;
  double m_joy_angle_control_threshold = 0.0;

  // Slew Rate Control
  double m_joystick_slew_rate = 2.0;
  double m_last_x_cmd = 0.0;
  double m_last_y_cmd = 0.0;
  double m_last_theta_cmd = 0.0;
  double m_curr_x_cmd = 0.0;
  double m_curr_y_cmd = 0.0;
  double m_curr_theta_cmd = 0.0;

  // Auton States
  bool m_auton_button_pressed = false;
  int m_auton_index = 0;

  // stick
  double m_stick_angle_start = 0;
  double m_stick_angle_kick = 0;

  // Burnout Prevention
  float m_burnout_absolute_current_threshold_ma;
  float m_burnout_absolute_rpm_threshold;
  long m_burnout_stall_duration_ms;
  long m_burnout_cooldown_duration_ms;

  rclcpp::Time m_intake_stall_start;
  rclcpp::Time m_intake_cooldown_start;
  bool m_intake_stalling = false;
  bool m_intake_cooling_down = false;

  bool m_interaction_started = false;
};

} // namespace ghost_tank
