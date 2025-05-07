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
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/int64.hpp>

#include <ghost_tank/tank_tree.hpp>
#include <ghost_tank/tank_odom.hpp>
#include <ghost_tank/pdcontrol.hpp>
#include <ghost_tank/boomerang.hpp>

namespace ghost_tank
{

class AlphaJerryPlugin : public ghost_ros_interfaces::V5RobotBase
{
public:
  using JoyPtr = std::shared_ptr<ghost_v5_interfaces::devices::JoystickDeviceData>;

  AlphaJerryPlugin();

  void initialize() override;
  void disabled() override;
  void autonomous(double current_time) override;
  void teleop(double current_time) override;
  void onNewSensorData() override;

protected:
  // Construction
  void populateMotorNames();
  void populateDigitalIONames();

  // Initialization
  void initROSComms();
  void initEstimation();
  void initIntake();
  void initTankModel();
  void initAutonomy();

  // onNewSensorData
  void updateConveyorPositionSensing();
  void updateAndPublishOdometry();
  void publishBaseTwist();
  void publishTrajectoryVisualization();

  // Teleop
  bool runAutonFromDriver(JoyPtr joy_data, double current_time);
  void toggleBagRecorder(JoyPtr joy_data);

  /**
   * @brief Handles intaking logic
   *
   * Holding R2 alone intakes the Ground Pickup, and aligns the next conveyor hook for intaking rings
   * Holding R alone outtakes the Ground Pickup
   * Holding R1 intakes the Conveyor
   * Holding L1 alone outtakes the Conveyor
   * Holding R2 and L1 will initiate the ejector sequence for the current ring
   *
   * @param R2
   * @param R1
   * @param L1
   * @param R
   * @param current_time
   */
  void updateIntake(bool R2, bool R1, bool L1, bool R, double current_time);

  void updateIntakeFromJoystick(JoyPtr joy_data, bool shift_l, bool shift_r, double current_time);
  void toggleBite(bool signal);

  void updateClamp(bool close, bool open, bool shift2);
  void updateGoalRush(bool left_rush, bool right_rush, bool enabled);

  void updateScissor(bool up, bool down, bool enabled);
  void updateT1Climb(bool up, bool down, bool enabled);

  void ringDetector(bool active, double current_time, bool want_red, bool store_ring);
  void updateMusic(JoyPtr joy_data, double current_time);

  void updateDrivetrain(JoyPtr joy_data);

  void resetBT();


  // Output
  void playMusic(std::string m);
  void playTTS(std::string m);

  void colorTargetButtonCallback(const std_msgs::msg::Int64::SharedPtr msg);
  void mirroredButtonCallback(const std_msgs::msg::Int64::SharedPtr msg);

  void resetWorldPose();

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_tank_viz_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_trajectory_viz_pub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_base_twist_cmd_pub;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_des_twist_pub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_cur_twist_pub;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_des_pos_pub;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_err_pos_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_set_pose_publisher;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_tts_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_music_pub;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_reset_ekf_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_reset_pf_pub;

  void publishDesiredTwist(Eigen::Vector3d twist);
  void publishCurrentTwist(Eigen::Vector3d twist);
  void publishDesiredPose(Eigen::Vector3d pose);
  void publishErrorPose(Eigen::Vector3d pose);

  // Subscribers
  void worldOdometryUpdateCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void worldOdometryUpdateCallbackBackup(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_robot_pose_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_robot_backup_pose_sub;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_robot_color;
  void colorCallback(const std_msgs::msg::String msg)
  {
    m_color = msg.data;
  }
  std::string m_color;
  double m_first_color_detect_inches = INFINITY;

  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr m_button_color_target_sub;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr m_button_mirrored_sub;

  // Service Clients
  rclcpp::Client<ghost_msgs::srv::StartRecorder>::SharedPtr m_start_recorder_client;
  rclcpp::Client<ghost_msgs::srv::StopRecorder>::SharedPtr m_stop_recorder_client;

  // Tank Model
  std::shared_ptr<TankModel> m_tank_model_ptr;

  // Autonomy
  std::string bt_path_;
  std::shared_ptr<TankTree> bt_;
  std::shared_ptr<TankTree> bt_interaction;

  // Motion Planner
  double m_search_radius = 0.0;
  Eigen::Vector3d m_desired_pose = Eigen::Vector3d::Zero();
  Eigen::Vector3d m_desired_twist = Eigen::Vector3d::Zero();
  Eigen::Vector3d m_final_pose = Eigen::Vector3d::Zero();
  double m_move_to_pose_kp_xy = 0.0;
  double m_move_to_pose_kd_xy = 0.0;
  double m_move_to_pose_kp_theta = 0.0;
  double m_move_to_pose_kd_theta = 0.0;

  // Odometry
  std::shared_ptr<TankOdometry> m_odom_ptr;
  double m_imu_yaw;
  Eigen::Vector3d m_last_odom_pose = Eigen::Vector3d::Zero();

  Eigen::Vector3d m_curr_odom_pose = Eigen::Vector3d::Zero();

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
  double m_init_sigma_x = 0.2;        // 99% within +-24" (two tiles)
  double m_init_sigma_y = 0.2;        // 99% within +-24" (two tiles)
  double m_init_sigma_theta = 0.35;   // 99% within 60 degrees
  double m_init_world_x = 0.0;
  double m_init_world_y = 0.0;
  double m_init_world_theta = 0.0;
  static constexpr size_t m_cov_n = 6 * 6;

  Eigen::Vector2d m_reset_pose_xy_m;
  double m_reset_pose_angle_rad;
  std::vector<double> m_initial_estimate_covariance;

  bool m_use_backup_estimator = false;
  bool m_reset_world_pose = false;
  bool m_clamp_closed{false};
  bool m_bite_closed{false};
  bool m_goal_rush_active{false};
  bool m_goal_rush_clamp_active{false};

  // Conveyor
  double m_conveyor_ticks_per_loop{0.0};
  double m_conveyor_ticks_per_hook{0.0};

  double m_conveyor_position_abs{0.0};
  double m_conveyor_position_rel{0.0};
  double m_hook_fraction{0.0};

  double m_conveyor_hook_align_threshold{0.0};
  double m_conveyor_hook_align_power{0.0};
  double m_conveyor_last_aligned_position{0.0};
  bool m_conveyor_hook_is_aligned{false};

  double m_conveyor_hook_throw_fraction{0.0};
  double m_conveyor_hook_throw_duration{0.0};
  double m_conveyor_throw_start_time{0.0};
  bool m_conveyor_hook_is_ejecting{false};
  bool m_conveyor_is_throwing{false};

  // Neutral Stake Arm
  double m_neutral_stake_arm_kp{0.0};
  double m_neutral_stake_arm_gear_ratio{0.0};
  double m_neutral_stake_arm_rest_pos_deg{0.0};
  double m_neutral_stake_arm_loading_pos_deg{0.0};
  double m_neutral_stake_arm_loaded_pos_deg{0.0};
  double m_neutral_stake_arm_score_neutral_pos_deg{0.0};
  double m_neutral_stake_arm_score_alliance_pos_deg{0.0};
  double m_neutral_stake_arm_down_pos_deg{0.0};
  double m_neutral_stake_arm_des_pos{0.0};
  double m_scissor_max_extension{0.0};
  double m_scissor_reset_extension{0.0};
  int m_arm_mode{0};

  // Digital IO
  std::vector<bool> m_digital_io;
  std::unordered_map<std::string, size_t> m_digital_io_name_map;

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
  bool m_color_target_red = false;
  bool m_mirrored = false;

  bool m_interaction_started = false;
  bool m_sim_mode = false;

  // boomerang
  double m_max_speed_linear;
  double m_max_speed_angular;

  // pure pursuit
  int m_past_index = 0;
  int m_next_index = 0;

  std::shared_ptr<Boomerang> m_boomerang;
  std::shared_ptr<PDControl> m_pd_control;
  std::shared_ptr<PDControl> m_pd_control_threshold;

  std::vector<std::string> m_right_drive_motor_names;
  std::vector<std::string> m_left_drive_motor_names;
  std::vector<std::string> m_all_drive_motor_names;

  std::unordered_map<std::string, int> digital_io_port_map;

  // Current Limiting
  std::vector<double> m_loop_current_limits;
  int m_num_motors{16};

  // ring detection
  bool m_ring_found = false;
  int m_ring_color = 0;
  std::map<std::string, int> m_color_map;
};

} // namespace ghost_tank
