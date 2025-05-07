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

#include <iostream>
#include <cmath>
#include <bits/stdc++.h>
#include <ghost_tank/tank_model.hpp>
#include <ghost_tank/alpha_jerry_plugin.hpp>
#include <ghost_util/angle_util.hpp>
#include <ghost_util/math_util.hpp>
#include <ghost_util/unit_conversion_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <ghost_util/read_path.hpp>
#include <ghost_control/models/v5_current_limiting.hpp>

using ghost_planners::RobotTrajectory;
using ghost_ros_interfaces::msg_helpers::fromROSMsg;
using std::placeholders::_1;

using ghost_v5_interfaces::devices::JoystickDeviceData;

using ghost_util::INCHES_TO_METERS;

using JoyPtr = std::shared_ptr<ghost_v5_interfaces::devices::JoystickDeviceData>;

namespace ghost_tank
{

AlphaJerryPlugin::AlphaJerryPlugin()
{
  populateMotorNames();
  populateDigitalIONames();
}

void AlphaJerryPlugin::populateMotorNames()
{
  m_right_drive_motor_names = {
    "drive_r1",
    "drive_r2",
    "drive_r3",
    "drive_r4",
    "drive_r5",
    "drive_r6",
    "drive_r7",
    "drive_r8",
  };
  m_left_drive_motor_names = {
    "drive_l1",
    "drive_l2",
    "drive_l3",
    "drive_l4",
    "drive_l5",
    "drive_l6",
    "drive_l7",
    "drive_l8",
  };

  m_all_drive_motor_names.insert(
    m_all_drive_motor_names.end(),
    m_left_drive_motor_names.begin(),
    m_left_drive_motor_names.end());

  m_all_drive_motor_names.insert(
    m_all_drive_motor_names.end(),
    m_right_drive_motor_names.begin(),
    m_right_drive_motor_names.end());
}

void AlphaJerryPlugin::populateDigitalIONames()
{
  // digital_io_port_map["goal_rush_sensor"] = 4;
  digital_io_port_map["goal_rush_l"] = 0;
  digital_io_port_map["climb"] = 1;
  digital_io_port_map["goal_rush_r"] = 2;
  digital_io_port_map["bite"] = 3;
  digital_io_port_map["clamp"] = 4;
}

//////////////////////
/// Initialization ///
//////////////////////

void AlphaJerryPlugin::initialize()
{
  initROSComms();
  initEstimation();
  initIntake();
  initTankModel();
  initAutonomy();
  resetWorldPose();
}

void AlphaJerryPlugin::initROSComms()
{
  std::cout << "[AlphaJerryPlugin::initROSComms]" << std::endl;
  // Services
  node_ptr_->declare_parameter("bag_recorder_start_topic", "bag_recorder/start");
  std::string bag_recorder_start_topic = node_ptr_->get_parameter("bag_recorder_start_topic").as_string();
  m_start_recorder_client = node_ptr_->create_client<ghost_msgs::srv::StartRecorder>(bag_recorder_start_topic);

  node_ptr_->declare_parameter("bag_recorder_stop_topic", "bag_recorder/stop");
  std::string bag_recorder_stop_topic = node_ptr_->get_parameter("bag_recorder_stop_topic").as_string();
  m_stop_recorder_client = node_ptr_->create_client<ghost_msgs::srv::StopRecorder>(bag_recorder_stop_topic);

  // Publishers
  node_ptr_->declare_parameter("joint_state_topic", "/joint_states");
  std::string joint_state_topic = node_ptr_->get_parameter("joint_state_topic").as_string();
  m_joint_state_pub = node_ptr_->create_publisher<sensor_msgs::msg::JointState>(joint_state_topic, 10);

  node_ptr_->declare_parameter("trajectory_marker_topic", "/trajectory_markers");
  std::string trajectory_marker_topic = node_ptr_->get_parameter("trajectory_marker_topic").as_string();
  m_trajectory_viz_pub = node_ptr_->create_publisher<visualization_msgs::msg::MarkerArray>(trajectory_marker_topic, 10);

  node_ptr_->declare_parameter("tank_robot_plugin.cmd_twist_topic", "/cmd_vel");
  std::string cmd_twist_topic = node_ptr_->get_parameter("tank_robot_plugin.cmd_twist_topic").as_string();
  m_base_twist_cmd_pub = node_ptr_->create_publisher<geometry_msgs::msg::Twist>(cmd_twist_topic, 10);

  node_ptr_->declare_parameter("odom_topic", "/sensors/wheel_odom");
  std::string odom_topic = node_ptr_->get_parameter("odom_topic").as_string();
  m_odom_pub = node_ptr_->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);

  // Subscriptions
  node_ptr_->declare_parameter("pose_topic", "/odometry/filtered");
  std::string pose_topic = node_ptr_->get_parameter("pose_topic").as_string();
  m_robot_pose_sub = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(pose_topic, 10, std::bind(&AlphaJerryPlugin::worldOdometryUpdateCallback, this, _1));

  node_ptr_->declare_parameter("backup_pose_topic", "/odom_ekf/odometry");
  std::string backup_pose_topic = node_ptr_->get_parameter("backup_pose_topic").as_string();
  m_robot_backup_pose_sub = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(backup_pose_topic, 10, std::bind(&AlphaJerryPlugin::worldOdometryUpdateCallbackBackup, this, _1));

  m_robot_color = node_ptr_->create_subscription<std_msgs::msg::String>("/sensors/color_sensors/intake/color", 10, std::bind(&AlphaJerryPlugin::colorCallback, this, _1));

  // Tank-Specific Publishers
  node_ptr_->declare_parameter("tank_robot_plugin.cmd_pose_topic", "/set_pose");
  std::string cmd_pose_topic = node_ptr_->get_parameter("tank_robot_plugin.cmd_pose_topic").as_string();
  m_set_pose_publisher = node_ptr_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(cmd_pose_topic, 10);

  node_ptr_->declare_parameter("tank_robot_plugin.des_twist_topic", "/des_vel");
  std::string des_twist_topic = node_ptr_->get_parameter("tank_robot_plugin.des_twist_topic").as_string();
  m_des_twist_pub = node_ptr_->create_publisher<geometry_msgs::msg::Twist>(des_twist_topic, 10);

  node_ptr_->declare_parameter("tank_robot_plugin.cur_twist_topic", "/cur_vel");
  std::string cur_twist_topic = node_ptr_->get_parameter("tank_robot_plugin.cur_twist_topic").as_string();
  m_cur_twist_pub = node_ptr_->create_publisher<geometry_msgs::msg::Twist>(cur_twist_topic, 10);

  node_ptr_->declare_parameter("tank_robot_plugin.des_pos_topic", "/des_pos");
  std::string des_pos_topic = node_ptr_->get_parameter("tank_robot_plugin.des_pos_topic").as_string();
  m_des_pos_pub = node_ptr_->create_publisher<geometry_msgs::msg::Pose>(des_pos_topic, 10);

  node_ptr_->declare_parameter("tank_robot_plugin.err_pos_topic", "/err_pos");
  std::string err_pos_topic = node_ptr_->get_parameter("tank_robot_plugin.err_pos_topic").as_string();
  m_err_pos_pub = node_ptr_->create_publisher<geometry_msgs::msg::Pose>(err_pos_topic, 10);

  m_tts_pub = node_ptr_->create_publisher<std_msgs::msg::String>("/io/speaker/tts", 1);
  m_music_pub = node_ptr_->create_publisher<std_msgs::msg::String>("/io/speaker/music", 1);

  m_button_color_target_sub = node_ptr_->create_subscription<std_msgs::msg::Int64>(
    "/io/buttons/color_target", 10,
    std::bind(&AlphaJerryPlugin::colorTargetButtonCallback, this, _1));
  m_button_mirrored_sub = node_ptr_->create_subscription<std_msgs::msg::Int64>(
    "/io/buttons/mirrored", 10,
    std::bind(&AlphaJerryPlugin::mirroredButtonCallback, this, _1));
}

void AlphaJerryPlugin::initEstimation()
{
  std::cout << "[AlphaJerryPlugin::initEstimation]" << std::endl;

  node_ptr_->declare_parameter("set_pf_pose_topic", "/set_pf_pose");
  std::string pf_pose_topic = node_ptr_->get_parameter("set_pf_pose_topic").as_string();

  rclcpp::QoS qos_profile(1);
  qos_profile.transient_local();
  m_reset_pf_pub = node_ptr_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pf_pose_topic, qos_profile);

  node_ptr_->declare_parameter("tank_robot_plugin.use_backup_estimator", false);
  m_use_backup_estimator = node_ptr_->get_parameter("tank_robot_plugin.use_backup_estimator").as_bool();

  node_ptr_->declare_parameter("particle_filter.k1", 0.0);
  node_ptr_->declare_parameter("particle_filter.k2", 0.0);
  node_ptr_->declare_parameter("particle_filter.k3", 0.0);
  node_ptr_->declare_parameter("particle_filter.k4", 0.0);
  node_ptr_->declare_parameter("particle_filter.k5", 0.0);
  node_ptr_->declare_parameter("particle_filter.k6", 0.0);
  node_ptr_->declare_parameter("particle_filter.k7", 0.0);
  node_ptr_->declare_parameter("particle_filter.k8", 0.0);
  node_ptr_->declare_parameter("particle_filter.k9", 0.0);
  m_k1 = node_ptr_->get_parameter("particle_filter.k1").as_double();
  m_k2 = node_ptr_->get_parameter("particle_filter.k2").as_double();
  m_k3 = node_ptr_->get_parameter("particle_filter.k3").as_double();
  m_k4 = node_ptr_->get_parameter("particle_filter.k4").as_double();
  m_k5 = node_ptr_->get_parameter("particle_filter.k5").as_double();
  m_k6 = node_ptr_->get_parameter("particle_filter.k6").as_double();
  m_k7 = node_ptr_->get_parameter("particle_filter.k7").as_double();
  m_k8 = node_ptr_->get_parameter("particle_filter.k8").as_double();
  m_k9 = node_ptr_->get_parameter("particle_filter.k9").as_double();

  node_ptr_->declare_parameter("particle_filter.init_world_x", m_init_world_x);
  node_ptr_->declare_parameter("particle_filter.init_world_y", m_init_world_y);
  node_ptr_->declare_parameter("particle_filter.init_world_theta", m_init_world_theta);
  node_ptr_->declare_parameter("particle_filter.init_sigma_x", m_init_sigma_x);
  node_ptr_->declare_parameter("particle_filter.init_sigma_y", m_init_sigma_y);
  node_ptr_->declare_parameter("particle_filter.init_sigma_theta", m_init_sigma_theta);
  m_init_world_x = node_ptr_->get_parameter("particle_filter.init_world_x").as_double();
  m_init_world_y = node_ptr_->get_parameter("particle_filter.init_world_y").as_double();
  m_init_world_theta = node_ptr_->get_parameter("particle_filter.init_world_theta").as_double();
  m_init_sigma_x = node_ptr_->get_parameter("particle_filter.init_sigma_x").as_double();
  m_init_sigma_y = node_ptr_->get_parameter("particle_filter.init_sigma_y").as_double();
  m_init_sigma_theta = node_ptr_->get_parameter("particle_filter.init_sigma_theta").as_double();

  node_ptr_->declare_parameter("map_ekf.initial_estimate_covariance", std::vector<double>());
  node_ptr_->declare_parameter("map_ekf.initial_state", std::vector<double>());
  node_ptr_->declare_parameter("map_ekf.initial_state_mirrored", std::vector<double>());

  m_initial_estimate_covariance = node_ptr_->get_parameter("map_ekf.initial_estimate_covariance").as_double_array();
  m_reset_pose = node_ptr_->get_parameter("map_ekf.initial_state").as_double_array();
  m_reset_pose_mirrored = node_ptr_->get_parameter("map_ekf.initial_state_mirrored").as_double_array();

  std::cout << "[AlphaJerryPlugin::initEstimation] m_initial_estimate_covariance: " << m_initial_estimate_covariance.size() << std::endl;
  std::cout << "[AlphaJerryPlugin::initEstimation] m_reset_pose: " << m_reset_pose.size() << std::endl;
}

void AlphaJerryPlugin::initIntake()
{
  std::cout << "[AlphaJerryPlugin::initIntake]" << std::endl;
  node_ptr_->declare_parameter("tank_robot_plugin.conveyor_num_links", 0.0);
  node_ptr_->declare_parameter("tank_robot_plugin.conveyor_sprocket_teeth", 0.0);
  node_ptr_->declare_parameter("tank_robot_plugin.conveyor_num_hooks", 0.0);
  double conveyor_num_links = node_ptr_->get_parameter("tank_robot_plugin.conveyor_num_links").as_double();
  double conveyor_sprocket_teeth = node_ptr_->get_parameter("tank_robot_plugin.conveyor_sprocket_teeth").as_double();
  double conveyor_num_hooks = node_ptr_->get_parameter("tank_robot_plugin.conveyor_num_hooks").as_double();
  m_conveyor_ticks_per_loop = 360.0 * conveyor_num_links / conveyor_sprocket_teeth;
  m_conveyor_ticks_per_hook = m_conveyor_ticks_per_loop / conveyor_num_hooks;

  node_ptr_->declare_parameter("tank_robot_plugin.conveyor_hook_align_threshold", 0.0);
  node_ptr_->declare_parameter("tank_robot_plugin.conveyor_hook_align_power", 0.0);
  m_conveyor_hook_align_threshold = node_ptr_->get_parameter("tank_robot_plugin.conveyor_hook_align_threshold").as_double();
  m_conveyor_hook_align_power = node_ptr_->get_parameter("tank_robot_plugin.conveyor_hook_align_power").as_double();

  node_ptr_->declare_parameter("tank_robot_plugin.conveyor_hook_throw_fraction", 0.0);
  node_ptr_->declare_parameter("tank_robot_plugin.conveyor_hook_throw_duration", 0.0);
  m_conveyor_hook_throw_fraction = node_ptr_->get_parameter("tank_robot_plugin.conveyor_hook_throw_fraction").as_double();
  m_conveyor_hook_throw_duration = node_ptr_->get_parameter("tank_robot_plugin.conveyor_hook_throw_duration").as_double();

  m_color_map =
  {
    {"red", 1},
    {"blue", 2},
    {"unknown", 0}};

  m_ring_found = false;
  m_ring_color = m_color_map["unknown"];

  node_ptr_->declare_parameter("tank_robot_plugin.scissor_max_extension", 0.0);
  node_ptr_->declare_parameter("tank_robot_plugin.scissor_reset_extension", 0.0);
  m_scissor_max_extension = node_ptr_->get_parameter("tank_robot_plugin.scissor_max_extension").as_double();
  m_scissor_reset_extension = node_ptr_->get_parameter("tank_robot_plugin.scissor_reset_extension").as_double();
}

void AlphaJerryPlugin::initTankModel()
{
  std::cout << "[AlphaJerryPlugin::initTankModel]" << std::endl;
  node_ptr_->declare_parameter("tank_robot_plugin.drive_motor_ticks_per_rotation", 0.0);
  node_ptr_->declare_parameter("tank_robot_plugin.drive_gear_ratio", 0.0);
  node_ptr_->declare_parameter("tank_robot_plugin.drive_wheel_rad_in", 0.0);
  node_ptr_->declare_parameter("tank_robot_plugin.wheel_base_inches", 0.0);

  double motor_ticks_per_rotation = node_ptr_->get_parameter("tank_robot_plugin.drive_motor_ticks_per_rotation").as_double();
  double drive_gear_ratio = node_ptr_->get_parameter("tank_robot_plugin.drive_gear_ratio").as_double();
  double wheel_rad_in = node_ptr_->get_parameter("tank_robot_plugin.drive_wheel_rad_in").as_double();
  double wheel_base_inches = node_ptr_->get_parameter("tank_robot_plugin.wheel_base_inches").as_double();

  TankConfig tank_model_config;
  tank_model_config.motor_list_left = m_left_drive_motor_names;
  tank_model_config.motor_list_right = m_right_drive_motor_names;
  tank_model_config.wheel_radius = wheel_rad_in;   // in
  tank_model_config.wheel_gear_ratio = 1.0 / drive_gear_ratio;
  tank_model_config.wheel_dist = wheel_base_inches / 2.0;   // in

  m_tank_model_ptr = std::make_shared<TankModel>(node_ptr_, rhi_ptr_, tank_model_config);
  m_odom_ptr = std::make_shared<TankOdometry>(motor_ticks_per_rotation * drive_gear_ratio, wheel_rad_in * INCHES_TO_METERS, wheel_base_inches * INCHES_TO_METERS);
  m_odom_ptr->resetPose();

  node_ptr_->declare_parameter("tank_robot_plugin.search_radius", -1.0);
  node_ptr_->declare_parameter("tank_robot_plugin.move_to_pose_kp_xy", 0.5);
  node_ptr_->declare_parameter("tank_robot_plugin.move_to_pose_kd_xy", 0.5);
  node_ptr_->declare_parameter("tank_robot_plugin.move_to_pose_kp_theta", 0.5);
  node_ptr_->declare_parameter("tank_robot_plugin.move_to_pose_kd_theta", 0.5);
  node_ptr_->declare_parameter("tank_robot_plugin.move_to_pose_ki_theta", 0.5);
  node_ptr_->declare_parameter("tank_robot_plugin.max_speed_linear", 0.5);
  node_ptr_->declare_parameter("tank_robot_plugin.max_speed_angular", 0.5);
  node_ptr_->declare_parameter("tank_robot_plugin.move_to_pose_kp_xy_fine", 0.5);
  node_ptr_->declare_parameter("tank_robot_plugin.move_to_pose_kd_xy_fine", 0.5);
  node_ptr_->declare_parameter("tank_robot_plugin.move_to_pose_kp_theta_fine", 0.5);
  node_ptr_->declare_parameter("tank_robot_plugin.move_to_pose_kd_theta_fine", 0.5);
  node_ptr_->declare_parameter("tank_robot_plugin.move_to_pose_ki_theta_fine", 0.5);
  node_ptr_->declare_parameter("tank_robot_plugin.move_to_pose_integral_limit", 0.5);
  m_search_radius = node_ptr_->get_parameter("tank_robot_plugin.search_radius").as_double();
  float kp_xy = node_ptr_->get_parameter("tank_robot_plugin.move_to_pose_kp_xy").as_double();
  float kd_xy = node_ptr_->get_parameter("tank_robot_plugin.move_to_pose_kd_xy").as_double();
  float kp_theta = node_ptr_->get_parameter("tank_robot_plugin.move_to_pose_kp_theta").as_double();
  float kd_theta = node_ptr_->get_parameter("tank_robot_plugin.move_to_pose_kd_theta").as_double();
  float ki_theta = node_ptr_->get_parameter("tank_robot_plugin.move_to_pose_kd_theta").as_double();
  m_max_speed_linear = node_ptr_->get_parameter("tank_robot_plugin.max_speed_linear").as_double();
  m_max_speed_angular = node_ptr_->get_parameter("tank_robot_plugin.max_speed_angular").as_double();
  float kp_xy_fine = node_ptr_->get_parameter("tank_robot_plugin.move_to_pose_kp_xy_fine").as_double();
  float kd_xy_fine = node_ptr_->get_parameter("tank_robot_plugin.move_to_pose_kd_xy_fine").as_double();
  float kp_theta_fine = node_ptr_->get_parameter("tank_robot_plugin.move_to_pose_kp_theta_fine").as_double();
  float kd_theta_fine = node_ptr_->get_parameter("tank_robot_plugin.move_to_pose_kd_theta_fine").as_double();
  float ki_theta_fine = node_ptr_->get_parameter("tank_robot_plugin.move_to_pose_ki_theta_fine").as_double();
  float integral_limit = node_ptr_->get_parameter("tank_robot_plugin.move_to_pose_integral_limit").as_double();

  m_boomerang = std::make_shared<Boomerang>();
  m_pd_control = std::make_shared<PDControl>(kp_xy, kd_xy, kp_theta, kd_theta, ki_theta, integral_limit);
  m_pd_control_threshold = std::make_shared<PDControl>(kp_xy_fine, kd_xy_fine, kp_theta_fine, kd_theta_fine, ki_theta_fine, integral_limit);
}

void AlphaJerryPlugin::initAutonomy()
{
  std::cout << "[AlphaJerryPlugin::initAutonomy]" << std::endl;
  node_ptr_->declare_parameter<std::string>("bt_path");
  std::string bt_path = node_ptr_->get_parameter("bt_path").as_string();

  node_ptr_->declare_parameter<std::string>("config_path");
  std::string config_path = node_ptr_->get_parameter("config_path").as_string();

  bt_ = std::make_shared<TankTree>(bt_path);
  bt_->set_variable("rhi_ptr", rhi_ptr_);
  bt_->set_variable("tank_model_ptr", m_tank_model_ptr);
  bt_->set_variable("node_ptr", node_ptr_);
  bt_->set_variable("pd_control_ptr", m_pd_control);
  bt_->set_variable("pd_control_threshold_ptr", m_pd_control_threshold);
  bt_->set_variable("trajectory_viz_pub", m_trajectory_viz_pub);
  bt_->set_variable("digital_io_port_map", digital_io_port_map);
  resetBT();
}

/////////////////////
/// State Machine ///
/////////////////////
void AlphaJerryPlugin::onNewSensorData()
{
  static bool first_loop = true;
  if (first_loop) {
    playMusic("hello_there");
    resetWorldPose();
    first_loop = false;
  }

  // Clear current limits at start of loop
  m_loop_current_limits.clear();

  updateConveyorPositionSensing();
  updateAndPublishOdometry();
  publishTrajectoryVisualization();
}

void AlphaJerryPlugin::updateConveyorPositionSensing()
{
  m_conveyor_position_abs = rhi_ptr_->getMotorPosition("conveyor_motor_bottom");
  m_conveyor_position_rel = std::fmod(m_conveyor_position_abs, m_conveyor_ticks_per_loop);
  m_conveyor_position_rel += (m_conveyor_position_rel < 0.0) ? m_conveyor_ticks_per_loop : 0.0;
  m_hook_fraction = std::fmod(m_conveyor_position_rel, m_conveyor_ticks_per_hook) / m_conveyor_ticks_per_hook;
}

void AlphaJerryPlugin::disabled()
{
}

void AlphaJerryPlugin::autonomous(double current_time)
{
  if (m_is_first_auton_loop) {
    m_is_first_auton_loop = false;
    playTTS("starting autonomous");
    m_odom_ptr->resetPose();
    resetWorldPose();

    bt_->set_variable<bool>("clamp_closed", false);
    bt_->set_variable<bool>("bite_closed", false);
    bt_->set_variable<bool>("goal_rush_l_down", false);
    bt_->set_variable<bool>("goal_rush_r_down", false);
    bt_->set_variable<bool>("climb_extended", false);
  }

  bt_->set_variable("auton_time_elapsed", current_time);
  bt_->set_variable("mirrored", m_mirrored);

  try {
    bt_->tick_tree();
  } catch (std::exception & e) {
    std::cout << "Error tick_tree: " << e.what() << std::endl;
  }

  // Get best state estimate
  // auto curr_pose = m_tank_model_ptr->getWorldPose();
  auto curr_twist = m_tank_model_ptr->getWorldTwist();

  publishCurrentTwist(curr_twist);
  // publishDesiredTwist(m_desired_twist);

  if (bt_->get_variable("desired_pose", m_desired_pose)) {
    publishDesiredPose(m_desired_pose);
  }

  bool ring_detector_active = false;
  bool want_red = m_color_target_red;
  // bool want_red = false;
  bool store_ring = false;
  if (bt_->get_variable("ring_detector_active", ring_detector_active)
    // && bt_->get_variable("want_red", want_red)
    && bt_->get_variable("store_ring", store_ring))
  {
    ringDetector(ring_detector_active, current_time, want_red, store_ring);
  }

  // Update Pneumatics
  m_bite_closed = (bt_->get_variable<int>("bite_closed"));
  rhi_ptr_->setDigitalOut(digital_io_port_map["climb"], (bt_->get_variable<int>("climb_extended")));
  rhi_ptr_->setDigitalOut(digital_io_port_map["clamp"], (bt_->get_variable<int>("clamp_closed")));
  rhi_ptr_->setDigitalOut(digital_io_port_map["goal_rush_l"], (bt_->get_variable<int>("goal_rush_l_down")));
  rhi_ptr_->setDigitalOut(digital_io_port_map["goal_rush_r"], (bt_->get_variable<int>("goal_rush_r_down")));
  rhi_ptr_->setDigitalOut(digital_io_port_map["bite"], m_bite_closed);

  bool ground_intake_active = false;
  if (bt_->get_variable("ground_intake_active", ground_intake_active) && !ring_detector_active) {
    updateIntake(ground_intake_active, false, false, false, current_time);
  }

  // Publish Twist Command
  geometry_msgs::msg::Twist msg{};
  bt_->get_variable("fwd_cmd", msg.linear.x);
  bt_->get_variable("turn_cmd", msg.angular.z);
  m_base_twist_cmd_pub->publish(msg);
}

void AlphaJerryPlugin::resetBT()
{
  try {
    std::cout << "Initializing Behavior Tree" << std::endl;
    bt_->init_tree();
  } catch (std::exception & e) {
    std::cout << "Error init_tree: " << e.what() << std::endl;
  }
  std::cout << "ResetBT Complete!" << std::endl;
}

void AlphaJerryPlugin::teleop(double current_time)
{
  auto joy_data = rhi_ptr_->getMainJoystickData();
  bool shift_r = joy_data->btn_b;
  bool shift_l = joy_data->btn_d;

  // Shutdown Request
  if (joy_data->btn_a && joy_data->btn_b && joy_data->btn_x && joy_data->btn_y &&
    joy_data->btn_u && joy_data->btn_l && joy_data->btn_d && joy_data->btn_r)
  {
    std::cout << "SHUTDOWN" << std::endl;
    std::system("echo 1 | sudo -S shutdown now");
    return;
  }

  // Auton Request
  bool running_auton = runAutonFromDriver(joy_data, current_time);
  if (running_auton) {
    return;
  }

  updateMusic(joy_data, current_time);   // MUST RUN FIRST: pressing u takes over all right buttons and overwrites data to be false
  toggleBagRecorder(joy_data);

  updateScissor(joy_data->btn_l1, joy_data->btn_l2, shift_r);
  updateT1Climb(joy_data->btn_r1, joy_data->btn_r2, shift_r);
  updateClamp(joy_data->btn_l2, joy_data->btn_r2, shift_l);
  updateGoalRush(joy_data->btn_l1, joy_data->btn_r1, shift_l);

  // Disable intake when shifts are active

  updateIntakeFromJoystick(joy_data, shift_l, shift_r, current_time);

  updateDrivetrain(joy_data);

  // std::cout << "extension: " << m_scissor_max_extension << std::endl;
  // std::cout << "pos: " << rhi_ptr_->getMotorPosition("scissor_motor") << std::endl;

}

void AlphaJerryPlugin::ringDetector(bool active, double current_time, bool want_red, bool store_ring)
{
  static double last_input_time = 0.0;
  static double ring_found_time = 0.0;
  static double stuck_detection_time = 0.0;
  static bool running = false;
  static bool retry_mode = false;
  static double retry_start_time = 0.0;

  // Constants (adjust as needed for your specific system)
  const double STUCK_TIMEOUT = 1.5;     // Time to consider a ring stuck
  const double RETRY_DURATION = 0.8;    // How long to attempt the retry
  const double COOLDOWN_PERIOD = 0.2;   // Brief pause between retry attempts

  if (!active) {
    last_input_time = 0.0;
    ring_found_time = 0.0;
    stuck_detection_time = 0.0;
    retry_mode = false;
    retry_start_time = 0.0;
    running = false;
    return;
  }

  m_ring_found = m_color_map[m_color] != 0;
  m_ring_color = m_color_map[m_color];
  std::cout << "color: " << m_ring_color << std::endl;

  // Initial ring detection
  if (m_ring_found && !running) {
    ring_found_time = current_time;
    stuck_detection_time = current_time;
    running = true;
    retry_mode = false;
  } else if (!m_ring_found) {
    running = false;
    retry_mode = false;
  }

  // Stuck ring detection logic
  if (running && m_ring_found) {
    if (!retry_mode && (current_time - stuck_detection_time > STUCK_TIMEOUT)) {
      // Ring has been detected for too long - initiate retry sequence
      retry_mode = true;
      retry_start_time = current_time;
    } else if (current_time - ring_found_time > 1.0) {
      ring_found_time = current_time;
    }
  }

  // Handle retry cycle
  if (retry_mode) {
    double retry_elapsed = current_time - retry_start_time;

    if (retry_elapsed > RETRY_DURATION) {
      // End retry attempt and go back to normal operation
      retry_mode = false;
      stuck_detection_time = current_time;   // Reset stuck timer
    }
  }

  // Normal processing
  bool ring_prewaited = (current_time - ring_found_time > 0.3) && m_ring_found && !retry_mode;
  if (ring_prewaited) {
    last_input_time = current_time;
  }

  // Determine hook and eject status
  bool hook = false;
  if (retry_mode) {
    // During retry: alternate between hook on/off with a small cooldown period
    double retry_cycle = fmod(current_time - retry_start_time, COOLDOWN_PERIOD * 2);
    hook = (retry_cycle < COOLDOWN_PERIOD);
  } else {
    // Normal hook logic
    if (store_ring) {
      // should not score the ring, will be stored in the center of the robot
      hook = ring_prewaited;
    } else {
      // If not storing, keep hooks moving for an extra period of time to ensure scoring
      hook = ring_prewaited || (current_time - last_input_time < 0.5);
    }
  }

  bool ejecting = false;
  static double last_eject_time = 0.0;
  bool eject = false;
  if (!retry_mode) { // Don't eject during retry attempts
    if (want_red) {
      eject = (m_ring_color == m_color_map["blue"]);   //&& ring_prewaited;
    } else {
      eject = (m_ring_color == m_color_map["red"]);   //&& ring_prewaited;
    }
  }

  if (eject) {
    last_eject_time = current_time;
  }
  if (current_time - last_eject_time < 0.0) {
    last_eject_time = 0.0;
  }
  if (current_time - last_eject_time < 1.0) {
    ejecting = true;
  }
  if (ejecting) {
    hook = false;
  }
  // std::cout << "ejecting: " << ejecting << std::endl;
  // std::cout << "hook: " << hook << std::endl;
  // std::cout << "retry_mode: " << retry_mode << std::endl;

  // Call motor control with determined states
  updateIntake(true, hook, ejecting, !hook && retry_mode, current_time);
}

bool AlphaJerryPlugin::runAutonFromDriver(JoyPtr joy_data, double current_time)
{
  static bool auton_button_pressed = false;
  if (joy_data->btn_u && joy_data->btn_l) {
    if (!auton_button_pressed) {
      auton_button_pressed = true;
      m_is_first_auton_loop = true;
      m_auton_start_time = current_time;
      resetBT();
    }
    autonomous(current_time - m_auton_start_time);

    return true;
  }
  auton_button_pressed = false;
  return false;
}

void AlphaJerryPlugin::toggleBagRecorder(JoyPtr joy_data)
{
  return;
  if (joy_data->btn_y && joy_data->btn_x && !m_recording_btn_pressed) {
    m_recording_btn_pressed = true;
    if (!m_recording) {
      std::cout << "[AlphaJerryPlugin::toggleBagRecorder] Starting Bag Recorder!" << std::endl;
      auto req = std::make_shared<ghost_msgs::srv::StartRecorder::Request>();
      m_start_recorder_client->async_send_request(req);
    } else {
      std::cout << "[AlphaJerryPlugin::toggleBagRecorder] Stopping Bag Recorder!" << std::endl;
      auto req = std::make_shared<ghost_msgs::srv::StopRecorder::Request>();
      m_stop_recorder_client->async_send_request(req);
    }
    m_recording = !m_recording;
  } else if (!(joy_data->btn_y && joy_data->btn_x)) {
    m_recording_btn_pressed = false;
  }
}

void AlphaJerryPlugin::updateT1Climb(bool up, bool down, bool enabled)
{
  if (enabled) {
    if (up) {
      rhi_ptr_->setDigitalOut(digital_io_port_map["climb"], true);
    } else if (down) {
      rhi_ptr_->setDigitalOut(digital_io_port_map["climb"], false);
    }
  }
}


void AlphaJerryPlugin::updateScissor(bool up, bool down, bool enabled)
{
  if (enabled) {
    if (up) {
      rhi_ptr_->setMotorCurrentLimitMilliAmps("scissor_motor", 2500);
      rhi_ptr_->setMotorVoltageCommandPercent("scissor_motor", 1.0);
    } else if (down) {
      rhi_ptr_->setMotorCurrentLimitMilliAmps("scissor_motor", 2500);
      rhi_ptr_->setMotorVoltageCommandPercent("scissor_motor", -1.0);
    } else {
      rhi_ptr_->setMotorCurrentLimitMilliAmps("scissor_motor", 0);
      rhi_ptr_->setMotorVoltageCommandPercent("scissor_motor", 0.0);
    }
  } else {
    rhi_ptr_->setMotorCurrentLimitMilliAmps("scissor_motor", 0);
    rhi_ptr_->setMotorVoltageCommandPercent("scissor_motor", 0.0);
  }
}

void AlphaJerryPlugin::updateIntake(bool R2, bool R1, bool L1, bool R, double current_time)
{
  static bool first_r2 = false;
  static bool first_r2_started = false;
  // Manual Ground Pickup control
  double ground_pickup_power = 0;
  int32_t ground_pickup_current = 0;
  if (R2) {
    ground_pickup_power = 1.0;
    ground_pickup_current = 2500;
    if (first_r2) {
      first_r2_started = true;
    }
  } else if (R) {
    ground_pickup_power = -1.0;
    ground_pickup_current = 2500;
  } else {
    ground_pickup_power = 0.0;
    ground_pickup_current = 0;
    if (first_r2_started) {
      first_r2 = false;
    }
  }
  // Conveyor control
  // We assume any manual conveyor control misaligns the hooks
  double conveyor_power = 0;
  int32_t conveyor_current = 0;
  if (R1) {
    conveyor_power = 1.0;
    conveyor_current = 2500;
    m_conveyor_hook_is_aligned = false;
  } else if (L1 && !R2) {
    conveyor_power = -1.0;
    conveyor_current = 2500;
    m_conveyor_hook_is_aligned = false;
  } else {
    conveyor_power = 0.0;
    conveyor_current = 0;
  }

  // Align Conveyor when Ground Pickup is active and there are no commands going to manual Conveyor control
  if (R2 && !R1 && !m_conveyor_hook_is_ejecting) {
    m_conveyor_hook_is_aligned = !(m_hook_fraction < m_conveyor_hook_align_threshold);
    if (m_conveyor_hook_is_aligned || first_r2) {
      m_conveyor_last_aligned_position = m_conveyor_position_abs;
      conveyor_power = 0;
      conveyor_current = 0;
    } else {
      conveyor_power = m_conveyor_hook_align_power;
      conveyor_current = 1000;
    }
  }
  // std::cout << "m_conveyor_hook_is_aligned: " << m_conveyor_hook_is_aligned << std::endl;
  // std::cout << "m_hook_fraction: " << m_hook_fraction << std::endl;
  // std::cout << "conveyor_power: " << conveyor_power << std::endl;

  // Transition to ejection mode
  static double ejecting_start_time = 0.0;
  if (R2 && L1 && m_conveyor_hook_is_aligned && !m_conveyor_hook_is_ejecting) {
    ejecting_start_time = current_time;
    m_conveyor_hook_is_ejecting = true;
    m_conveyor_hook_is_aligned = false;
  }

  // Max timeout on ejection
  if (m_conveyor_hook_is_ejecting && current_time > ejecting_start_time + 1.5) {
    m_conveyor_hook_is_ejecting = false;
  }

  // During ejection, run until we reach throw position, then transition to throw
  if (m_conveyor_hook_is_ejecting) {
    double throw_dist_rel = m_conveyor_hook_throw_fraction * m_conveyor_ticks_per_hook;
    if ((m_conveyor_position_abs - m_conveyor_last_aligned_position) > throw_dist_rel && !m_conveyor_is_throwing) {
      m_conveyor_is_throwing = true;
      m_conveyor_hook_is_ejecting = false;
      m_conveyor_throw_start_time = current_time;
    }
    conveyor_power = 1.0;
    conveyor_current = 2500;
  }

  // Throw reverses for set duration and then zeros conveyor and returns to manual control
  if (m_conveyor_is_throwing) {
    conveyor_power = -0.1;
    conveyor_current = 500;
    if (current_time > m_conveyor_throw_start_time + m_conveyor_hook_throw_duration) {
      m_conveyor_is_throwing = false;
      conveyor_power = 0.0;
      conveyor_current = 0;
    }
  }

  if (!m_bite_closed) {
    ground_pickup_power = 0.0;
    ground_pickup_current = 0.0;
  }

  rhi_ptr_->setMotorVoltageCommandPercent("ground_pickup_motor", ground_pickup_power);
  rhi_ptr_->setMotorCurrentLimitMilliAmps("ground_pickup_motor", ground_pickup_current);

  rhi_ptr_->setMotorVoltageCommandPercent("conveyor_motor_top", conveyor_power);
  rhi_ptr_->setMotorCurrentLimitMilliAmps("conveyor_motor_top", conveyor_current);
  rhi_ptr_->setMotorVoltageCommandPercent("conveyor_motor_bottom", conveyor_power);
  rhi_ptr_->setMotorCurrentLimitMilliAmps("conveyor_motor_bottom", conveyor_current);

  m_loop_current_limits.push_back(ground_pickup_current);
  m_loop_current_limits.push_back(conveyor_current * 2.0);
}

void AlphaJerryPlugin::updateIntakeFromJoystick(JoyPtr joy_data, bool shift_l, bool shift_r, double current_time)
{
  if (shift_r || shift_l) {
    updateIntake(false, false, false, false, current_time); // Default mode
  } else {
    if (joy_data->btn_l1 && joy_data->btn_r1) {
      toggleBite(true);
    } else {
      toggleBite(false);
    }
    updateIntake(joy_data->btn_r2, joy_data->btn_r1, joy_data->btn_l1, joy_data->btn_l2, current_time); // Default mode
  }
}

void AlphaJerryPlugin::toggleBite(bool signal)
{
  static bool bite_btn_pressed = false;
  if (signal && !bite_btn_pressed) {
    bite_btn_pressed = true;
    m_bite_closed = !m_bite_closed;
  } else if (!signal) {
    bite_btn_pressed = false;
  }
  rhi_ptr_->setDigitalOut(digital_io_port_map["bite"], m_bite_closed);
}

void AlphaJerryPlugin::updateClamp(bool close, bool open, bool shift_l)
{
  static bool clamp_btn_pressed = false;

  if (shift_l) {
    // Close on L2 rising edge
    if (close) {
      m_clamp_closed = true;
    }

    // Open on R2 rising edge
    if (open) {
      m_clamp_closed = false;
    }

    rhi_ptr_->setDigitalOut(digital_io_port_map["clamp"], m_clamp_closed);
  }
}

// pressing u takes over all right buttons
void AlphaJerryPlugin::updateMusic(JoyPtr joy_data, double current_time)
{
  static double btn_pressed = 0;
  if (true && btn_pressed < (current_time - 5)) {
    static double btn_pressed = 0;
    if (joy_data->btn_u) {
      if (btn_pressed < (current_time - 2)) {
        if (joy_data->btn_x) {
          btn_pressed = current_time;
          playMusic("rand");
        } else if (joy_data->btn_y) {
          btn_pressed = current_time;
          playMusic("seinfeld");
        } else if (joy_data->btn_a) {
          btn_pressed = current_time;
          playMusic("awesome");
        } else if (joy_data->btn_b) {
          btn_pressed = current_time;
          playMusic("objection");
        } else if (joy_data->btn_r1) {
          btn_pressed = current_time;
          playMusic("emotional");
        } else if (joy_data->btn_r2) {
          btn_pressed = current_time;
          playMusic("feminominon");
        }
      }
      joy_data->btn_x = joy_data->btn_y = joy_data->btn_a = joy_data->btn_b = joy_data->btn_r1 = joy_data->btn_r2 = false;
    }
  }
}

void AlphaJerryPlugin::updateGoalRush(bool left_rush, bool right_rush, bool enabled)
{
  if (enabled) {
    rhi_ptr_->setDigitalOut(digital_io_port_map["goal_rush_l"], left_rush);
    rhi_ptr_->setDigitalOut(digital_io_port_map["goal_rush_r"], right_rush);
  } else {
    rhi_ptr_->setDigitalOut(digital_io_port_map["goal_rush_l"], false);
    rhi_ptr_->setDigitalOut(digital_io_port_map["goal_rush_r"], false);
  }
}

void AlphaJerryPlugin::updateDrivetrain(JoyPtr joy_data)
{
  m_tank_model_ptr->driveCommandJoystick(joy_data->left_y, -joy_data->right_x, 0.05);

  int32_t drive_curr_lim = static_cast<int32_t>(ghost_control::v5_current_limiting::getRemainingCurrentDistributed(m_loop_current_limits, m_num_motors));
  if (std::fabs(joy_data->left_y / 127.0) < 0.05 && std::fabs(-joy_data->right_x / 127.0) < 0.05) {
    drive_curr_lim = 0;
  }

  for (const auto & name : m_all_drive_motor_names) {
    rhi_ptr_->setMotorCurrentLimitMilliAmps(name, drive_curr_lim);
  }
}

void AlphaJerryPlugin::worldOdometryUpdateCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!m_use_backup_estimator) {
    double theta = ghost_util::quaternionToYawRad(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z);
    m_tank_model_ptr->setWorldPose(msg->pose.pose.position.x, msg->pose.pose.position.y, theta);
    m_tank_model_ptr->setWorldTwist(
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.angular.z);
  }
}

void AlphaJerryPlugin::worldOdometryUpdateCallbackBackup(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (m_use_backup_estimator) {
    double theta = ghost_util::quaternionToYawRad(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z);
    m_tank_model_ptr->setWorldPose(msg->pose.pose.position.x, msg->pose.pose.position.y, theta);
    m_tank_model_ptr->setWorldTwist(
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.angular.z);
  }
}

void AlphaJerryPlugin::publishBaseTwist()
{
  // geometry_msgs::msg::Twist msg{};
  // auto base_vel_cmd = m_tank_model_ptr->getBaseVelocityCommand();
  // msg.linear.x = base_vel_cmd.x();
  // msg.linear.y = base_vel_cmd.y();
  // msg.angular.z = base_vel_cmd.z();
  // m_base_twist_cmd_pub->publish(msg);
}

void AlphaJerryPlugin::updateAndPublishOdometry()
{
  std::vector<long> r_pos;
  std::vector<long> l_pos;

  for (const auto & name : m_right_drive_motor_names) {
    r_pos.push_back(rhi_ptr_->getMotorPosition(name));
  }
  for (const auto & name : m_left_drive_motor_names) {
    l_pos.push_back(rhi_ptr_->getMotorPosition(name));
  }

  m_odom_ptr->update(
    Eigen::Map<Eigen::VectorX<long>>(l_pos.data(), l_pos.size()),
    Eigen::Map<Eigen::VectorX<long>>(r_pos.data(), r_pos.size()));

  m_curr_odom_pose = m_odom_ptr->getPose();

  nav_msgs::msg::Odometry msg{};
  msg.header.frame_id = "odom";
  msg.header.stamp = node_ptr_->get_clock()->now();
  msg.child_frame_id = "base_link";

  msg.pose.pose.position.x = m_curr_odom_pose.x();
  msg.pose.pose.position.y = m_curr_odom_pose.y();
  msg.pose.pose.position.z = 0.0;
  // if (!(m_curr_odom_pose.z() < 1 && m_curr_odom_pose.z() > -1)){
  //   printf("ROBOT MOVED ANGLE IS %f\n", m_curr_odom_pose.z());
  // }
  ghost_util::yawToQuaternionRad(
    m_curr_odom_pose.z(),
    msg.pose.pose.orientation.w,
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z);

  // Calculate differences for odometry
  auto odom_diff_x = std::fabs(m_curr_odom_pose.x() - m_last_odom_pose.x());
  auto odom_diff_y = std::fabs(m_curr_odom_pose.y() - m_last_odom_pose.y());
  auto odom_diff_theta =
    std::fabs(ghost_util::SmallestAngleDistRad(m_curr_odom_pose.z(), m_last_odom_pose.z()));

  // Holonomic Motion Model
  Eigen::Vector3d diff_std = Eigen::Vector3d(
    m_k1 * odom_diff_x + m_k2 * odom_diff_y + m_k3 * odom_diff_theta,
    m_k4 * odom_diff_x + m_k5 * odom_diff_y + m_k6 * odom_diff_theta,
    m_k7 * odom_diff_x + m_k8 * odom_diff_y + m_k9 * odom_diff_theta);

  m_curr_odom_std += diff_std;
  m_curr_odom_cov = m_curr_odom_std.array().square();

  // covariance is row major form
  std::array<double, 36> pose_covariance{
    m_curr_odom_cov.x(), 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, m_curr_odom_cov.y(), 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, m_curr_odom_cov.z()};

  msg.pose.covariance = pose_covariance;

  // INFO: Publishing twist is unimplemented, higher layers do without it
  // Leave this in since it may be a useful reference in the future

  // auto current_velocity = m_tank_model_ptr->getBaseVelocityCurrent();

  // msg.twist.twist.linear.x = current_velocity.x();
  // msg.twist.twist.linear.y = current_velocity.y();
  // msg.twist.twist.linear.z = 0.0;
  // msg.twist.twist.angular.x = 0.0;
  // msg.twist.twist.angular.y = 0.0;
  // msg.twist.twist.angular.z = current_velocity.z();

  // double sigma_x_vel =
  //   m_k1 * current_velocity.x() +
  //   m_k2 * current_velocity.y() +
  //   m_k3 * abs(current_velocity.z());
  // double sigma_y_vel =
  //   m_k4 * current_velocity.x() +
  //   m_k5 * current_velocity.y() +
  //   m_k6 * abs(current_velocity.z());
  //// Get noisy angle
  // double sigma_tht_vel =
  //   m_k7 * current_velocity.x() +
  //   m_k8 * current_velocity.y() +
  //   m_k9 * abs(current_velocity.z());

  // std::array<double, 36> vel_covariance{
  //   sigma_x_vel * sigma_x_vel, 0.0, 0.0, 0.0, 0.0, 0.0,
  //   0.0, sigma_y_vel * sigma_y_vel, 0.0, 0.0, 0.0, 0.0,
  //   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  //   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  //   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  //   0.0, 0.0, 0.0, 0.0, 0.0, sigma_tht_vel * sigma_tht_vel};

  // msg.twist.covariance = vel_covariance;

  m_odom_pub->publish(msg);

  m_last_odom_pose = m_curr_odom_pose;
}

void AlphaJerryPlugin::resetWorldPose()
{
  // Copy yaml vectors to array
  std::array<double, m_cov_n> m_initial_estimate_covariance_arr;
  for (int i = 0; i < m_initial_estimate_covariance.size(); i++) {
    m_initial_estimate_covariance_arr[i] = m_initial_estimate_covariance[i];
  }

  geometry_msgs::msg::Quaternion quat{};
  geometry_msgs::msg::PoseWithCovarianceStamped new_pose{};
  new_pose.header.frame_id = "map";
  new_pose.header.stamp = node_ptr_->get_clock()->now();

  if (m_mirrored) {
    ghost_util::yawToQuaternionRad(m_reset_pose_mirrored[5], quat.w, quat.x, quat.y, quat.z);
    new_pose.pose.pose.position.x = m_reset_pose_mirrored[0];
    new_pose.pose.pose.position.y = m_reset_pose_mirrored[1];
  } else {
    ghost_util::yawToQuaternionRad(m_reset_pose[5], quat.w, quat.x, quat.y, quat.z);
    new_pose.pose.pose.position.x = m_reset_pose[0];
    new_pose.pose.pose.position.y = m_reset_pose[1];
  }

  new_pose.pose.pose.orientation = quat;
  new_pose.pose.covariance = m_initial_estimate_covariance_arr;

  // Publish to Particle Filter
  m_reset_pf_pub->publish(new_pose);
  if (m_mirrored) {
    std::cout << "Done reset: mirrored" << std::endl;
  } else {
    std::cout << "Done reset: regular" << std::endl;
  }
}

void AlphaJerryPlugin::publishCurrentTwist(
  Eigen::Vector3d twist)
{
  geometry_msgs::msg::Twist msg{};
  msg.linear.x = twist.x();
  msg.linear.y = twist.y();
  msg.angular.z = twist.z();
  m_cur_twist_pub->publish(msg);
}

void AlphaJerryPlugin::publishDesiredTwist(
  Eigen::Vector3d twist)
{
  geometry_msgs::msg::Twist msg{};
  msg.linear.x = twist.x();
  msg.linear.y = twist.y();
  msg.angular.z = twist.z();
  m_des_twist_pub->publish(msg);
}

void AlphaJerryPlugin::publishDesiredPose(Eigen::Vector3d pose)
{
  geometry_msgs::msg::Pose msg{};
  msg.position.x = pose.x();
  msg.position.y = pose.y();
  ghost_util::yawToQuaternionRad(
    pose.z(),
    msg.orientation.w,
    msg.orientation.x,
    msg.orientation.y,
    msg.orientation.z);
  m_des_pos_pub->publish(msg);
}

void AlphaJerryPlugin::publishErrorPose(Eigen::Vector3d pose)
{
  geometry_msgs::msg::Pose msg{};
  msg.position.x = pose.x();
  msg.position.y = pose.y();
  ghost_util::yawToQuaternionRad(
    pose.z(),
    msg.orientation.w,
    msg.orientation.x,
    msg.orientation.y,
    msg.orientation.z);
  m_err_pos_pub->publish(msg);
}

void AlphaJerryPlugin::publishTrajectoryVisualization()
{
  if (!robot_trajectory_ptr_->isNotEmpty()) {
    return;
  }
  visualization_msgs::msg::MarkerArray msg{};

  visualization_msgs::msg::Marker search_radius_marker{};
  search_radius_marker.header.frame_id = "base_link";
  search_radius_marker.header.stamp = node_ptr_->get_clock()->now();
  search_radius_marker.id = 1;
  search_radius_marker.type = 3;   // cylinder type
  search_radius_marker.action = 0;
  search_radius_marker.scale.x = 2 * m_search_radius;
  search_radius_marker.scale.y = 2 * m_search_radius;
  search_radius_marker.scale.z = 0.01;
  search_radius_marker.color.b = 1.0;
  search_radius_marker.color.a = 0.3;

  visualization_msgs::msg::Marker carrot{};
  carrot.header.frame_id = "map";
  carrot.header.stamp = node_ptr_->get_clock()->now();
  carrot.id = 2;
  carrot.type = 4;   // line type
  carrot.action = 0;
  carrot.scale.x = 0.01;
  carrot.scale.y = 1.0;
  carrot.scale.z = 1.0;
  carrot.color.g = 1.0;
  carrot.color.a = 0.5;
  geometry_msgs::msg::Point p_robot;
  p_robot.x = m_tank_model_ptr->getWorldPose().x();
  p_robot.y = m_tank_model_ptr->getWorldPose().y();
  p_robot.z = 0.0;
  geometry_msgs::msg::Point p_carrot;
  p_carrot.set__x(m_desired_pose.x());
  p_carrot.set__y(m_desired_pose.y());
  p_carrot.z = 0.0;
  carrot.points.push_back(p_robot);
  carrot.points.push_back(p_carrot);

  visualization_msgs::msg::Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = node_ptr_->get_clock()->now();
  marker.id = 0;
  marker.type = 8;   // points type
  marker.action = 0;
  marker.scale.x = 0.025;
  marker.scale.y = 0.025;
  marker.scale.z = 0.1;
  marker.color.r = 1.0;
  marker.color.a = 1.0;

  for (int i = 0; i < robot_trajectory_ptr_->x_trajectory.position_vector.size(); i += 5) {
    geometry_msgs::msg::Point p;
    p.x = robot_trajectory_ptr_->x_trajectory.position_vector[i];
    p.y = robot_trajectory_ptr_->y_trajectory.position_vector[i];
    p.z = 0.0;
    marker.points.push_back(p);
  }
  msg.markers.push_back(search_radius_marker);
  msg.markers.push_back(carrot);
  msg.markers.push_back(marker);
  m_trajectory_viz_pub->publish(msg);
}
void AlphaJerryPlugin::playMusic(std::string musicFileName)
{
  auto message = std_msgs::msg::String();
  message.data = musicFileName;
  m_music_pub->publish(message);
}

void AlphaJerryPlugin::playTTS(std::string textString)
{
  auto message = std_msgs::msg::String();
  message.data = textString;
  m_tts_pub->publish(message);
}

void AlphaJerryPlugin::colorTargetButtonCallback(const std_msgs::msg::Int64::SharedPtr msg)
{
  if (m_color_target_red != msg->data) {
    RCLCPP_INFO(node_ptr_->get_logger(), "color_target_red state changed: %ld", msg->data);
  }
  if (msg->data == 1) {
    m_color_target_red = true;
  } else if (msg->data == 0) {
    m_color_target_red = false;
  } else {
    RCLCPP_WARN(node_ptr_->get_logger(), "Received unknown button command: %ld", msg->data);
  }
}

void AlphaJerryPlugin::mirroredButtonCallback(const std_msgs::msg::Int64::SharedPtr msg)
{
  if (m_mirrored != msg->data) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Mirrored state changed: %ld", msg->data);
    if (msg->data == 1) {
      m_mirrored = true;
    } else if (msg->data == 0) {
      m_mirrored = false;
    } else {
      RCLCPP_WARN(node_ptr_->get_logger(), "Received unknown button command: %ld", msg->data);
    }
    resetWorldPose();
  }
}

} // namespace ghost_tank

PLUGINLIB_EXPORT_CLASS(ghost_tank::AlphaJerryPlugin, ghost_ros_interfaces::V5RobotBase)
