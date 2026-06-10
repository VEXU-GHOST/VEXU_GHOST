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
#include <ghost_tank/tank_robot_plugin.hpp>
#include <ghost_util/angle_util.hpp>
#include <ghost_util/math_util.hpp>
#include <ghost_util/unit_conversion_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <ghost_util/read_path.hpp>
#include <ghost_control/models/v5_current_limiting.hpp>

using ghost_planners::RobotTrajectory;
using ghost_ros_interfaces::msg_helpers::fromROSMsg;
using std::placeholders::_1;
using namespace std::chrono_literals;

using ghost_control::PIDController;
using ghost_control::PIDConfig;

using ghost_v5_interfaces::devices::JoystickDeviceData;

using ghost_util::INCHES_TO_METERS;

using JoyPtr = std::shared_ptr<ghost_v5_interfaces::devices::JoystickDeviceData>;

namespace ghost_tank
{

TankRobotPlugin::TankRobotPlugin()
{
  populateMotorNames();
  populateDigitalIONames();
}

void TankRobotPlugin::populateMotorNames()
{
  m_right_drive_motor_names = {
    "drive_r1",
    "drive_r2",
    "drive_r3",
    "drive_r4",
    "drive_r5",
    "drive_r6",
    "drive_r7",
    // "drive_r8",
  };
  m_left_drive_motor_names = {
    "drive_l1",
    "drive_l2",
    "drive_l3",
    "drive_l4",
    "drive_l5",
    "drive_l6",
    "drive_l7",
    // "drive_l8",
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

void TankRobotPlugin::populateDigitalIONames()
{
  digital_io_port_map["score_pos"] = 2;
  digital_io_port_map["color_sorter"] = 0;
  digital_io_port_map["descorer"] = 1;
  digital_io_port_map["match_loading"] = 7;
  // TODO: what port is it actually

}

//////////////////////
/// Initialization ///
//////////////////////

void TankRobotPlugin::initialize()
{
  std::cout << "TankRobotPlugin::initialize" << std::endl;
  initROSComms();
  initEstimation();
  initIntake();
  initTankModel();
  initAutonomy();
  resetWorldPose();
}

void TankRobotPlugin::initROSComms()
{
  std::cout << "[TankRobotPlugin::initROSComms]" << std::endl;
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
  m_joint_state_pub = node_ptr_->create_publisher<sensor_msgs::msg::JointState>(joint_state_topic, rclcpp::SensorDataQoS());

  node_ptr_->declare_parameter("trajectory_marker_topic", "/trajectory_markers");
  std::string trajectory_marker_topic = node_ptr_->get_parameter("trajectory_marker_topic").as_string();
  m_trajectory_viz_pub = node_ptr_->create_publisher<visualization_msgs::msg::MarkerArray>(trajectory_marker_topic, rclcpp::SensorDataQoS());

  node_ptr_->declare_parameter("tank_robot_plugin.cmd_twist_topic", "/cmd_vel");
  std::string cmd_twist_topic = node_ptr_->get_parameter("tank_robot_plugin.cmd_twist_topic").as_string();
  m_base_twist_cmd_pub = node_ptr_->create_publisher<geometry_msgs::msg::Twist>(cmd_twist_topic, rclcpp::SensorDataQoS());

  node_ptr_->declare_parameter("odom_topic", "/sensors/wheel_odom");
  std::string odom_topic = node_ptr_->get_parameter("odom_topic").as_string();
  m_odom_pub = node_ptr_->create_publisher<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::SensorDataQoS());

  // Wheel-odom watchdog: odom_ekf only initializes once it receives a wheel-odom
  // message, and the V5 brain stops sending them when it's off. A 1 Hz timer
  // republishes the last odom whenever nothing has gone out in the last second,
  // so the EKF still gets a measurement (identity until the first real one) and
  // the odom->base_link tree comes up. The normal sensor path resets the timer.
  m_last_odom_msg.header.frame_id = "odom";
  m_last_odom_msg.child_frame_id = "base_link";
  m_last_odom_msg.pose.pose.orientation.w = 1.0;
  m_last_sensor_time = std::chrono::steady_clock::now();
  m_odom_watchdog_timer = node_ptr_->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&TankRobotPlugin::odomWatchdogLoop, this));

  // Subscriptions
  node_ptr_->declare_parameter("pose_topic", "/odometry/filtered");
  std::string pose_topic = node_ptr_->get_parameter("pose_topic").as_string();
  m_robot_pose_sub = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(pose_topic, rclcpp::SensorDataQoS(), std::bind(&TankRobotPlugin::worldOdometryUpdateCallback, this, _1));

  node_ptr_->declare_parameter("backup_pose_topic", "/odom_ekf/odometry");
  std::string backup_pose_topic = node_ptr_->get_parameter("backup_pose_topic").as_string();
  m_robot_backup_pose_sub = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(backup_pose_topic, rclcpp::SensorDataQoS(), std::bind(&TankRobotPlugin::worldOdometryUpdateCallbackBackup, this, _1));

  m_robot_color = node_ptr_->create_subscription<std_msgs::msg::String>("/sensors/color_sensors/intake/color", rclcpp::SensorDataQoS(), std::bind(&TankRobotPlugin::colorCallback, this, _1));

  // Tank-Specific Publishers
  node_ptr_->declare_parameter("tank_robot_plugin.cmd_pose_topic", "/set_pose");
  std::string cmd_pose_topic = node_ptr_->get_parameter("tank_robot_plugin.cmd_pose_topic").as_string();
  m_set_pose_publisher = node_ptr_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(cmd_pose_topic, rclcpp::SensorDataQoS());

  node_ptr_->declare_parameter("tank_robot_plugin.des_twist_topic", "/des_vel");
  std::string des_twist_topic = node_ptr_->get_parameter("tank_robot_plugin.des_twist_topic").as_string();
  m_des_twist_pub = node_ptr_->create_publisher<geometry_msgs::msg::Twist>(des_twist_topic, rclcpp::SensorDataQoS());

  node_ptr_->declare_parameter("tank_robot_plugin.cur_twist_topic", "/cur_vel");
  std::string cur_twist_topic = node_ptr_->get_parameter("tank_robot_plugin.cur_twist_topic").as_string();
  m_cur_twist_pub = node_ptr_->create_publisher<geometry_msgs::msg::Twist>(cur_twist_topic, rclcpp::SensorDataQoS());

  node_ptr_->declare_parameter("tank_robot_plugin.des_pos_topic", "/des_pos");
  std::string des_pos_topic = node_ptr_->get_parameter("tank_robot_plugin.des_pos_topic").as_string();
  m_des_pos_pub = node_ptr_->create_publisher<geometry_msgs::msg::Pose>(des_pos_topic, rclcpp::SensorDataQoS());

  node_ptr_->declare_parameter("tank_robot_plugin.err_pos_topic", "/err_pos");
  std::string err_pos_topic = node_ptr_->get_parameter("tank_robot_plugin.err_pos_topic").as_string();
  m_err_pos_pub = node_ptr_->create_publisher<geometry_msgs::msg::Pose>(err_pos_topic, rclcpp::SensorDataQoS());

  node_ptr_->declare_parameter("input_imu_topic", "/sensors/imu");
  std::string input_imu_topic = node_ptr_->get_parameter("input_imu_topic").as_string();
  imu_pub = node_ptr_->create_publisher<sensor_msgs::msg::Imu>(input_imu_topic, rclcpp::SensorDataQoS());

  m_tts_pub = node_ptr_->create_publisher<std_msgs::msg::String>("/io/speaker/tts", 1);
  m_music_pub = node_ptr_->create_publisher<std_msgs::msg::String>("/io/speaker/music", 1);

  m_led_color_red_pub = node_ptr_
    ->create_publisher<std_msgs::msg::Int64>("/io/leds/color_red", 1);
  m_led_side_right_pub = node_ptr_
    ->create_publisher<std_msgs::msg::Int64>("/io/leds/side_right", 1);

  m_button_color_target_sub = node_ptr_->create_subscription<std_msgs::msg::Int64>(
    "/io/buttons/color_target", rclcpp::SensorDataQoS(),
    std::bind(&TankRobotPlugin::colorTargetButtonCallback, this, _1));
  m_button_mirrored_sub = node_ptr_->create_subscription<std_msgs::msg::Int64>(
    "/io/buttons/mirrored", rclcpp::SensorDataQoS(),
    std::bind(&TankRobotPlugin::mirroredButtonCallback, this, _1));
  m_button_reset_sub = node_ptr_->create_subscription<std_msgs::msg::Int64>(
    "/io/buttons/reset", rclcpp::SensorDataQoS(),
    std::bind(&TankRobotPlugin::resetButtonCallback, this, _1));
}

void TankRobotPlugin::initEstimation()
{
  std::cout << "[TankRobotPlugin::initEstimation]" << std::endl;

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

  node_ptr_->declare_parameter("particle_filter.init_sigma_x", m_init_sigma_x);
  node_ptr_->declare_parameter("particle_filter.init_sigma_y", m_init_sigma_y);
  node_ptr_->declare_parameter("particle_filter.init_sigma_theta", m_init_sigma_theta);
  m_init_sigma_x = node_ptr_->get_parameter("particle_filter.init_sigma_x").as_double();
  m_init_sigma_y = node_ptr_->get_parameter("particle_filter.init_sigma_y").as_double();
  m_init_sigma_theta = node_ptr_->get_parameter("particle_filter.init_sigma_theta").as_double();

  node_ptr_->declare_parameter("tank_robot_plugin.init_x_tiles", 0.0);
  node_ptr_->declare_parameter("tank_robot_plugin.init_y_tiles", 0.0);
  node_ptr_->declare_parameter("tank_robot_plugin.init_theta_degrees", 0.0);

  constexpr double tiles_to_meters = 0.6096;

  m_reset_pose_xy_m = tiles_to_meters * Eigen::Vector2d(
    node_ptr_->get_parameter("tank_robot_plugin.init_x_tiles").as_double(),
    node_ptr_->get_parameter("tank_robot_plugin.init_y_tiles").as_double());
  m_reset_pose_angle_rad = node_ptr_->get_parameter("tank_robot_plugin.init_theta_degrees").as_double() * ghost_util::DEG_TO_RAD;
}

void TankRobotPlugin::initIntake()
{
  std::cout << "[TankRobotPlugin::initIntake]" << std::endl;
  node_ptr_->declare_parameter("tank_robot_plugin.conveyor_num_links", 0.0);

  double conveyor_num_links = node_ptr_->get_parameter("tank_robot_plugin.conveyor_num_links").as_double();

  node_ptr_->declare_parameter("tank_robot_plugin.conveyor_hook_throw_fraction", 0.0);
  node_ptr_->declare_parameter("tank_robot_plugin.conveyor_hook_throw_duration", 0.0);
  m_conveyor_hook_throw_fraction = node_ptr_->get_parameter("tank_robot_plugin.conveyor_hook_throw_fraction").as_double();
  m_conveyor_hook_throw_duration = node_ptr_->get_parameter("tank_robot_plugin.conveyor_hook_throw_duration").as_double();

  m_color_map =
  {
    {"red", 1},
    {"blue", 2},
    {"unknown", 0}
  };

  m_ring_found = false;
  m_ring_color = m_color_map["unknown"];
}

PIDConfig TankRobotPlugin::loadPIDConfig(const std::string & param_prefix)
{
  PIDConfig config;

  node_ptr_->declare_parameter("tank_robot_plugin." + param_prefix + ".kp", -1.0);
  node_ptr_->declare_parameter("tank_robot_plugin." + param_prefix + ".ki", -1.0);
  node_ptr_->declare_parameter("tank_robot_plugin." + param_prefix + ".kd", -1.0);
  node_ptr_->declare_parameter("tank_robot_plugin." + param_prefix + ".integral_limit", -1.0);
  node_ptr_->declare_parameter("tank_robot_plugin." + param_prefix + ".integral_activation_bound", -1.0);

  config.kp = node_ptr_->get_parameter("tank_robot_plugin." + param_prefix + ".kp").as_double();
  config.ki = node_ptr_->get_parameter("tank_robot_plugin." + param_prefix + ".ki").as_double();
  config.kd = node_ptr_->get_parameter("tank_robot_plugin." + param_prefix + ".kd").as_double();
  config.integral_limit = node_ptr_->get_parameter("tank_robot_plugin." + param_prefix + ".integral_limit").as_double();
  config.integral_activation_bound = node_ptr_->get_parameter("tank_robot_plugin." + param_prefix + ".integral_activation_bound").as_double();

  return config;
}

void TankRobotPlugin::initTankModel()
{
  std::cout << "[TankRobotPlugin::initTankModel]" << std::endl;
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
  tank_model_config.wheel_radius_in = wheel_rad_in;
  tank_model_config.wheel_gear_ratio = 1.0 / drive_gear_ratio;
  tank_model_config.wheel_dist_in = wheel_base_inches / 2.0; //in

  m_tank_model_ptr = std::make_shared<TankModel>(node_ptr_, rhi_ptr_, tank_model_config);
  tank_trajectory_ptr_ = std::make_shared<motion_planning::Trajectory>();
  m_odom_ptr = std::make_shared<TankOdometry>(motor_ticks_per_rotation * drive_gear_ratio, wheel_rad_in * INCHES_TO_METERS, wheel_base_inches * INCHES_TO_METERS);
  m_odom_ptr->resetPose();

  // Load PID Controller Gains
  auto distance_approach_config = loadPIDConfig("distance_approach");
  m_distance_approach_controller_ptr = std::make_shared<PIDController>(distance_approach_config);

  auto steering_approach_config = loadPIDConfig("steering_approach");
  m_steering_approach_controller_ptr = std::make_shared<PIDController>(steering_approach_config);

  auto distance_settling_config = loadPIDConfig("distance_settling");
  m_distance_settling_controller_ptr = std::make_shared<PIDController>(distance_settling_config);

  auto steering_settling_config = loadPIDConfig("steering_settling");
  m_steering_settling_controller_ptr = std::make_shared<PIDController>(steering_settling_config);

  auto arc_turn_config = loadPIDConfig("arc_turn");
  m_arc_turn_controller_ptr = std::make_shared<PIDController>(arc_turn_config);
}

void TankRobotPlugin::initAutonomy()
{
  std::cout << "[TankRobotPlugin::initAutonomy]" << std::endl;
  node_ptr_->declare_parameter<std::string>("bt_path");
  std::string bt_path = node_ptr_->get_parameter("bt_path").as_string();

  node_ptr_->declare_parameter<std::string>("bt_path_interaction");
  m_bt_path_interaction = node_ptr_->get_parameter("bt_path_interaction").as_string();
  // auto bt_interaction = std::make_shared<TankTree>(m_bt_path_interaction);
  // try {
  //   std::cout << "Initializing Interaction Behavior Tree" << std::endl;
  //   bt_interaction->init_tree();
  // } catch (std::exception & e) {
  //   std::cout << "Error init_tree: " << e.what() << std::endl;
  // }

  node_ptr_->declare_parameter<std::string>("config_path");
  std::string config_path = node_ptr_->get_parameter("config_path").as_string();

  node_ptr_->declare_parameter<double>("tank_robot_plugin.ring_score_timeout");
  m_ring_score_timeout = node_ptr_->get_parameter("tank_robot_plugin.ring_score_timeout").as_double();

  node_ptr_->declare_parameter<double>("tank_robot_plugin.ring_prewait_time");
  m_ring_prewait_time = node_ptr_->get_parameter("tank_robot_plugin.ring_prewait_time").as_double();

  bt_ = std::make_shared<TankTree>(bt_path);
  bt_->set_variable("rhi_ptr", rhi_ptr_);
  bt_->set_variable("tank_model_ptr", m_tank_model_ptr);
  bt_->set_variable("node_ptr", node_ptr_);
  bt_->set_variable("tank_trajectory_ptr", tank_trajectory_ptr_);
  bt_->set_variable("distance_approach_controller_ptr", m_distance_approach_controller_ptr);
  bt_->set_variable("distance_settling_controller_ptr", m_steering_approach_controller_ptr);
  bt_->set_variable("steering_approach_controller_ptr", m_distance_settling_controller_ptr);
  bt_->set_variable("steering_settling_controller_ptr", m_steering_settling_controller_ptr);
  bt_->set_variable("arc_turn_controller_ptr", m_arc_turn_controller_ptr);
  bt_->set_variable("trajectory_viz_pub", m_trajectory_viz_pub);
  bt_->set_variable("digital_io_port_map", digital_io_port_map);
  bt_->set_variable("config_path", config_path);
  resetBT();
}

/////////////////////
/// State Machine ///
/////////////////////
void TankRobotPlugin::onNewSensorData()
{
  static bool first_loop = true;
  if (first_loop) {
    playMusic("hello_there");
    resetWorldPose();
    first_loop = false;
  }

  // Clear current limits at start of loop
  m_loop_current_limits.clear();
// 
  // updateConveyorPositionSensing();
  publishIMUData();
  updateAndPublishOdometry();
  // publishTrajectoryVisualization();
}


void TankRobotPlugin::publishIMUData()
{
  sensor_msgs::msg::Imu imu_msg{};
  imu_msg.header.frame_id = "imu_link";
  imu_msg.header.stamp = node_ptr_->get_clock()->now();
  if (!std::isnan(rhi_ptr_->getInertialSensorXRate("imu"))) {
    imu_msg.angular_velocity.x = rhi_ptr_->getInertialSensorXRate("imu") * ghost_util::DEG_TO_RAD;
  }
  if (!std::isnan(rhi_ptr_->getInertialSensorYRate("imu"))) {
    imu_msg.angular_velocity.y = rhi_ptr_->getInertialSensorYRate("imu") * ghost_util::DEG_TO_RAD;
  }
  if (!std::isnan(rhi_ptr_->getInertialSensorZRate("imu"))) {
    imu_msg.angular_velocity.z = rhi_ptr_->getInertialSensorZRate("imu") * ghost_util::DEG_TO_RAD;
  }
  double world_yaw;
  if (!std::isnan(rhi_ptr_->getInertialSensorHeading("imu"))) {
    m_imu_yaw_rad = -rhi_ptr_->getInertialSensorHeading("imu") * ghost_util::DEG_TO_RAD;

    world_yaw = ghost_util::WrapAngle2PI(m_imu_yaw_rad + m_imu_offset_rad);
    ghost_util::yawToQuaternionRad(
      m_imu_yaw_rad, imu_msg.orientation.w, imu_msg.orientation.x,
      imu_msg.orientation.y, imu_msg.orientation.z);
  }
  imu_pub->publish(imu_msg);
}

void TankRobotPlugin::disabled()
{
  if (rhi_ptr_) {
    rhi_ptr_->setMotorVoltageCommandPercent("switcher_motor", 0.0);
  }
}

void TankRobotPlugin::autonomous(double current_time)
{
  if (m_is_first_auton_loop) {
    m_is_first_auton_loop = false;
    playTTS("starting autonomous");
    // m_odom_ptr->resetPose();
    // resetWorldPose();
    if (m_interaction) {
      bt_->set_path(m_bt_path_interaction);
      resetBT();
      m_tank_model_ptr->driveCommandTank(0.0, 0.0);
    }

    bt_->set_variable<bool>("clamp_closed", false);
    bt_->set_variable<bool>("bite_closed", false);
    bt_->set_variable<bool>("goal_rush_down", false);
    bt_->set_variable<bool>("goal_rush_l_down", false);
    bt_->set_variable<bool>("goal_rush_r_down", false);
    bt_->set_variable<bool>("conveyor_active", false);
    bt_->set_variable<bool>("store_ring", false);
    bt_->set_variable<bool>("ring_detector_active", false);
    bt_->set_variable<bool>("score_pos_up", false);
    bt_->set_variable<bool>("match_loading_up", false);
    bt_->set_variable<bool>("descorer_up", false);
    bt_->set_variable<int>("switcher_direction", 0);
    bt_->set_variable<int>("outtake_direction", 0);
    bt_->set_variable<int>("score_ball_direction", 0);
  }

  bt_->set_variable("auton_time_elapsed", current_time);
  // bt_->set_variable<bool>("mirrored", m_mirrored);

  try {
    bt_->tick_tree();
  } catch (std::exception & e) {
    std::cout << "Error tick_tree: " << e.what() << std::endl;
  }

  // Get best state estimate
  // auto curr_pose = m_tank_model_ptr->getWorldPose();
  auto curr_twist = m_tank_model_ptr->getWorldTwist();

  bool ring_detector_active = false;
  bool want_red = m_color_target_red;
  bool store_ring = false;
  bool conveyor_active = false;
  bool ground_intake_active = false;
  bool score_pos_up = false;
  int outtake_direction = 0;
  int score_ball_direction = 0;
  bt_->get_variable<bool>("ring_detector_active", ring_detector_active);
  bt_->get_variable<bool>("store_ring", store_ring);
  bt_->get_variable<bool>("conveyor_active", conveyor_active);
  bt_->get_variable<bool>("ground_intake_active", ground_intake_active);
  bt_->get_variable<bool>("score_pos_up", score_pos_up);
  bt_->get_variable<int>("outtake_direction", outtake_direction);
  bt_->get_variable<int>("score_ball_direction", score_ball_direction);

  if (score_pos_up) {
    rhi_ptr_->setMotorVoltageCommandPercent("intake_motor", 1.0);
    rhi_ptr_->setMotorVoltageCommandPercent("scorer_motor", 1.0);
    rhi_ptr_->setMotorCurrentLimitMilliAmps("intake_motor", 2500);
    rhi_ptr_->setMotorCurrentLimitMilliAmps("scorer_motor", 2500);
  } else if (score_ball_direction != 0) {
    // ScoreBall uses scorer_motor only (top); intake stays off
    double motor_pct = static_cast<double>(score_ball_direction);
    rhi_ptr_->setMotorVoltageCommandPercent("intake_motor", 0.0);
    rhi_ptr_->setMotorVoltageCommandPercent("scorer_motor", motor_pct);
    rhi_ptr_->setMotorCurrentLimitMilliAmps("intake_motor", 2500);
    rhi_ptr_->setMotorCurrentLimitMilliAmps("scorer_motor", 2500);
  } else if (outtake_direction != 0) {
    double intake_pct = static_cast<double>(outtake_direction);
    rhi_ptr_->setMotorVoltageCommandPercent("intake_motor", intake_pct);
    rhi_ptr_->setMotorVoltageCommandPercent("scorer_motor", 0.0);
    rhi_ptr_->setMotorCurrentLimitMilliAmps("intake_motor", 2500);
    rhi_ptr_->setMotorCurrentLimitMilliAmps("scorer_motor", 2500);
  } else if (conveyor_active) {
    updateConveyorOnly(true);
  } else if (ring_detector_active) {
    ringDetector(ring_detector_active, current_time, want_red, store_ring);
  } else {
    updateIntake(ground_intake_active, false, false, false);
  }

  // Update Pneumatics
  rhi_ptr_->setDigitalOut(digital_io_port_map["clamp"], bt_->get_variable<int>("clamp_closed"));
  if (m_mirrored) {
    rhi_ptr_->setDigitalOut(digital_io_port_map["goal_rush_l"], bt_->get_variable<int>("goal_rush_r_down") || bt_->get_variable<int>("goal_rush_down"));
    rhi_ptr_->setDigitalOut(digital_io_port_map["goal_rush_r"], bt_->get_variable<int>("goal_rush_l_down"));
  } else {
    rhi_ptr_->setDigitalOut(digital_io_port_map["goal_rush_l"], bt_->get_variable<int>("goal_rush_l_down"));
    rhi_ptr_->setDigitalOut(digital_io_port_map["goal_rush_r"], bt_->get_variable<int>("goal_rush_r_down") || bt_->get_variable<int>("goal_rush_down"));
  }
  rhi_ptr_->setDigitalOut(digital_io_port_map["bite"], bt_->get_variable<int>("bite_closed"));
  rhi_ptr_->setDigitalOut(digital_io_port_map["score_pos"], bt_->get_variable<int>("score_pos_up"));
  rhi_ptr_->setDigitalOut(digital_io_port_map["match_loading"], bt_->get_variable<int>("match_loading_up"));
  rhi_ptr_->setDigitalOut(digital_io_port_map["descorer"], bt_->get_variable<int>("descorer_up"));
  // Switcher motor control (auton): direction -1=down, 0=stop, 1=up
  int switcher_direction = 0;
  bt_->get_variable<int>("switcher_direction", switcher_direction);
  if (switcher_direction != 0) {
    double motor_pct = static_cast<double>(switcher_direction);
    rhi_ptr_->setMotorCurrentLimitMilliAmps("switcher_motor", 2500);
    rhi_ptr_->setMotorVoltageCommandPercent("switcher_motor", motor_pct);
  } else {
    rhi_ptr_->setMotorCurrentLimitMilliAmps("switcher_motor", 2500);
    rhi_ptr_->setMotorVoltageCommandPercent("switcher_motor", 0.0);
  }

  // Publish Twist Command
  geometry_msgs::msg::Twist msg{};
  bt_->get_variable("fwd_cmd", msg.linear.x);
  bt_->get_variable("turn_cmd", msg.angular.z);
  m_base_twist_cmd_pub->publish(msg);

  publishCurrentTwist(curr_twist);
  // publishDesiredTwist(m_desired_twist);

  if (bt_->get_variable("desired_pose", m_desired_pose)) {
    publishDesiredPose(m_desired_pose);
  }
}

void TankRobotPlugin::resetBT()
{
  try {
    std::cout << "Initializing Behavior Tree" << std::endl;
    bt_->init_tree();
  } catch (std::exception & e) {
    std::cout << "Error init_tree: " << e.what() << std::endl;
  }
  std::cout << "ResetBT Complete!" << std::endl;
}

void TankRobotPlugin::teleop(double current_time)
{
  auto joy_data = rhi_ptr_->getMainJoystickData();

  bool r2_held = joy_data->btn_r2;

  // Shutdown Request
  if (joy_data->btn_a && joy_data->btn_b && joy_data->btn_x && joy_data->btn_y &&
    joy_data->btn_u && joy_data->btn_l && joy_data->btn_d && joy_data->btn_r)
  {
    std::cout << "SHUTDOWN" << std::endl;
    std::system("echo 1 | sudo -S shutdown now");
    return;
  }

  // Auton Request
  m_running_auton = runAutonFromDriver(joy_data, current_time);
  if (m_running_auton) {
    return;
  }

  updateMusic(current_time, joy_data); //MUST RUN FIRST: pressing u takes over all right buttons
  toggleBagRecorder(joy_data);

  updateDescore(joy_data->btn_d);
  updateIntakeFromJoystick(joy_data);
  updateDrivetrain(joy_data);
  updateScorePos((!r2_held) && (joy_data->btn_l2));
  updateMatchLoading(joy_data->btn_b);
  updateColorSwitcher(joy_data->btn_u);
}

void TankRobotPlugin::ringDetector(bool active, double current_time, bool want_red, bool store_ring)
{

  static double last_input_time = 0.0;
  static double ring_found_time = 0.0;
  static double stuck_detection_time = 0.0;
  static bool running = false;
  static bool retry_mode = false;
  static double retry_start_time = 0.0;
  static int last_color = 0;

  static std::queue<int> ring_queue;
  static std::queue<double> ring_time_queue;

  static double last_time = 0.0;
  static double time_sum = 0.0;
  // if (!store_ring && ring_queue.size() < 2){
  time_sum += std::clamp(current_time - last_time, 0.0, 0.05);
  // }
  last_time = current_time;
  current_time = time_sum;

  std::cout << "current_time: " << current_time << std::endl;
  std::cout << "last_time: " << last_time << std::endl;
  std::cout << "time_sum: " << time_sum << std::endl;

  // Constants (adjust as needed for your specific system)
  const double STUCK_TIMEOUT = 1.5;      // Time to consider a ring stuck
  const double RETRY_DURATION = 0.8;     // How long to attempt the retry
  const double COOLDOWN_PERIOD = 0.2;    // Brief pause between retry attempts

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
      stuck_detection_time = current_time; // Reset stuck timer
    }
  }

  // Normal processing
  bool hook = false;
  bool ring_prewaited = (current_time - ring_found_time > m_ring_prewait_time);
  if (ring_prewaited && m_ring_found && !retry_mode) {
    // last_input_time = current_time;
    hook = true;
  } else if (ring_prewaited && !m_ring_found) {
    ring_found_time = 1000000.0;
    ring_queue.push(last_color);
    ring_time_queue.push(current_time);
  }

  // Determine hook and eject status
  if (retry_mode) {
    // During retry: alternate between hook on/off with a small cooldown period
    double retry_cycle = fmod(current_time - retry_start_time, COOLDOWN_PERIOD * 2);
    hook = (retry_cycle < COOLDOWN_PERIOD);
  } else if (!ring_queue.empty()) {
    // Normal hook logic
    if (store_ring) {
      // should not score the ring, will be stored in the center of the robot
      // hook = false;
      bool scoring_ring = current_time - ring_time_queue.front() < m_ring_score_timeout / 2.0;
      hook = scoring_ring;
    } else {
      // If not storing, keep hooks moving for an extra period of time to ensure scoring
      bool scoring_ring = current_time - ring_time_queue.front() < m_ring_score_timeout;
      hook = true;
      if (!scoring_ring) {
        ring_queue.pop();
        ring_time_queue.pop();
      }
    }
  }

  bool ejecting = false;
  static double last_eject_time = 0.0;
  bool eject = false;
  if (!retry_mode && !ring_queue.empty()) {  // Don't eject during retry attempts
    if (want_red) {
      ejecting = (ring_queue.front() == m_color_map["blue"]); //&& ring_prewaited;
    } else {
      ejecting = (ring_queue.front() == m_color_map["red"]); //&& ring_prewaited;
    }
  }

  // if (eject) {
  //   last_eject_time = current_time;
  // }
  // if (current_time - last_eject_time < 0.0) {
  //   last_eject_time = 0.0;
  // }
  // if (current_time - last_eject_time < 1.0) {
  //   ejecting = true;
  // }
  if (ejecting) {
    hook = false;
  }
  // std::cout << "hook: " << hook << std::endl;
  // std::cout << "ejecting: " << ejecting << std::endl;
  // std::cout << "queue.size: " << ring_queue.size() << std::endl;
  // std::cout << "queue.front: " << ring_queue.front() << std::endl;

  bool ground_intake = !store_ring || !(ring_queue.size() >= 2 && store_ring);

  last_color = m_ring_color;
  // std::cout << "queue.back: " << ring_queue.back() << std::endl;
  // Call motor control with determined states
  updateIntake(ground_intake, hook, ejecting, !hook && retry_mode);
}

bool TankRobotPlugin::runAutonFromDriver(JoyPtr joy_data, double current_time)
{
  static bool auton_button_pressed = false;
  if (joy_data->btn_u && joy_data->btn_l) {
    if (!auton_button_pressed) {
      auton_button_pressed = true;
      m_is_first_auton_loop = true;
      m_auton_start_time = current_time;
      resetBT();

      m_odom_ptr->resetPose();
      resetWorldPose();

      std::this_thread::sleep_for(500ms);
    }
    autonomous(current_time - m_auton_start_time);

    return true;
  }
  auton_button_pressed = false;
  return false;
}


void TankRobotPlugin::toggleBagRecorder(JoyPtr joy_data)
{
  return;
  if (joy_data->btn_y && joy_data->btn_x && !m_recording_btn_pressed) {
    m_recording_btn_pressed = true;
    if (!m_recording) {
      std::cout << "[TankRobotPlugin::toggleBagRecorder] Starting Bag Recorder!" << std::endl;
      auto req = std::make_shared<ghost_msgs::srv::StartRecorder::Request>();
      m_start_recorder_client->async_send_request(req);
    } else {
      std::cout << "[TankRobotPlugin::toggleBagRecorder] Stopping Bag Recorder!" << std::endl;
      auto req = std::make_shared<ghost_msgs::srv::StopRecorder::Request>();
      m_stop_recorder_client->async_send_request(req);
    }
    m_recording = !m_recording;
  } else if (!(joy_data->btn_y && joy_data->btn_x)) {
    m_recording_btn_pressed = false;
  }
}

void TankRobotPlugin::updateIntake(bool R2, bool R1, bool L1, bool L2)
{
  double intake_power = 0.0;
  double scorer_power = 0.0;

  // R2 logic: Intake mode
  if (R1) {
    intake_power = 1.0;     // Intake motor always runs with R2

    // Chord logic: R2 is held, check for L1/L2
    

    // Hardware Update: Write the forced state to the piston immediately
    rhi_ptr_->setDigitalOut(digital_io_port_map["score_pos"], m_score_pos_up);
  }
  // R1 logic: Outtake mode (Reverses both motors)
  else if (R2) {
    intake_power = -1.0;
  } 
  else if (L2) {
      m_score_pos_up = true;        // Force scoring piston UP
      scorer_power = 1.0;           // Activate scorer motor
      intake_power = 1.0;
          rhi_ptr_->setDigitalOut(digital_io_port_map["score_pos"], m_score_pos_up);

  } 
  else if (L1) {
      m_score_pos_up = false;       // Force scoring piston DOWN
      scorer_power = 1.0;       
      intake_power = 1.0;
      rhi_ptr_->setDigitalOut(digital_io_port_map["score_pos"], m_score_pos_up);

  } 
  else {
      // R2 alone: only the intake motor runs
      intake_power = 0.0;
      scorer_power = 0.0;
  }

  // Set motor voltages
  rhi_ptr_->setMotorVoltageCommandPercent("intake_motor", intake_power);
  rhi_ptr_->setMotorVoltageCommandPercent("scorer_motor", scorer_power);

  // Safety current limits to prevent burnouts during jams
  rhi_ptr_->setMotorCurrentLimitMilliAmps("intake_motor", 2500);
  rhi_ptr_->setMotorCurrentLimitMilliAmps("scorer_motor", 2500);
}

void TankRobotPlugin::updateIntakeFromJoystick(JoyPtr joy_data)
{
  // Pass R2 for intake, R1 for outtake
  updateIntake(joy_data->btn_r2, joy_data->btn_r1, joy_data->btn_l1, joy_data->btn_l2);
}


void TankRobotPlugin::updateMatchLoading(bool input)
{
  static bool last_btn_b_state = false;
  if (input && !last_btn_b_state) {
    m_match_loading_up = !(m_match_loading_up);
  }
  last_btn_b_state = input;

  rhi_ptr_->setDigitalOut(digital_io_port_map["match_loading"], m_match_loading_up);
}


void TankRobotPlugin::updateColorSwitcher(bool input)
{
  static bool last_btn_d_state = false;
  if (input && !last_btn_d_state) {
    m_color_switcher = !(m_color_switcher);
  }
  last_btn_d_state = input;

  rhi_ptr_->setDigitalOut(digital_io_port_map["color_sorter"], !m_color_switcher);

}

void TankRobotPlugin::updateScorePos(bool input)
{
  static bool last_l2_state = false;
  if (input && !last_l2_state) {
    m_score_pos_up = !(m_score_pos_up);
  }
  last_l2_state = input;

  rhi_ptr_->setDigitalOut(digital_io_port_map["score_pos"], m_score_pos_up);

}


void TankRobotPlugin::updateDescore(bool input)
{
  static bool last_l1_state = false;
  if (input && !last_l1_state) {
    m_descore_up = !(m_descore_up);
  }
  last_l1_state = input;

  rhi_ptr_->setDigitalOut(digital_io_port_map["descorer"], m_descore_up);

}

// pressing u takes over all right buttons
void TankRobotPlugin::updateMusic(double current_time, JoyPtr joy_data)
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


void TankRobotPlugin::updateDrivetrain(JoyPtr joy_data)
{
  double left_pct = joy_data->left_y / 127.0;
  double right_pct = joy_data->right_y / 127.0;
  m_tank_model_ptr->driveCommandTank(left_pct, right_pct);

  int32_t drive_curr_lim = static_cast<int32_t>(ghost_control::v5_current_limiting::getRemainingCurrentDistributed(m_loop_current_limits, m_num_motors));
  if (std::fabs(left_pct) < 0.05 && std::fabs(right_pct) < 0.05) {
    drive_curr_lim = 0;
  }

  for (const auto & name : m_all_drive_motor_names) {
    rhi_ptr_->setMotorCurrentLimitMilliAmps(name, drive_curr_lim);
  }
}

void TankRobotPlugin::worldOdometryUpdateCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
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

void TankRobotPlugin::worldOdometryUpdateCallbackBackup(
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

void TankRobotPlugin::publishBaseTwist()
{
  //geometry_msgs::msg::Twist msg{};
  //auto base_vel_cmd = m_tank_model_ptr->getBaseVelocityCommand();
  //msg.linear.x = base_vel_cmd.x();
  //msg.linear.y = base_vel_cmd.y();
  //msg.angular.z = base_vel_cmd.z();
  //m_base_twist_cmd_pub->publish(msg);
}

void TankRobotPlugin::updateAndPublishOdometry()
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
    Eigen::Map<Eigen::VectorX<long>>(r_pos.data(), r_pos.size())
  );

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

    // printf("delta_theta: %.4f  yaw: %.4f\n", odom_diff_theta, m_curr_odom_pose.z());

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

  //auto current_velocity = m_tank_model_ptr->getBaseVelocityCurrent();

  //msg.twist.twist.linear.x = current_velocity.x();
  //msg.twist.twist.linear.y = current_velocity.y();
  //msg.twist.twist.linear.z = 0.0;
  //msg.twist.twist.angular.x = 0.0;
  //msg.twist.twist.angular.y = 0.0;
  //msg.twist.twist.angular.z = current_velocity.z();

  //double sigma_x_vel =
  //  m_k1 * current_velocity.x() +
  //  m_k2 * current_velocity.y() +
  //  m_k3 * abs(current_velocity.z());
  //double sigma_y_vel =
  //  m_k4 * current_velocity.x() +
  //  m_k5 * current_velocity.y() +
  //  m_k6 * abs(current_velocity.z());
  //// Get noisy angle
  //double sigma_tht_vel =
  //  m_k7 * current_velocity.x() +
  //  m_k8 * current_velocity.y() +
  //  m_k9 * abs(current_velocity.z());

  //std::array<double, 36> vel_covariance{
  //  sigma_x_vel * sigma_x_vel, 0.0, 0.0, 0.0, 0.0, 0.0,
  //  0.0, sigma_y_vel * sigma_y_vel, 0.0, 0.0, 0.0, 0.0,
  //  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  //  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  //  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  //  0.0, 0.0, 0.0, 0.0, 0.0, sigma_tht_vel * sigma_tht_vel};

  //msg.twist.covariance = vel_covariance;

  m_odom_pub->publish(msg);

  // Cache for the watchdog so it can hold this value if the brain drops out.
  m_last_odom_msg = msg;
  m_last_sensor_time = std::chrono::steady_clock::now();

  m_last_odom_pose = m_curr_odom_pose;
}

void TankRobotPlugin::odomWatchdogLoop()
{
  // Something was published in the last second; nothing to do.
  if (std::chrono::steady_clock::now() - m_last_sensor_time <
    std::chrono::seconds(1))
  {
    return;
  }
  // Stale: republish the last odom so odom_ekf keeps getting a measurement
  // (identity until the first real reading).
  m_last_odom_msg.header.stamp = node_ptr_->get_clock()->now();
  m_odom_pub->publish(m_last_odom_msg);
}

void TankRobotPlugin::resetWorldPose()
{
  // Copy yaml vectors to array
  std::array<double, m_cov_n> m_initial_estimate_covariance_arr;
  m_initial_estimate_covariance_arr[0] = m_init_sigma_x * m_init_sigma_x;
  m_initial_estimate_covariance_arr[7] = m_init_sigma_y * m_init_sigma_y;
  m_initial_estimate_covariance_arr[35] = m_init_sigma_theta * m_init_sigma_theta;

  geometry_msgs::msg::Quaternion quat{};
  geometry_msgs::msg::PoseWithCovarianceStamped new_pose{};
  new_pose.header.frame_id = "map";
  new_pose.header.stamp = node_ptr_->get_clock()->now();

  Eigen::Vector2d reset_pose = m_reset_pose_xy_m;
  double reset_angle = m_reset_pose_angle_rad;

  if (m_mirrored) {
    reset_pose.x() = 6 * 24.0 * 2.54 / 100.0 - m_reset_pose_xy_m.x();
    reset_angle = M_PI - m_reset_pose_angle_rad;
  }
  ghost_util::yawToQuaternionRad(reset_angle, quat.w, quat.x, quat.y, quat.z);
  new_pose.pose.pose.position.x = reset_pose.x();
  new_pose.pose.pose.position.y = reset_pose.y();

  new_pose.pose.pose.orientation = quat;
  new_pose.pose.covariance = m_initial_estimate_covariance_arr;

  // Publish to Particle Filter
  m_reset_pf_pub->publish(new_pose);
  if (m_mirrored) {
    std::cout << "Done reset: mirrored" << std::endl;
  } else {
    std::cout << "Done reset: regular" << std::endl;
  }
  bt_->set_variable<bool>("mirrored", m_mirrored);
}

void TankRobotPlugin::publishCurrentTwist(
  Eigen::Vector3d twist)
{
  geometry_msgs::msg::Twist msg{};
  msg.linear.x = twist.x();
  msg.linear.y = twist.y();
  msg.angular.z = twist.z();
  m_cur_twist_pub->publish(msg);
}

void TankRobotPlugin::publishDesiredTwist(
  Eigen::Vector3d twist)
{
  geometry_msgs::msg::Twist msg{};
  msg.linear.x = twist.x();
  msg.linear.y = twist.y();
  msg.angular.z = twist.z();
  m_des_twist_pub->publish(msg);
}

void TankRobotPlugin::publishDesiredPose(Eigen::Vector3d pose)
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

void TankRobotPlugin::publishErrorPose(Eigen::Vector3d pose)
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

// void TankRobotPlugin::publishTrajectoryVisualization()
// {
//   if (!robot_trajectory_ptr_->isNotEmpty()) {
//     return;
//   }
//   visualization_msgs::msg::MarkerArray msg{};

//   visualization_msgs::msg::Marker search_radius_marker{};
//   search_radius_marker.header.frame_id = "base_link";
//   search_radius_marker.header.stamp = node_ptr_->get_clock()->now();
//   search_radius_marker.id = 1;
//   search_radius_marker.type = 3;   // cylinder type
//   search_radius_marker.action = 0;
//   search_radius_marker.scale.x = 2 * m_search_radius;
//   search_radius_marker.scale.y = 2 * m_search_radius;
//   search_radius_marker.scale.z = 0.01;
//   search_radius_marker.color.b = 1.0;
//   search_radius_marker.color.a = 0.3;

//   visualization_msgs::msg::Marker carrot{};
//   carrot.header.frame_id = "map";
//   carrot.header.stamp = node_ptr_->get_clock()->now();
//   carrot.id = 2;
//   carrot.type = 4;   // line type
//   carrot.action = 0;
//   carrot.scale.x = 0.01;
//   carrot.scale.y = 1.0;
//   carrot.scale.z = 1.0;
//   carrot.color.g = 1.0;
//   carrot.color.a = 0.5;
//   geometry_msgs::msg::Point p_robot;
//   p_robot.x = m_tank_model_ptr->getWorldPose().x();
//   p_robot.y = m_tank_model_ptr->getWorldPose().y();
//   p_robot.z = 0.0;
//   geometry_msgs::msg::Point p_carrot;
//   p_carrot.set__x(m_desired_pose.x());
//   p_carrot.set__y(m_desired_pose.y());
//   p_carrot.z = 0.0;
//   carrot.points.push_back(p_robot);
//   carrot.points.push_back(p_carrot);

//   visualization_msgs::msg::Marker marker{};
//   marker.header.frame_id = "map";
//   marker.header.stamp = node_ptr_->get_clock()->now();
//   marker.id = 0;
//   marker.type = 8;   // points type
//   marker.action = 0;
//   marker.scale.x = 0.025;
//   marker.scale.y = 0.025;
//   marker.scale.z = 0.1;
//   marker.color.r = 1.0;
//   marker.color.a = 1.0;

//   for (int i = 0; i < robot_trajectory_ptr_->x_trajectory.position_vector.size(); i += 5) {
//     geometry_msgs::msg::Point p;
//     p.x = robot_trajectory_ptr_->x_trajectory.position_vector[i];
//     p.y = robot_trajectory_ptr_->y_trajectory.position_vector[i];
//     p.z = 0.0;
//     marker.points.push_back(p);
//   }
//   msg.markers.push_back(search_radius_marker);
//   msg.markers.push_back(carrot);
//   msg.markers.push_back(marker);
//   m_trajectory_viz_pub->publish(msg);
// }

void TankRobotPlugin::playMusic(std::string musicFileName)
{
  auto message = std_msgs::msg::String();
  message.data = musicFileName;
  m_music_pub->publish(message);
}

void TankRobotPlugin::playTTS(std::string textString)
{
  auto message = std_msgs::msg::String();
  message.data = textString;
  m_tts_pub->publish(message);
}

void TankRobotPlugin::colorTargetButtonCallback(const std_msgs::msg::Int64::SharedPtr msg)
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
  auto message = std_msgs::msg::Int64();
  message.data = m_color_target_red;
  m_led_color_red_pub->publish(message);
}

void TankRobotPlugin::mirroredButtonCallback(const std_msgs::msg::Int64::SharedPtr msg)
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
    bt_->set_variable<bool>("og_mirrored", m_mirrored);
  }

  auto message = std_msgs::msg::Int64();
  message.data = m_mirrored;
  m_led_side_right_pub->publish(message);
}

void TankRobotPlugin::resetButtonCallback(const std_msgs::msg::Int64::SharedPtr msg)
{
  if (m_reset != msg->data) {
    RCLCPP_INFO(node_ptr_->get_logger(), "m_reset state changed: %ld", msg->data);
    if (msg->data == 1) {
      m_reset = true;
    } else if (msg->data == 0) {
      m_reset = false;
    } else {
      RCLCPP_WARN(node_ptr_->get_logger(), "Received unknown button command: %ld", msg->data);
    }
    resetWorldPose();
  }
}


} // namespace ghost_tank

PLUGINLIB_EXPORT_CLASS(ghost_tank::TankRobotPlugin, ghost_ros_interfaces::V5RobotBase)
