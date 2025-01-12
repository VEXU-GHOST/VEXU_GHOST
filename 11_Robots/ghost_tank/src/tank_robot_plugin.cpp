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
#include <ghost_util/unit_conversion_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <ghost_util/read_path.hpp>

using ghost_planners::RobotTrajectory;
using ghost_ros_interfaces::msg_helpers::fromROSMsg;
using std::placeholders::_1;

std::vector<double> x_values;
    std::vector<double> y_values;
    std::vector<double> angle_values;
namespace ghost_tank
{

TankRobotPlugin::TankRobotPlugin()
{
  // TODO: test and implement digital io in rhi
  m_digital_io = std::vector<bool>(8, false);
  // m_digital_io_name_map = std::unordered_map<std::string, size_t>{
  //   {"tail", 0},
  //   {"claw", 1}
  // };
}

void TankRobotPlugin::initialize()
{
  std::cout << "Tank Robot Initialization!" << std::endl;

  node_ptr_->declare_parameter("odom_topic", "/sensors/wheel_odom");
  std::string odom_topic = node_ptr_->get_parameter("odom_topic").as_string();

  node_ptr_->declare_parameter("pose_topic", "/odometry/filtered");
  std::string pose_topic = node_ptr_->get_parameter("pose_topic").as_string();

  node_ptr_->declare_parameter("backup_pose_topic", "/odometry/filtered");
  std::string backup_pose_topic = node_ptr_->get_parameter("backup_pose_topic").as_string();

  node_ptr_->declare_parameter("joint_state_topic", "/joint_states");
  std::string joint_state_topic = node_ptr_->get_parameter("joint_state_topic").as_string();

  // vizualization
  node_ptr_->declare_parameter("trajectory_marker_topic", "/trajectory_markers");
  std::string trajectory_marker_topic =
    node_ptr_->get_parameter("trajectory_marker_topic").as_string();

  node_ptr_->declare_parameter("tank_robot_plugin.joy_angle_control_threshold", 0.0);
  m_joy_angle_control_threshold = node_ptr_->get_parameter(
    "tank_robot_plugin.joy_angle_control_threshold").as_double();

  node_ptr_->declare_parameter<std::string>("bt_path");
  std::string bt_path = node_ptr_->get_parameter("bt_path").as_string();

  // for vex ai
  node_ptr_->declare_parameter<std::string>("bt_path_interaction");
  std::string bt_path_interaction = node_ptr_->get_parameter("bt_path_interaction").as_string();

  node_ptr_->declare_parameter("tank_robot_plugin.k1", 0.0);
  node_ptr_->declare_parameter("tank_robot_plugin.k2", 0.0);
  node_ptr_->declare_parameter("tank_robot_plugin.k3", 0.0);
  node_ptr_->declare_parameter("tank_robot_plugin.k4", 0.0);
  node_ptr_->declare_parameter("tank_robot_plugin.k5", 0.0);
  node_ptr_->declare_parameter("tank_robot_plugin.k6", 0.0);
  node_ptr_->declare_parameter("tank_robot_plugin.k7", 0.0);
  node_ptr_->declare_parameter("tank_robot_plugin.k8", 0.0);
  node_ptr_->declare_parameter("tank_robot_plugin.k9", 0.0);
  m_k1 = node_ptr_->get_parameter("tank_robot_plugin.k1").as_double();
  m_k2 = node_ptr_->get_parameter("tank_robot_plugin.k2").as_double();
  m_k3 = node_ptr_->get_parameter("tank_robot_plugin.k3").as_double();
  m_k4 = node_ptr_->get_parameter("tank_robot_plugin.k4").as_double();
  m_k5 = node_ptr_->get_parameter("tank_robot_plugin.k5").as_double();
  m_k6 = node_ptr_->get_parameter("tank_robot_plugin.k6").as_double();
  m_k7 = node_ptr_->get_parameter("tank_robot_plugin.k7").as_double();
  m_k8 = node_ptr_->get_parameter("tank_robot_plugin.k8").as_double();
  m_k9 = node_ptr_->get_parameter("tank_robot_plugin.k9").as_double();

  // Setup tank Model
  TankConfig tank_model_config;
  // TODO: define config params
  // tank_model_config.steering_ratio = 13.0 / 44.0;
  // tank_model_config.wheel_ratio = tank_model_config.steering_ratio * 30.0 / 14.0;
  // tank_model_config.wheel_radius = 2.75 / 2.0;

  // initial position params
  node_ptr_->declare_parameter("tank_robot_plugin.init_world_x", m_init_world_x);
  node_ptr_->declare_parameter("tank_robot_plugin.init_world_y", m_init_world_y);
  node_ptr_->declare_parameter("tank_robot_plugin.init_world_theta", m_init_world_theta);

  m_init_world_x = node_ptr_->get_parameter("tank_robot_plugin.init_world_x").as_double();
  m_init_world_y = node_ptr_->get_parameter("tank_robot_plugin.init_world_y").as_double();
  m_init_world_theta =
    node_ptr_->get_parameter("tank_robot_plugin.init_world_theta").as_double();

  node_ptr_->declare_parameter("tank_robot_plugin.init_sigma_x", m_init_sigma_x);
  node_ptr_->declare_parameter("tank_robot_plugin.init_sigma_y", m_init_sigma_y);
  node_ptr_->declare_parameter("tank_robot_plugin.init_sigma_theta", m_init_sigma_theta);

  m_init_sigma_x = node_ptr_->get_parameter("tank_robot_plugin.init_sigma_x").as_double();
  m_init_sigma_y = node_ptr_->get_parameter("tank_robot_plugin.init_sigma_y").as_double();
  m_init_sigma_theta =
    node_ptr_->get_parameter("tank_robot_plugin.init_sigma_theta").as_double();

  m_tank_model_ptr = std::make_shared<TankModel>(tank_model_config);

  // m_burnout_absolute_current_threshold_ma = node_ptr_->get_parameter(
  //   "tank_robot_plugin.burnout_absolute_current_threshold_ma").as_double();
  // m_burnout_absolute_rpm_threshold = node_ptr_->get_parameter(
  //   "tank_robot_plugin.burnout_absolute_velocity_threshold_rpm").as_double();
  // m_burnout_stall_duration_ms = node_ptr_->get_parameter(
  //   "tank_robot_plugin.burnout_stall_duration_ms").as_int();
  // m_burnout_cooldown_duration_ms = node_ptr_->get_parameter(
  //   "tank_robot_plugin.burnout_cooldown_duration_ms").as_int();

  // ROS Topics
  m_robot_pose_sub = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
    pose_topic,
    10,
    std::bind(&TankRobotPlugin::worldOdometryUpdateCallback, this, _1));

  m_robot_backup_pose_sub = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
    backup_pose_topic,
    10,
    std::bind(&TankRobotPlugin::worldOdometryUpdateCallbackBackup, this, _1));

  m_odom_pub = node_ptr_->create_publisher<nav_msgs::msg::Odometry>(
    odom_topic,
    10);

  m_joint_state_pub = node_ptr_->create_publisher<sensor_msgs::msg::JointState>(
    joint_state_topic,
    10);

  m_trajectory_viz_pub = node_ptr_->create_publisher<visualization_msgs::msg::MarkerArray>(
    trajectory_marker_topic,
    10);

  // TODO: parameterize these topics
  m_base_twist_cmd_pub = node_ptr_->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    10);

  m_start_recorder_client = node_ptr_->create_client<ghost_msgs::srv::StartRecorder>(
    "bag_recorder/start");

  m_stop_recorder_client = node_ptr_->create_client<ghost_msgs::srv::StopRecorder>(
    "bag_recorder/stop");

  m_set_pose_publisher = node_ptr_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/set_pose",
    10);

  imu_pub = node_ptr_->create_publisher<sensor_msgs::msg::Imu>(
    "/sensors/imu",
    10);

  m_des_vel_pub = node_ptr_->create_publisher<geometry_msgs::msg::Twist>(
    "/des_vel",
    10);

  m_cur_vel_pub = node_ptr_->create_publisher<geometry_msgs::msg::Twist>(
    "/cur_vel",
    10);

  m_des_pos_pub = node_ptr_->create_publisher<geometry_msgs::msg::Pose>(
    "/des_pos",
    10);

  bt_ = std::make_shared<TankTree>(
    bt_path, bt_path_interaction, rhi_ptr_, m_tank_model_ptr,
    node_ptr_);
    //read path from file 
    // std::string path_file = "path/to/your/file.csv";
    // readPathFromFile(path_file);
}

void TankRobotPlugin::onNewSensorData()
{
  sensor_msgs::msg::Imu imu_msg{};
  // imu_msg.header.frame_id = "imu_link";
  // imu_msg.header.stamp = node_ptr_->get_clock()->now();
  // imu_msg.linear_acceleration.x = rhi_ptr_->getInertialSensorXAccel("imu");
  // imu_msg.linear_acceleration.y = rhi_ptr_->getInertialSensorYAccel("imu");
  // imu_msg.linear_acceleration.z = rhi_ptr_->getInertialSensorZAccel("imu");
  // imu_msg.angular_velocity.x = rhi_ptr_->getInertialSensorXRate("imu") * ghost_util::DEG_TO_RAD;
  // imu_msg.angular_velocity.y = rhi_ptr_->getInertialSensorYRate("imu") * ghost_util::DEG_TO_RAD;
  // imu_msg.angular_velocity.z = rhi_ptr_->getInertialSensorZRate("imu") * ghost_util::DEG_TO_RAD;
  // double yaw = -rhi_ptr_->getInertialSensorHeading("imu");
  // ghost_util::yawToQuaternionDeg(
  //   yaw, imu_msg.orientation.w, imu_msg.orientation.x,
  //   imu_msg.orientation.y, imu_msg.orientation.z);
  // imu_pub->publish(imu_msg);

  m_tank_model_ptr->updateTankModel();

  // publishOdometry();
  // publishVisualization();
  // publishBaseTwist();
  // publishTrajectoryVisualization();
}

void TankRobotPlugin::disabled()
{
}

void TankRobotPlugin::go_forward(float target_inch)
{
  std::vector<std::string> motor_list = {
    "drive_l1",
    "drive_l2",
    "drive_l3",
    "drive_l4",
    "drive_l5",
    "drive_l6",
    "drive_r1",
    "drive_r2",
    "drive_r3",
    "drive_r4",
    "drive_r5",
    "drive_r6"
  };
  for (const auto motor_name: motor_list) {
    rhi_ptr_->setMotorCurrentLimitMilliAmps(motor_name, 2500);
  }

  double left_cmd = 0.0;
  double right_cmd = 0.0;

  static double tick_per_IN = 39.93342;
  double target_inch_distance = 1;

  float right_motor_position = rhi_ptr_->getMotorPosition("drive_r1");
  float left_motor_position = rhi_ptr_->getMotorPosition("drive_l1");
  float average = (right_motor_position + left_motor_position) / 2;
  static float p_constant = 5.0;


  if (average < abs(target_inch) * tick_per_IN) {
    left_cmd = p_constant * -0.01 * ((left_motor_position / tick_per_IN) - (target_inch));
    right_cmd = p_constant * -0.01 * ((right_motor_position / tick_per_IN) - (target_inch));

  } else {
    left_cmd = 0.0;
    right_cmd = 0.0;
  }

  //2.75 in per revolution
  /*if (counter<4){
    // for(int i= 0; i<4; i++){
    if (current_time < 4.0 + 5.0 *counter) {
      left_cmd = 10;
      right_cmd = 10;
    } else if (current_time < 5.0 + 5.0 * counter) {
      left_cmd = 10;
      right_cmd = 0;
    } else {
      left_cmd =0;
      right_cmd=0;
      counter = counter +1;
    }
    // }
  }
  */
  for (int i = 0; i < 6; i++) {
    rhi_ptr_->setMotorVoltageCommandPercent(motor_list[i], left_cmd);
  }

  for (int i = 6; i < 12; i++) {
    rhi_ptr_->setMotorVoltageCommandPercent(motor_list[i], right_cmd);
  }
}

void TankRobotPlugin::turn(float target_angle)
{
  std::vector<std::string> motor_list = {
    "drive_l1",
    "drive_l2",
    "drive_l3",
    "drive_l4",
    "drive_l5",
    "drive_l6",
    "drive_r1",
    "drive_r2",
    "drive_r3",
    "drive_r4",
    "drive_r5",
    "drive_r6"
  };
  for (const auto motor_name: motor_list) {
    rhi_ptr_->setMotorCurrentLimitMilliAmps(motor_name, 2500);
  }

  double left_cmd = 0.0;
  double right_cmd = 0.0;

  float robot_angle = m_curr_odom_pose.z();
  static float p_constant = 0.5;
  float pastdiff = 0;
  float diff = abs(target_angle - robot_angle);
  float ddiff = abs(diff- pastdiff);
  float p_const = 0.5;
  float d_const = 0.5;
  if (diff >0.5) {
    left_cmd = right_cmd = p_const* diff + d_const*ddiff;

  } else {
    left_cmd = 0.0;
    right_cmd = 0.0;
  }

 if (target_angle <0){
  for (int i = 0; i < 5; i++) {
    rhi_ptr_->setMotorVoltageCommandPercent(motor_list[i], right_cmd);
    
  }for (int i = 7; i < 12; i++) {
    rhi_ptr_->setMotorVoltageCommandPercent(motor_list[i], -1*left_cmd);
  }
 }else{
  for (int i = 0; i < 5; i++) {
    rhi_ptr_->setMotorVoltageCommandPercent(motor_list[i], -1* right_cmd);
    
  }for (int i = 7; i < 12; i++) {
    rhi_ptr_->setMotorVoltageCommandPercent(motor_list[i], left_cmd);
  }
  
}
pastdiff = diff;
}

void TankRobotPlugin::autonomous(double current_time)
{
  
  go_forward(10);


  //2.75 in per revolution


}

void TankRobotPlugin::teleop(double current_time)
{
  auto joy_data = rhi_ptr_->getMainJoystickData();

  double forward_vel = joy_data->left_y / 127.0;
  double angular_vel = joy_data->right_x / 127.0;

  double threshold = 0.05;
  forward_vel = (std::fabs(forward_vel) < threshold) ? 0.0 : forward_vel;
  angular_vel = (std::fabs(angular_vel) < threshold) ? 0.0 : angular_vel;

  double left_cmd = forward_vel + angular_vel;
  double right_cmd = forward_vel - angular_vel;

  // this is from ghost_high_stakes/config/robot_hardware_config_tank.yaml
  std::vector<std::string> motor_list = {
    "drive_l1",
    "drive_l2",
    "drive_l3",
    "drive_l4",
    "drive_l5",
    "drive_l6",
    "drive_r1",
    "drive_r2",
    "drive_r3",
    "drive_r4",
    "drive_r5",
    "drive_r6"
  };

  for (const auto motor_name: motor_list) {
    rhi_ptr_->setMotorCurrentLimitMilliAmps(motor_name, 2500);
  }

  for (int i = 0; i < 6; i++) {
    rhi_ptr_->setMotorVoltageCommandPercent(motor_list[i], left_cmd);
  }

  for (int i = 6; i < 12; i++) {
    rhi_ptr_->setMotorVoltageCommandPercent(motor_list[i], right_cmd);
  }

  double intake_power = 0;
  if (joy_data->btn_r2) {
    intake_power = 1.0;
  } else if (joy_data->btn_r1) {
    intake_power = -1.0;
  } else {
    intake_power = 0.0;
  }

  rhi_ptr_->setMotorVoltageCommandPercent(motor_list[5], intake_power);
  rhi_ptr_->setMotorVoltageCommandPercent(motor_list[6], intake_power);

  static bool forklift_pressed = false;
  static bool forklift_up = false;

  if (joy_data->btn_l1 && !forklift_pressed) {
    forklift_pressed = true;
    forklift_up = !forklift_up;
  } else if (!joy_data->btn_l1) {
    forklift_pressed = false;
  }

  m_digital_io[1] = forklift_up; // forklift
  m_digital_io[2] = joy_data->btn_l2; // pooper
  rhi_ptr_->setDigitalIO(m_digital_io);
}

// TODO: should/can this also reset ekf?
void TankRobotPlugin::resetPose(double x, double y, double theta)
{
  std::cout << "Resetting Pose!" << std::endl;
  m_last_odom_pose = m_curr_odom_pose;

  m_init_world_x = x;
  m_init_world_y = y;
  m_init_world_theta = theta;

  geometry_msgs::msg::PoseWithCovarianceStamped msg{};

  msg.header.frame_id = "odom";
  msg.header.stamp = node_ptr_->get_clock()->now();

  msg.pose.pose.position.x = x;
  msg.pose.pose.position.y = y;
  msg.pose.pose.position.z = 0;

  ghost_util::yawToQuaternionRad(
    theta,
    msg.pose.pose.orientation.w,
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z);

  msg.pose.covariance[0] = m_init_sigma_x * m_init_sigma_x;
  msg.pose.covariance[7] = m_init_sigma_y * m_init_sigma_y;
  msg.pose.covariance[35] = m_init_sigma_theta * m_init_sigma_theta;

  m_set_pose_publisher->publish(msg);
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
  geometry_msgs::msg::Twist msg{};
  auto base_vel_cmd = m_tank_model_ptr->getBaseVelocityCommand();
  msg.linear.x = base_vel_cmd.x();
  msg.linear.y = base_vel_cmd.y();
  msg.angular.z = base_vel_cmd.z();
  m_base_twist_cmd_pub->publish(msg);
}

void TankRobotPlugin::publishOdometry()
{
  m_curr_odom_pose = m_tank_model_ptr->getOdometryPose();

  nav_msgs::msg::Odometry msg{};
  msg.header.frame_id = "odom";
  msg.header.stamp = node_ptr_->get_clock()->now();
  msg.child_frame_id = "base_link";

  msg.pose.pose.position.x = m_curr_odom_pose.x();
  msg.pose.pose.position.y = m_curr_odom_pose.y();
  msg.pose.pose.position.z = 0.0;
  ghost_util::yawToQuaternionRad(
    m_curr_odom_pose.z(),
    msg.pose.pose.orientation.w,
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z);

  // Calculate differences for odometry
  auto odom_diff_x = std::fabs(m_curr_odom_pose.x() - m_curr_odom_pose.x());
  auto odom_diff_y = std::fabs(m_curr_odom_pose.y() - m_curr_odom_pose.y());
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

  auto current_velocity = m_tank_model_ptr->getBaseVelocityCurrent();

  msg.twist.twist.linear.x = current_velocity.x();
  msg.twist.twist.linear.y = current_velocity.y();
  msg.twist.twist.linear.z = 0.0;
  msg.twist.twist.angular.x = 0.0;
  msg.twist.twist.angular.y = 0.0;
  msg.twist.twist.angular.z = current_velocity.z();

  double sigma_x_vel =
    m_k1 * current_velocity.x() +
    m_k2 * current_velocity.y() +
    m_k3 * abs(current_velocity.z());
  double sigma_y_vel =
    m_k4 * current_velocity.x() +
    m_k5 * current_velocity.y() +
    m_k6 * abs(current_velocity.z());
  // Get noisy angle
  double sigma_tht_vel =
    m_k7 * current_velocity.x() +
    m_k8 * current_velocity.y() +
    m_k9 * abs(current_velocity.z());

  std::array<double, 36> vel_covariance{
    sigma_x_vel * sigma_x_vel, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, sigma_y_vel * sigma_y_vel, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, sigma_tht_vel * sigma_tht_vel};

  msg.twist.covariance = vel_covariance;

  m_odom_pub->publish(msg);

  m_last_odom_pose = m_curr_odom_pose;
}

void TankRobotPlugin::publishCurrentTwist(
  Eigen::Vector3d twist)
{
  geometry_msgs::msg::Twist msg{};
  msg.linear.x = twist.x();
  msg.linear.y = twist.y();
  msg.angular.z = twist.z();
  m_cur_vel_pub->publish(msg);
}

void TankRobotPlugin::publishDesiredTwist(
  Eigen::Vector3d twist)
{
  geometry_msgs::msg::Twist msg{};
  msg.linear.x = twist.x();
  msg.linear.y = twist.y();
  msg.angular.z = twist.z();
  m_des_vel_pub->publish(msg);
}

void TankRobotPlugin::publishDesiredPose(Eigen::Vector3d twist)
{
  geometry_msgs::msg::Pose msg{};
  msg.position.x = twist.x();
  msg.position.y = twist.y();
  ghost_util::yawToQuaternionRad(
    twist.z(),
    msg.orientation.w,
    msg.orientation.x,
    msg.orientation.y,
    msg.orientation.z);
  m_des_pos_pub->publish(msg);
}

void TankRobotPlugin::readPathFromFile(const std::string& filename) {
    ghost_util::readPathFromFile(filename, x_values, y_values, angle_values);
}


void TankRobotPlugin::movePointToPoint(){
   
    for (size_t i = 0; i < x_values.size(); ++i) {
        double goal_x = x_values[i];
        double goal_y = y_values[i];
        double goal_angle = angle_values[i];
    
        double current_x = 0;
        double current_y= 0;
        double current_angle = m_curr_odom_pose.z();

        //goal angle 
        double dx= goal_x - current_x;
        double dy= goal_y - current_y;
        double angle_radians = atan2(dx, dy);
        double goal_degrees= angle_radians * (double)(180/3.14159265358987932);
        if (goal_degrees <= 0){
          goal_degrees += 360;
        }

        //turn to goal angle 
        auto turn_degree =
    std::fabs(ghost_util::SmallestAngleDistDeg(goal_degrees, current_angle));

        turn(turn_degree);

      //distance between current and goal pt 
      double distance = sqrt(pow(dx,2)+pow(dy,2));

      //move 

      go_forward(distance);
      current_x = x_values[i];
      current_y= y_values[i];

      
    }
}


} // namespace ghost_tank

PLUGINLIB_EXPORT_CLASS(ghost_tank::TankRobotPlugin, ghost_ros_interfaces::V5RobotBase)
