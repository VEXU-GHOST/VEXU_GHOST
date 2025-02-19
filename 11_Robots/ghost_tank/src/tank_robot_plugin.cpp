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

using ghost_planners::RobotTrajectory;
using ghost_ros_interfaces::msg_helpers::fromROSMsg;
using std::placeholders::_1;

using ghost_v5_interfaces::devices::JoystickDeviceData;

using ghost_util::INCHES_TO_METERS;

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
  };
  m_left_drive_motor_names = {
    "drive_l1",
    "drive_l2",
    "drive_l3",
    "drive_l4",
    "drive_l5",
    "drive_l6",
  };

  m_all_motor_names.insert(
    m_all_motor_names.end(),
    m_left_drive_motor_names.begin(),
    m_left_drive_motor_names.end());

  m_all_motor_names.insert(
    m_all_motor_names.end(),
    m_right_drive_motor_names.begin(),
    m_right_drive_motor_names.end());

  m_all_motor_names.push_back("ground_pickup_motor");
  m_all_motor_names.push_back("conveyor_motor");
}

void TankRobotPlugin::populateDigitalIONames()
{
  digital_io_port_map["conveyor_switch"] = 0;
  digital_io_port_map["clamp"] = 6;
  digital_io_port_map["bite"] = 7;
}

//////////////////////
/// Initialization ///
//////////////////////

void TankRobotPlugin::initialize()
{
  initROSComms();
  initEstimation();
  initTankModel();
  initAutonomy();
}

void TankRobotPlugin::initROSComms()
{
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
  m_robot_pose_sub = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(pose_topic, 10, std::bind(&TankRobotPlugin::worldOdometryUpdateCallback, this, _1));

  node_ptr_->declare_parameter("backup_pose_topic", "/odom_ekf/odometry");
  std::string backup_pose_topic = node_ptr_->get_parameter("backup_pose_topic").as_string();
  m_robot_backup_pose_sub = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(backup_pose_topic, 10, std::bind(&TankRobotPlugin::worldOdometryUpdateCallbackBackup, this, _1));

  // Tank-Specific Publishers
  node_ptr_->declare_parameter("tank_robot_plugin.cmd_pose_topic", "/set_pose");
  std::string cmd_pose_topic = node_ptr_->get_parameter("tank_robot_plugin.cmd_pose_topic").as_string();
  m_set_pose_publisher = node_ptr_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(cmd_pose_topic, 10);

  node_ptr_->declare_parameter("input_imu_topic", "/sensors/imu");
  std::string input_imu_topic = node_ptr_->get_parameter("input_imu_topic").as_string();
  imu_pub = node_ptr_->create_publisher<sensor_msgs::msg::Imu>(input_imu_topic, 10);

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
}

void TankRobotPlugin::initEstimation()
{
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
}

void TankRobotPlugin::initTankModel()
{
  // Setup tank Model
  TankConfig tank_model_config;
  tank_model_config.motor_list = m_all_motor_names;
  tank_model_config.wheel_radius = 2.75 / 2.0; //in
  tank_model_config.wheel_gear_ratio = 20.0 / 23.0;
  tank_model_config.wheel_dist = 7.5; //in
  m_tank_model_ptr = std::make_shared<TankModel>(node_ptr_, rhi_ptr_, tank_model_config);

  node_ptr_->declare_parameter("tank_robot_plugin.search_radius", -1.0);
  m_search_radius = node_ptr_->get_parameter("tank_robot_plugin.search_radius").as_double();

  node_ptr_->declare_parameter("tank_robot_plugin.drive_motor_ticks_per_rotation", 0.0);
  node_ptr_->declare_parameter("tank_robot_plugin.drive_gear_ratio", 0.0);
  node_ptr_->declare_parameter("tank_robot_plugin.drive_wheel_size_inches", 0.0);
  node_ptr_->declare_parameter("tank_robot_plugin.wheel_base_inches", 0.0);

  double motor_ticks_per_rotation = node_ptr_->get_parameter("tank_robot_plugin.drive_motor_ticks_per_rotation").as_double();
  double drive_gear_ratio = node_ptr_->get_parameter("tank_robot_plugin.drive_gear_ratio").as_double();
  double wheel_size_inches = node_ptr_->get_parameter("tank_robot_plugin.drive_wheel_size_inches").as_double();
  double wheel_base_inches = node_ptr_->get_parameter("tank_robot_plugin.wheel_base_inches").as_double();

  m_odom_ptr = std::make_shared<TankOdometry>(motor_ticks_per_rotation * drive_gear_ratio, wheel_size_inches * INCHES_TO_METERS, wheel_base_inches * INCHES_TO_METERS);
  // m_odom_ptr->resetPose();

  node_ptr_->declare_parameter("tank_robot_plugin.move_to_pose_kp_xy", 0.5);
  node_ptr_->declare_parameter("tank_robot_plugin.move_to_pose_kd_xy", 0.5);
  node_ptr_->declare_parameter("tank_robot_plugin.move_to_pose_kp_theta", 0.5);
  node_ptr_->declare_parameter("tank_robot_plugin.move_to_pose_kd_theta", 0.5);
  node_ptr_->declare_parameter("tank_robot_plugin.max_speed_linear", 0.5);
  node_ptr_->declare_parameter("tank_robot_plugin.max_speed_angular", 0.5);
  float kp_xy = node_ptr_->get_parameter("tank_robot_plugin.move_to_pose_kp_xy").as_double();
  float kd_xy = node_ptr_->get_parameter("tank_robot_plugin.move_to_pose_kd_xy").as_double();
  float kp_theta = node_ptr_->get_parameter("tank_robot_plugin.move_to_pose_kp_theta").as_double();
  float kd_theta = node_ptr_->get_parameter("tank_robot_plugin.move_to_pose_kd_theta").as_double();
  m_max_speed_linear = node_ptr_->get_parameter("tank_robot_plugin.max_speed_linear").as_double();
  m_max_speed_angular = node_ptr_->get_parameter("tank_robot_plugin.max_speed_angular").as_double();

  m_boomerang = std::make_shared<Boomerang>();
  m_pd_control = std::make_shared<PDControl>(kp_xy, kd_xy, kp_theta, kd_theta);
}

void TankRobotPlugin::initAutonomy()
{
  node_ptr_->declare_parameter<std::string>("bt_path");
  std::string bt_path = node_ptr_->get_parameter("bt_path").as_string();

  node_ptr_->declare_parameter<std::string>("config_path");
  std::string config_path = node_ptr_->get_parameter("config_path").as_string();

  bt_ = std::make_shared<TankTree>(bt_path);
  bt_->set_variable("rhi_ptr", rhi_ptr_);
  bt_->set_variable("tank_model_ptr", m_tank_model_ptr);
  bt_->set_variable("node_ptr", node_ptr_);
  bt_->set_variable("pd_control_ptr", m_pd_control);
  bt_->init_tree();
}

/////////////////////
/// State Machine ///
/////////////////////
void TankRobotPlugin::onNewSensorData()
{
  publishIMUData();
  updateAndPublishOdometry();
  publishTrajectoryVisualization();
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
  double yaw;
  if (!std::isnan(rhi_ptr_->getInertialSensorHeading("imu"))) {
    yaw = -rhi_ptr_->getInertialSensorHeading("imu");
    ghost_util::yawToQuaternionDeg(
      yaw, imu_msg.orientation.w, imu_msg.orientation.x,
      imu_msg.orientation.y, imu_msg.orientation.z);
  }
  imu_pub->publish(imu_msg);
}

void TankRobotPlugin::disabled()
{
}

void TankRobotPlugin::autonomous(double current_time)
{
  // std::cout << "Autonomous: " << current_time << std::endl;
  bt_->set_variable("auton_time_elapsed", current_time);

  static bool first_loop = true;
  if (first_loop) {
    first_loop = false;
    // m_odom_ptr->resetPose();
  }

  bt_->tick_tree();

  // Get best state estimate
  // auto curr_pose = m_tank_model_ptr->getWorldPose();
  auto curr_twist = m_tank_model_ptr->getWorldTwist();

  publishCurrentTwist(curr_twist);
  // publishDesiredTwist(m_desired_twist);

  if(bt_->get_variable("desired_pose", m_desired_pose)){
    publishDesiredPose(m_desired_pose);
  }
  double fwd_cmd = 0.0;
  double turn_cmd = 0.0;
  if(bt_->get_variable("fwd_cmd", fwd_cmd)){
  }
  if(bt_->get_variable("turn_cmd", turn_cmd)){
  }

  geometry_msgs::msg::Twist msg{};
	msg.linear.x = fwd_cmd;
	msg.angular.z = turn_cmd;
	m_base_twist_cmd_pub->publish(msg);
}

void TankRobotPlugin::teleop(double current_time)
{
  auto joy_data = rhi_ptr_->getMainJoystickData();

  bool running_auton = runAutonFromDriver(joy_data, current_time);
  if (running_auton) {
    return;
  }

  toggleBagRecorder(joy_data);
  updateIntake(joy_data);
  updateBite(joy_data);
  updateClamp(joy_data);
  updateDrivetrain(joy_data);
}

bool TankRobotPlugin::runAutonFromDriver(std::shared_ptr<JoystickDeviceData> joy_data, double current_time)
{
  if (joy_data->btn_u && joy_data->btn_l) {
    if (!m_auton_button_pressed) {
      m_auton_button_pressed = true;
      m_is_first_auton_loop = true;
      m_auton_start_time = current_time;
      m_auton_index = 0;
    }
    autonomous(current_time - m_auton_start_time);

    return true;
  }
  m_auton_button_pressed = false;
  return false;
}


void TankRobotPlugin::toggleBagRecorder(std::shared_ptr<JoystickDeviceData> joy_data)
{
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

void TankRobotPlugin::updateIntake(std::shared_ptr<JoystickDeviceData> joy_data)
{
  double ground_pickup_power = 0;
  int32_t ground_pickup_current = 0;
  if (joy_data->btn_r2) {
    ground_pickup_power = 1.0;
    ground_pickup_current = 2500;
  } else if (joy_data->btn_r) {
    ground_pickup_power = -1.0;
    ground_pickup_current = 2500;
  } else {
    ground_pickup_power = 0.0;
    ground_pickup_current = 0;
  }

  double conveyor_power = 0;
  int32_t conveyor_current = 0;
  if (joy_data->btn_r1) {
    conveyor_power = 1.0;
    conveyor_current = 2500;
  } else if (joy_data->btn_l1) {
    conveyor_power = -1.0;
    conveyor_current = 2500;
  } else {
    conveyor_power = 0.0;
    conveyor_current = 0;
  }

  rhi_ptr_->setMotorVoltageCommandPercent("ground_pickup_motor", ground_pickup_power);
  rhi_ptr_->setMotorCurrentLimitMilliAmps("ground_pickup_motor", ground_pickup_current);

  rhi_ptr_->setMotorVoltageCommandPercent("conveyor_motor", conveyor_power);
  rhi_ptr_->setMotorCurrentLimitMilliAmps("conveyor_motor", conveyor_current);
}

void TankRobotPlugin::updateBite(std::shared_ptr<JoystickDeviceData> joy_data)
{
  static bool bite_btn_pressed = false;
  if (joy_data->btn_y && !bite_btn_pressed) {
    bite_btn_pressed = true;
    m_bite_closed = !m_bite_closed;
  } else if (!joy_data->btn_y) {
    bite_btn_pressed = false;
  }
  rhi_ptr_->setDigitalOut(digital_io_port_map["bite"], m_bite_closed);
}

void TankRobotPlugin::updateClamp(std::shared_ptr<JoystickDeviceData> joy_data)
{
  static bool clamp_btn_pressed = false;
  if (joy_data->btn_l2 && !clamp_btn_pressed) {
    clamp_btn_pressed = true;
    m_clamp_closed = !m_clamp_closed;
  } else if (!joy_data->btn_l2) {
    clamp_btn_pressed = false;
  }
  rhi_ptr_->setDigitalOut(digital_io_port_map["clamp"], m_clamp_closed);
}

void TankRobotPlugin::updateDrivetrain(std::shared_ptr<JoystickDeviceData> joy_data)
{
  m_tank_model_ptr->driveCommandJoystick(joy_data->left_y, -joy_data->right_x, 0.05);

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

  m_last_odom_pose = m_curr_odom_pose;
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

void TankRobotPlugin::publishTrajectoryVisualization()
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
  search_radius_marker.scale.x = 2*m_search_radius;
  search_radius_marker.scale.y = 2*m_search_radius;
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

} // namespace ghost_tank

PLUGINLIB_EXPORT_CLASS(ghost_tank::TankRobotPlugin, ghost_ros_interfaces::V5RobotBase)
