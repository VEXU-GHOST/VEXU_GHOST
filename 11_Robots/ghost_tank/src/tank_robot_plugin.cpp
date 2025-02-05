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
    m_right_drive_motor_names.begin(),
    m_right_drive_motor_names.end());

  m_all_motor_names.insert(
    m_all_motor_names.end(),
    m_left_drive_motor_names.begin(),
    m_left_drive_motor_names.end());
}

void TankRobotPlugin::initialize()
{
  std::cout << "Tank Robot Initialization!" << std::endl;

  node_ptr_->declare_parameter("odom_topic", "/sensors/wheel_odom");
  std::string odom_topic = node_ptr_->get_parameter("odom_topic").as_string();

  node_ptr_->declare_parameter("pose_topic", "/odometry/filtered");
  std::string pose_topic = node_ptr_->get_parameter("pose_topic").as_string();

  node_ptr_->declare_parameter("backup_pose_topic", "/odom_ekf/odometry");
  std::string backup_pose_topic = node_ptr_->get_parameter("backup_pose_topic").as_string();

  node_ptr_->declare_parameter("tank_robot_plugin.use_backup_estimator", false);
  m_use_backup_estimator = node_ptr_->get_parameter("tank_robot_plugin.use_backup_estimator").as_bool();

  std::cout << "backup: " << m_use_backup_estimator << std::endl;

  node_ptr_->declare_parameter("joint_state_topic", "/joint_states");
  std::string joint_state_topic = node_ptr_->get_parameter("joint_state_topic").as_string();

  // vizualization
  node_ptr_->declare_parameter("trajectory_marker_topic", "/trajectory_markers");
  std::string trajectory_marker_topic =
    node_ptr_->get_parameter("trajectory_marker_topic").as_string();

  node_ptr_->declare_parameter<std::string>("bt_path");
  std::string bt_path = node_ptr_->get_parameter("bt_path").as_string();

  node_ptr_->declare_parameter<std::string>("config_path");
  std::string config_path = node_ptr_->get_parameter("config_path").as_string();

  // for vex ai
  //node_ptr_->declare_parameter<std::string>("bt_path_interaction");
  //std::string bt_path_interaction = node_ptr_->get_parameter("bt_path_interaction").as_string();

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

  // Setup tank Model
  TankConfig tank_model_config;
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
    "drive_r6",
  };
  tank_model_config.motor_list = motor_list;
  tank_model_config.wheel_radius = 2.75 / 2.0; //in
  tank_model_config.wheel_gear_ratio = 20.0 / 23.0;
  tank_model_config.wheel_dist = 7.5; //in

  node_ptr_->declare_parameter("tank_robot_plugin.search_radius", m_search_radius);
  m_search_radius = node_ptr_->get_parameter("tank_robot_plugin.search_radius").as_double();

  // initial position params
  node_ptr_->declare_parameter("particle_filter.init_world_x", m_init_world_x);
  node_ptr_->declare_parameter("particle_filter.init_world_y", m_init_world_y);
  node_ptr_->declare_parameter("particle_filter.init_world_theta", m_init_world_theta);

  m_init_world_x =
    node_ptr_->get_parameter("particle_filter.init_world_x").as_double();
  m_init_world_y =
    node_ptr_->get_parameter("particle_filter.init_world_y").as_double();
  m_init_world_theta =
    node_ptr_->get_parameter("particle_filter.init_world_theta").as_double();

  node_ptr_->declare_parameter("particle_filter.init_sigma_x", m_init_sigma_x);
  node_ptr_->declare_parameter("particle_filter.init_sigma_y", m_init_sigma_y);
  node_ptr_->declare_parameter("particle_filter.init_sigma_theta", m_init_sigma_theta);

  m_init_sigma_x =
    node_ptr_->get_parameter("particle_filter.init_sigma_x").as_double();
  m_init_sigma_y =
    node_ptr_->get_parameter("particle_filter.init_sigma_y").as_double();
  m_init_sigma_theta =
    node_ptr_->get_parameter("particle_filter.init_sigma_theta").as_double();

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

  node_ptr_->declare_parameter("cmd_twist_topic", "/cmd_vel");
  std::string cmd_twist_topic = node_ptr_->get_parameter("cmd_twist_topic").as_string();
  m_base_twist_cmd_pub = node_ptr_->create_publisher<geometry_msgs::msg::Twist>(
    cmd_twist_topic,
    10);

  node_ptr_->declare_parameter("bag_recorder_start_topic", "bag_recorder/start");
  std::string bag_recorder_start_topic =
    node_ptr_->get_parameter("bag_recorder_start_topic").as_string();
  m_start_recorder_client = node_ptr_->create_client<ghost_msgs::srv::StartRecorder>(
    bag_recorder_start_topic);

  node_ptr_->declare_parameter("bag_recorder_stop_topic", "bag_recorder/stop");
  std::string bag_recorder_stop_topic =
    node_ptr_->get_parameter("bag_recorder_stop_topic").as_string();
  m_stop_recorder_client = node_ptr_->create_client<ghost_msgs::srv::StopRecorder>(
    bag_recorder_stop_topic);

  node_ptr_->declare_parameter("cmd_pose_topic", "/set_pose");
  std::string cmd_pose_topic = node_ptr_->get_parameter("cmd_pose_topic").as_string();
  m_set_pose_publisher = node_ptr_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    cmd_pose_topic,
    10);

  imu_pub = node_ptr_->create_publisher<sensor_msgs::msg::Imu>(
    "/sensors/imu",
    10);

  node_ptr_->declare_parameter("des_twist_topic", "/des_vel");
  std::string des_twist_topic = node_ptr_->get_parameter("des_twist_topic").as_string();
  m_des_twist_pub = node_ptr_->create_publisher<geometry_msgs::msg::Twist>(
    des_twist_topic,
    10);

  node_ptr_->declare_parameter("cur_twist_topic", "/cur_vel");
  std::string cur_twist_topic = node_ptr_->get_parameter("cur_twist_topic").as_string();
  m_cur_twist_pub = node_ptr_->create_publisher<geometry_msgs::msg::Twist>(
    cur_twist_topic,
    10);

  node_ptr_->declare_parameter("des_pos_topic", "/des_pos");
  std::string des_pos_topic = node_ptr_->get_parameter("des_pos_topic").as_string();
  m_des_pos_pub = node_ptr_->create_publisher<geometry_msgs::msg::Pose>(
    des_pos_topic,
    10);

  node_ptr_->declare_parameter("tank_robot_plugin.sim_mode", false);
  m_sim_mode = node_ptr_->get_parameter("tank_robot_plugin.sim_mode").as_bool();

  bt_ = std::make_shared<TankTree>(bt_path);
  // bt_interaction_ = std::make_shared<TankTree>(bt_path_interaction);

  m_tank_model_ptr = std::make_shared<TankModel>(node_ptr_, rhi_ptr_, tank_model_config);

  // blue motor is 300, TODO put this in config files
  m_odom_ptr = std::make_shared<TankOdometry>(
    300. * 23. / 20., 2.75 * ghost_util::INCHES_TO_METERS / 2., 12.5 * ghost_util::INCHES_TO_METERS
  );
  m_odom_ptr->resetPose();


  bt_->set_variable("rhi_ptr", rhi_ptr_);
  bt_->set_variable("tank_model_ptr", m_tank_model_ptr);
  bt_->set_variable("node_ptr", node_ptr_);// have to move this in front of bt somehow
  bt_->init_tree();

  readPathFromFile(config_path + "/path.txt");

  std::cout << "Tank Robot Initialization Done!" << std::endl;
}

void TankRobotPlugin::onNewSensorData()
{
  sensor_msgs::msg::Imu imu_msg{};
  imu_msg.header.frame_id = "imu_link";
  // imu_msg.linear_acceleration.x = rhi_ptr_->getInertialSensorXAccel("imu");
  // imu_msg.linear_acceleration.y = rhi_ptr_->getInertialSensorYAccel("imu");
  // imu_msg.linear_acceleration.z = rhi_ptr_->getInertialSensorZAccel("imu");
  imu_msg.angular_velocity.x = rhi_ptr_->getInertialSensorXRate("imu") * ghost_util::DEG_TO_RAD;
  imu_msg.angular_velocity.y = rhi_ptr_->getInertialSensorYRate("imu") * ghost_util::DEG_TO_RAD;
  imu_msg.angular_velocity.z = rhi_ptr_->getInertialSensorZRate("imu") * ghost_util::DEG_TO_RAD;
  double yaw = -rhi_ptr_->getInertialSensorHeading("imu");
  ghost_util::yawToQuaternionDeg(
    yaw, imu_msg.orientation.w, imu_msg.orientation.x,
    imu_msg.orientation.y, imu_msg.orientation.z);
  imu_pub->publish(imu_msg);

  //m_tank_model_ptr->updateTankModel();

  // publishOdometry();
  // publishVisualization();
  // publishTrajectoryVisualization();


  std::vector<long> r_pos;
  std::vector<long> l_pos;

  for (const auto & name : m_right_drive_motor_names) {
    r_pos.push_back(rhi_ptr_->getMotorPosition(name));
  }

  for (const auto & name : m_left_drive_motor_names) {
    l_pos.push_back(rhi_ptr_->getMotorPosition(name));
  }

  m_odom_ptr->update(l_pos, r_pos);
  publishOdometry();
  // publishVisualization();
  // publishTrajectoryVisualization();
}

void TankRobotPlugin::disabled()
{
  // if (m_sim_mode) {
  //   // std::cout << "sim_time: " << current_time << std::endl;
  //   autonomous(0.0);
  //   return;
  // }
}

void TankRobotPlugin::autonomous(double current_time)
{
  std::cout << "Autonomous: " << current_time << std::endl;
  bt_->set_variable("auton_time_elapsed", current_time);

  // bt_->tick_tree();
  static bool first_loop = true;
  if (first_loop){
    first_loop = false;
    m_odom_ptr->resetPose();
  }

  // Get best state estimate
  auto curr_pose = m_tank_model_ptr->getWorldPose();
  auto curr_twist = m_tank_model_ptr->getWorldTwist();
  auto curr_vel_x = curr_twist.x();
  auto curr_vel_y = curr_twist.y();
  auto curr_vel_theta = curr_twist.z();

  publishCurrentTwist(curr_twist);
  // publishDesiredTwist(des_vel_x, des_vel_y, des_vel_theta);
  // publishDesiredPose(des_pos_x, des_pos_y, des_pos_theta);

  geometry_msgs::msg::Twist msg{};
  // msg.linear.x = forward_vel;
  msg.linear.y = 0;
  // msg.angular.z = angular_vel;
  m_base_twist_cmd_pub->publish(msg);

  movePointToPoint();
}

void TankRobotPlugin::teleop(double current_time)
{
  // if (m_sim_mode) {
  //   std::cout << "Teleop: " << current_time << std::endl;
  //   autonomous(current_time);
  //   return;
  // }

  auto joy_data = rhi_ptr_->getMainJoystickData();
  // std::cout << "Teleop: " << current_time << std::endl;

  if (joy_data->btn_u) {
    if (!m_auton_button_pressed) {
      m_auton_button_pressed = true;
      m_is_first_auton_loop = true;
      m_auton_start_time = current_time;
      m_auton_index = 0;
    }
    autonomous(current_time - m_auton_start_time);
  } else {
    m_auton_button_pressed = false;

    // Toggle Bag Recorder
    if (joy_data->btn_y && !m_recording_btn_pressed) {
      m_recording_btn_pressed = true;

      if (!m_recording) {
        auto req = std::make_shared<ghost_msgs::srv::StartRecorder::Request>();
        m_start_recorder_client->async_send_request(req);
      } else {
        auto req = std::make_shared<ghost_msgs::srv::StopRecorder::Request>();
        m_stop_recorder_client->async_send_request(req);
      }

      m_recording = !m_recording;
    } else if (!joy_data->btn_y) {
      m_recording_btn_pressed = false;
    }

    m_tank_model_ptr->driveCommandJoystick(
      joy_data->left_y, joy_data->right_x, 0.05);

    double intake_power = 0;
    if (joy_data->btn_r2) {
      intake_power = 1.0;
    } else if (joy_data->btn_r1) {
      intake_power = -1.0;
    } else {
      intake_power = 0.0;
    }

    // rhi_ptr_->setMotorVoltageCommandPercent(motor_list[5], intake_power);
    // rhi_ptr_->setMotorVoltageCommandPercent(motor_list[6], intake_power);

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

    // updateDrivetrainMotors();
  }
}

// make a class for this
// void TankRobotPlugin::onButtonPress(bool button){
//   static bool btn_r_pressed = false;
//   if (joy_data->btn_r && !btn_r_pressed) {
//     btn_r_pressed = true;
//     m_use_backup_estimator = !m_use_backup_estimator;
//   } else if (!joy_data->btn_r) {
//     btn_r_pressed = false;
//   }
// }

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

void TankRobotPlugin::publishOdometry()
{
  m_curr_odom_pose = m_odom_ptr->getPose();

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
    float search_radius = m_search_radius; 
    static int past_index = 0; 
    static int next_index = 0; 
    double current_x = m_tank_model_ptr->getWorldPose().x();
    double current_y = m_tank_model_ptr->getWorldPose().y();
    double current_angle = m_tank_model_ptr->getWorldAngleRad();
    // double current_x = m_curr_odom_pose.x();
    // double current_y = m_curr_odom_pose.y();
    // double current_angle = m_curr_odom_pose.z() + 1.57;
    if (past_index == x_values.size()-1){
      return;
    }

    if (x_values.size() != y_values.size()) {
      std::cout << "x_values and y_values must be the same size" << std::endl;
      throw std::runtime_error("x_values and y_values must be the same size");
    }
    for(int i = past_index; i < x_values.size(); ++i){//find farthest point in radius 
      double distance = sqrt(pow((current_x - x_values[i]),2)+pow((current_y - y_values[i]),2));
      if (distance < search_radius){
        next_index = i;
      }
    }
   
    //goal angle 
    std::cout << "next index:" << next_index << std::endl;
    std::cout << "size:" << x_values.size() << std::endl;
    double dx = x_values[next_index] - current_x;
    double dy = y_values[next_index] - current_y;
    double goal_radians = atan2(dy, dx);
    // double goal_degrees = angle_radians * (double)(180/3.14159265358987932);
    // if (goal_degrees <= 0){
    //     goal_degrees += 360;
    // }
    //turn to goal angle 
    auto turn_rad = ghost_util::SmallestAngleDistRad(goal_radians, current_angle);

    auto turn_limit = 0.3;
    if (turn_rad < turn_limit && turn_rad > -turn_limit){ //5.7 DEGREES
      turn_rad = 0;
    }

    //distance between current and goal pt 
    double distance = sqrt(pow(dx,2)+pow(dy,2));
    std::cout << "posx:" << current_x << std::endl;
    std::cout << "posy:" << current_y << std::endl;
    std::cout << "postheta:" << current_angle << std::endl;
    std::cout << "dx:" << dx << std::endl;
    std::cout << "dy:" << dy << std::endl;
    std::cout << "dtheta:" << turn_rad << std::endl;
    
    double fwd_cmd = distance * 0.5;
    double turn_cmd = turn_rad / 3.14 / 2.0;
    geometry_msgs::msg::Twist msg{};
    msg.linear.x = fwd_cmd;
    msg.angular.z = turn_cmd;
    m_base_twist_cmd_pub->publish(msg);
    m_tank_model_ptr->driveCommand(fwd_cmd, turn_cmd);
  }
}


 // namespace ghost_tank

PLUGINLIB_EXPORT_CLASS(ghost_tank::TankRobotPlugin, ghost_ros_interfaces::V5RobotBase)
