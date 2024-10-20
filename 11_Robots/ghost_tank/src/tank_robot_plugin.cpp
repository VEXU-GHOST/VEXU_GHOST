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
#include <ghost_tank/tank_model.hpp>
#include <ghost_tank/tank_robot_plugin.hpp>
#include <ghost_util/angle_util.hpp>
#include <ghost_util/unit_conversion_utils.hpp>
#include <pluginlib/class_list_macros.hpp>

using ghost_planners::RobotTrajectory;
using ghost_ros_interfaces::msg_helpers::fromROSMsg;
using std::placeholders::_1;

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
    bt_path, bt_path_interaction);
    
  bt_->set_variable<int>("hello", 123);
  bt_->set_variable("rhi_ptr", rhi_ptr_);
  bt_->set_variable("tank_model_ptr", m_tank_model_ptr);
  bt_->set_variable("note_ptr", node_ptr_);

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

void TankRobotPlugin::autonomous(double current_time)
{
  std::cout << "Autonomous: " << current_time << std::endl;
  std::cout << "Is First Auton: " << m_is_first_auton_loop << std::endl;
  bt_->set_variable("auton_time_elapsed", current_time);

  // if(!m_recording){
  //    auto req = std::make_shared<ghost_msgs::srv::StartRecorder::Request>();
  //    m_start_recorder_client->async_send_request(req);
  //    m_recording = true;
  // }

  // vex AI only
  // if (!m_is_first_auton_loop) {
  //   bt_->tick_tree_interaction();
  // } else {
  bt_->tick_tree();
  // }

  // publishTrajectoryVisualization();

  // Get best state estimate
  auto curr_pose = m_tank_model_ptr->getWorldPose();
  auto curr_twist = m_tank_model_ptr->getWorldTwist();
  auto curr_vel_x = curr_twist.x();
  auto curr_vel_y = curr_twist.y();
  auto curr_vel_theta = curr_twist.z();

  publishCurrentTwist(curr_twist);
  // publishDesiredTwist(des_vel_x, des_vel_y, des_vel_theta);
  // publishDesiredPose(des_pos_x, des_pos_y, des_pos_theta);

  // if (m_tank_model_ptr->getAutoStatus()) {
  //   vel_cmd_x = 0.0;
  //   vel_cmd_y = 0.0;
  //   vel_cmd_theta = 0.0;
  // }

  // m_tank_model_ptr->drive command thing

  // std::vector<std::vector<double>> auton_instructions = {
  //   {3.0, 0.5, 0.0},
  //   {2.0, 0.0, 0.5},
  //   {4.0, 0.5, 0.0}
  // };

  double forward_vel = 0;
  double angular_vel = 0;
  // double time_sum = 0;
  // static int auton_index = 0;

  // if (auton_index < auton_instructions.size()) {
  //   if (current_time < auton_instructions[auton_index][0] + time_sum) {
  //     forward_vel = auton_instructions[auton_index][1];
  //     angular_vel = auton_instructions[auton_index][2];
  //   } else {
  //     time_sum += auton_instructions[auton_index][0];
  //     auton_index++;
  //   }
  // } else {

  // }

  double left_cmd = forward_vel + angular_vel;
  double right_cmd = forward_vel - angular_vel;

  std::vector<std::string> motor_list = {
    "drive_ltr",
    "drive_lbr",
    "drive_ltf",
    "drive_lbf",
    "drive_lttf",
    "indexer_right",
    "indexer_left",
    "drive_rttf",
    "drive_rtr",
    "drive_rbr",
    "drive_rtf",
    "drive_rbf"
  };

  for (const auto motor_name: motor_list) {
    rhi_ptr_->setMotorCurrentLimitMilliAmps(motor_name, 2500);
  }

  for (int i = 0; i < 5; i++) {
    rhi_ptr_->setMotorVoltageCommandPercent(motor_list[i], left_cmd);
  }

  for (int i = 7; i < 12; i++) {
    rhi_ptr_->setMotorVoltageCommandPercent(motor_list[i], right_cmd);
  }

  geometry_msgs::msg::Twist msg{};
  msg.linear.x = forward_vel;
  msg.linear.y = 0;
  msg.angular.z = angular_vel;
  m_base_twist_cmd_pub->publish(msg);
  // updateDrivetrainMotors();
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

void TankRobotPlugin::teleop(double current_time)
{
  auto joy_data = rhi_ptr_->getMainJoystickData();
  // std::cout << "Teleop: " << current_time << std::endl;

  if (joy_data->btn_a && joy_data->btn_b && joy_data->btn_x && joy_data->btn_y &&
    joy_data->btn_u && joy_data->btn_l && joy_data->btn_d && joy_data->btn_r)
  {
    std::system("echo 1 | sudo -S shutdown now");
  }

  if (joy_data->btn_u) {
    if (!m_auton_button_pressed) {
      m_auton_button_pressed = true;
      m_is_first_auton_loop = true;
      m_auton_start_time = current_time;
      m_auton_button_pressed = false;
      m_auton_index = 0;
    }
    autonomous(current_time - m_auton_start_time);
  } else {
    static bool reset_pose_btn_pressed = false;
    if (joy_data->btn_d && joy_data->btn_l && !reset_pose_btn_pressed && m_use_backup_estimator) {
      resetPose(0.0, 0.0, 0.0);
      reset_pose_btn_pressed = true;
    } else if (!joy_data->btn_d && !joy_data->btn_l) {
      reset_pose_btn_pressed = false;
    }

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


    static bool btn_r_pressed = false;
    if (joy_data->btn_r && !btn_r_pressed) {
      btn_r_pressed = true;
      m_use_backup_estimator = !m_use_backup_estimator;
    } else if (!joy_data->btn_r) {
      btn_r_pressed = false;
    }

    // m_curr_x_cmd = joy_data->left_x / 127.0;             // * scale;
    // m_curr_y_cmd = joy_data->left_y / 127.0;             // * scale;
    // m_curr_theta_cmd = joy_data->right_x / 127.0;             // * scale;

    // m_tank_model_ptr->drivecommandthing(
    //   m_curr_x_cmd, m_curr_y_cmd,
    //   m_curr_theta_cmd);

    // m_last_x_cmd = m_curr_x_cmd;
    // m_last_y_cmd = m_curr_y_cmd;
    // m_last_theta_cmd = m_curr_theta_cmd;

    double forward_vel = joy_data->left_y / 127.0;
    double angular_vel = joy_data->right_x / 127.0;

    double threshold = 0.05;
    forward_vel = (std::fabs(forward_vel) < threshold) ? 0.0 : forward_vel;
    angular_vel = (std::fabs(angular_vel) < threshold) ? 0.0 : angular_vel;

    double left_cmd = forward_vel + angular_vel;
    double right_cmd = forward_vel - angular_vel;

    std::vector<std::string> motor_list = {
      "drive_ltr",
      "drive_lbr",
      "drive_ltf",
      "drive_lbf",
      "drive_lttf",
      "indexer_right",
      "indexer_left",
      "drive_rttf",
      "drive_rtr",
      "drive_rbr",
      "drive_rtf",
      "drive_rbf"
    };

    for (const auto motor_name: motor_list) {
      rhi_ptr_->setMotorCurrentLimitMilliAmps(motor_name, 2500);
    }

    for (int i = 0; i < 5; i++) {
      rhi_ptr_->setMotorVoltageCommandPercent(motor_list[i], left_cmd);
    }

    for (int i = 7; i < 12; i++) {
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

    // updateDrivetrainMotors();

    // Intake
    // double intake_voltage;
    // if (joy_data->btn_r1) {
    //   rhi_ptr_->setMotorCurrentLimitMilliAmps("intake_motor", 2500);
    //   intake_voltage = -1.0;
    // } else if (joy_data->btn_r2) {
    //   rhi_ptr_->setMotorCurrentLimitMilliAmps("intake_motor", 2500);
    //   intake_voltage = 1.0;
    // } else {
    //   rhi_ptr_->setMotorCurrentLimitMilliAmps("intake_motor", 0);
    //   intake_voltage = 0.0;
    // }
    // rhi_ptr_->setMotorVoltageCommandPercent("intake_motor", intake_voltage);

    // intake burnout

    // // If INTAKE_MOTOR stalling, update state and timer
    // if ((intake_command) &&
    //   (std::fabs(rhi_ptr_->getMotorVelocityRPM("intake_motor")) < m_burnout_absolute_rpm_threshold))
    // {
    //   if (!m_intake_stalling) {
    //     m_intake_stall_start = node_ptr_->now();
    //     m_intake_stalling = true;
    //   }
    // } else {
    //   m_intake_stalling = false;
    // }

    // // If INTAKE_MOTOR stalled for too long, start cooldown period
    // if (!m_intake_cooling_down && m_intake_stalling &&
    //   ((node_ptr_->now() - m_intake_stall_start).nanoseconds() >
    //   m_burnout_stall_duration_ms * 1000000) )
    // {
    //   m_intake_stalling = false;
    //   m_intake_cooling_down = true;
    //   m_intake_cooldown_start = node_ptr_->now();
    // }

    // // Enforce INTAKE_MOTOR cooldown period
    // if (m_intake_cooling_down) {
    //   if (((node_ptr_->now() - m_intake_cooldown_start).nanoseconds() <=
    //     m_burnout_cooldown_duration_ms * 1000000) && intake_command)
    //   {
    //     rhi_ptr_->setMotorCurrentLimitMilliAmps("intake_motor", 0);
    //     rhi_ptr_->setMotorVoltageCommandPercent("intake_motor", 0);
    //   } else {
    //     m_intake_cooling_down = false;
    //   }
    // }
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

} // namespace ghost_tank

PLUGINLIB_EXPORT_CLASS(ghost_tank::TankRobotPlugin, ghost_ros_interfaces::V5RobotBase)
