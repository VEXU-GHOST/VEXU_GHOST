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

#include <ghost_example_robot/ghost_example_robot.hpp>
#include <pluginlib/class_list_macros.hpp>

using ghost_planners::RobotTrajectory;
using ghost_ros_interfaces::msg_helpers::fromROSMsg;
using std::placeholders::_1;

namespace ghost_example_robot
{

GhostExampleRobot::GhostExampleRobot()
{
}

void GhostExampleRobot::initialize()
{
  // Only called once when program starts!
  std::cout << "initialize" << std::endl;

  odom_ = std::make_unique<ghost_tank::TankOdometry>(
    360,   // Degrees per wheel revolution; direct drive
    0.035, // Wheel radius in m
    0.21   // Distance between wheel centers in m
  );

  odom_pub_ = node_ptr_->create_publisher<nav_msgs::msg::Odometry>(
    "/sensors/wheel_odom",
    rclcpp::SensorDataQoS()
  );
}

void GhostExampleRobot::onNewSensorData()
{
  // Optional, called before disabled/autonomous/teleop when new data arrives.
  std::cout << "onNewSensorData" << std::endl;

  if (!odom_) return;

  const double left_position =
    rhi_ptr_->getMotorPosition("left_motor");
  const double right_position =
    rhi_ptr_->getMotorPosition("right_motor");

  if (!std::isfinite(left_position) ||
      !std::isfinite(right_position)) {
    return;
  }

  if (!encoder_baseline_set_) {
    previous_left_position_ = left_position;
    previous_right_position_ = right_position;
    encoder_baseline_set_ = true;
    return;
  }

  left_remainder_ += left_position - previous_left_position_;
  right_remainder_ += right_position - previous_right_position_;

  previous_left_position_ = left_position;
  previous_right_position_ = right_position;

  const long left_delta = static_cast<long>(left_remainder_);
  const long right_delta = static_cast<long>(right_remainder_);

  left_remainder_ -= left_delta;
  right_remainder_ -= right_delta;

  odom_->update(left_delta, right_delta);

  const auto pose = odom_->getPose();

  nav_msgs::msg::Odometry msg;

  msg.header.stamp = node_ptr_->get_clock()->now();
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";

  msg.pose.pose.position.x = pose.x();
  msg.pose.pose.position.y = pose.y();
  msg.pose.pose.position.z = 0.0;

  // Convert yaw to quaternion
  msg.pose.pose.orientation.x = 0.0;
  msg.pose.pose.orientation.y = 0.0;
  msg.pose.pose.orientation.z = std::sin(pose.z() / 2.0);
  msg.pose.pose.orientation.w = std::cos(pose.z() / 2.0);

  odom_pub_->publish(msg);
}

void GhostExampleRobot::disabled()
{
  std::cout << "disabled" << std::endl;
}

void GhostExampleRobot::autonomous(double current_time)
{
  std::cout << "Autonomous " << current_time << std::endl;
}

void GhostExampleRobot::teleop(double current_time)
{
  static int loop_count = 0;

  if (loop_count++ % 100 == 0) {
    std::cout << "Teleop " << current_time << std::endl;
  }

  auto joy_data = rhi_ptr_->getMainJoystickData();

  if (joy_data->btn_a) {
    std::cout << "Button A!" << std::endl;
  } else if (joy_data->btn_b) {
    std::cout << "Button B!" << std::endl;
  } else if (joy_data->btn_x) {
    std::cout << "Button X!" << std::endl;
  } else if (joy_data->btn_y) {
    std::cout << "Button Y!" << std::endl;
  } else if (joy_data->btn_u) {
    std::cout << "Button U!" << std::endl;
  } else if (joy_data->btn_d) {
    std::cout << "Button D!" << std::endl;
  } else if (joy_data->btn_l) {
    std::cout << "Button L!" << std::endl;
  } else if (joy_data->btn_r) {
    std::cout << "Button R!" << std::endl;
  } else if (joy_data->btn_l1) {
    std::cout << "Button L1!" << std::endl;
  } else if (joy_data->btn_l2) {
    std::cout << "Button L2!" << std::endl;
  }

  // Print joystick data!
  if (joy_data->btn_r1) {
    // Left joystick up-down axis is "left_y", left-right axis is "left_x"
    // Right joystick up-down axis is "right_y", left-right axis is "right_x"
    std::cout << "Left X: " << joy_data->left_x << std::endl;
    std::cout << "Left Y: " << joy_data->left_y << std::endl;
    std::cout << "Right X: " << joy_data->right_x << std::endl;
    std::cout << "Right Y: " << joy_data->right_y << std::endl;
    std::cout << std::endl;
  }

  // While holding button R2, send motor commands based on joystick values
  if (joy_data->btn_r2) {
    // Joysticks go from -127 to 127, but motors take a value from -1.0 to 1.0.
    double left_wheel_power = joy_data->left_y / 127.0;
    double right_wheel_power = joy_data->right_y / 127.0;

    // setMotorVoltageCommandPercent maps -1.0 <-> 1.0
    // to -12000 <-> 12000 millivolts behind the scenes.
    rhi_ptr_->setMotorVoltageCommandPercent(
      "left_motor",
      left_wheel_power);

    rhi_ptr_->setMotorVoltageCommandPercent(
      "right_motor",
      right_wheel_power);

    // Each motor has a current limit that defaults to zero.
    // This is so we can carefully allocate battery power between systems.
    rhi_ptr_->setMotorCurrentLimitMilliAmps(
      "left_motor",
      2500.0);

    rhi_ptr_->setMotorCurrentLimitMilliAmps(
      "right_motor",
      2500.0);

    // Now we can get motor data and print it.
    double left_position =
      rhi_ptr_->getMotorPosition("left_motor");

    double right_position =
      rhi_ptr_->getMotorPosition("right_motor");

    // These are in degrees.
    // Units and other data can be configured in example_hardware_config.yaml.
    std::cout << "Left Motor: "
              << left_position
              << " deg"
              << std::endl;

    std::cout << "Right Motor: "
              << right_position
              << " deg"
              << std::endl;

    std::cout << std::endl;
  } else {
    // Don't forget to turn motors off!
    rhi_ptr_->setMotorVoltageCommandPercent(
      "left_motor",
      0.0);

    rhi_ptr_->setMotorVoltageCommandPercent(
      "right_motor",
      0.0);

    rhi_ptr_->setMotorCurrentLimitMilliAmps(
      "left_motor",
      0.0);

    rhi_ptr_->setMotorCurrentLimitMilliAmps(
      "right_motor",
      0.0);
  }
}

}  // namespace ghost_example_robot

PLUGINLIB_EXPORT_CLASS(
  ghost_example_robot::GhostExampleRobot,
  ghost_ros_interfaces::V5RobotBase)