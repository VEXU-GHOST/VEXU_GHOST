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
#include <ghost_example_robot/ghost_example_robot.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>  // for std::clamp
#include <std.h>
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
}

void GhostExampleRobot::onNewSensorData()
{
  // Optional, called before disabled/autonomous/teleop when new data arrives.
  // std::cout << "onNewSensorData" << std::endl;
}

void GhostExampleRobot::disabled()
{
  std::cout << "disabled" << std::endl;
}

void GhostExampleRobot::autonomous(double current_time)
{
  std::cout << "Autonomous" << current_time << std::endl;
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
    // While holding button R2, send motor commands based on joystick values
  if (joy_data->btn_r2) {
    // Arcade drive scheme:
    //  - Forward/back: left joystick Y
    //  - Turn left/right: right joystick X
    double forward = joy_data->left_y / 127.0;
    double turn    = joy_data->right_x / 127.0;


    // Apply a small deadzone to ignore joystick drift
    constexpr double kDeadzone = 0.05;
    if (std::fabs(forward) < kDeadzone) forward = 0.0;
    if (std::fabs(turn)    < kDeadzone) turn    = 0.0;

    double left_power  = forward + turn;
    double right_power = forward - turn;

    // check if either power is outside of [-1.0, 1.0]

    // if so, scale both values down proportionally so that the highest

    // absolute value is 1.0

    double maxMagnitude = std::max(std::fabs(left_power), std::fabs(right_power));
    if (maxMagnitude > 1.0) {
      left_power /= maxMagnitude;
      right_power /= maxMagnitude;
    }

    // Send motor commands (maps ±1.0 to ±12000 mV internally)

    rhi_ptr_->setMotorVoltageCommandPercent("left_motor",  left_power);
    rhi_ptr_->setMotorVoltageCommandPercent("right_motor", right_power);
    // Set current limits while active
    rhi_ptr_->setMotorCurrentLimitMilliAmps("left_motor",  250
0.0);
    rhi_ptr_->setMotorCurrentLimitMilliAmps("right_motor", 2500
.0);
      

    // Tank drive scheme:


    // // Combine forward and turning into left/right power
    // double left_power  = forward + turn;
    // double right_power = forward - turn;

    // // Clamp to [-1.0, 1.0] so we never overdrive the motors
    // left_power  = std::clamp(left_power,  -1.0, 1.0);
    // right_power = std::clamp(right_power, -1.0, 1.0);

    // // Send motor commands (maps ±1.0 to ±12000 mV internally)
    // rhi_ptr_->setMotorVoltageCommandPercent("left_motor",  left_power);
    // rhi_ptr_->setMotorVoltageCommandPercent("right_motor", right_power);

    // // Set current limits while active
    // rhi_ptr_->setMotorCurrentLimitMilliAmps("left_motor",  2500.0);
    // rhi_ptr_->setMotorCurrentLimitMilliAmps("right_motor", 2500.0);

    // // Optional: print motor positions for debugging
    // double left_position  = rhi_ptr_->getMotorPosition("left_motor");
    // double right_position = rhi_ptr_->getMotorPosition("right_motor");
    // std::cout << "Left Motor: "  << left_position  << " deg" << std::endl;
    // std::cout << "Right Motor: " << right_position << " deg" << std::endl;
    // std::cout << std::endl;
    } else {

    // Don't forget to turn motors off!
    rhi_ptr_->setMotorVoltageCommandPercent("left_motor", 0.0);
    rhi_ptr_->setMotorVoltageCommandPercent("right_motor", 0.0);

    rhi_ptr_->setMotorCurrentLimitMilliAmps("left_motor", 0.0);
    rhi_ptr_->setMotorCurrentLimitMilliAmps("right_motor", 0.0);
  }
}
} // namespace ghost_example_robot

PLUGINLIB_EXPORT_CLASS(
  ghost_example_robot::GhostExampleRobot,
  ghost_ros_interfaces::V5RobotBase)
