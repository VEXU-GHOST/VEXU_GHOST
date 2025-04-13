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
  static char control_scheme = '';
  static int loop_count = 0;
  double a = -5/16;
  double b = 45/16;
  double c = -75/16;
  double d = 35/16;
  double kd = 0.01;
  double kp = 0.1;

  double est_position(double time) {
    return a*t*t*t + b*t*t + c*t + d;
  }
  double est_velocity(double time) {
    return 3*a*t*t + 2*b*t + c;
  }
  double est_acceleration(double time) {
    return 6*a*t + 2*b;
  }
  




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

  //updata control scheme
  if(joy_data->btn_a) {
    control_scheme = 'a';
  } else if(joy_data->btn_b) {
    control_scheme = 'b';
  }


  // Tank drive if button a is pressed
  if (control_scheme == 'a') {
    // Joysticks go from -127 to 127, but motors take a value from -1.0 to 1.0.
    double left_wheel_power = joy_data->left_y / 127.0;
    double right_wheel_power = joy_data->right_y / 127.0;

    // setMotorVoltageCommandPercent maps -1.0 <-> 1.0 to -12000 <-> 12000 milliVolts behind the scenes.
    rhi_ptr_->setMotorVoltageCommandPercent("left_motor", left_wheel_power);
    rhi_ptr_->setMotorVoltageCommandPercent("right_motor", right_wheel_power);

    // Each motor has a current limit that defaults to zero.
    // This is so we can carefully allocate battery power between systems.
    // If we don't set these, the motors will be extremely weak, if they move at all.
    rhi_ptr_->setMotorCurrentLimitMilliAmps("left_motor", 2500.0);
    rhi_ptr_->setMotorCurrentLimitMilliAmps("right_motor", 2500.0);

    // Now we can get motor data and print it.
    double left_position = rhi_ptr_->getMotorPosition("left_motor");
    double right_position = rhi_ptr_->getMotorPosition("right_motor");

    // These are in degrees. Units and other data can be configured in example_hardware_config.yaml.
    std::cout << "Left Motor: " << left_position << " deg" << std::endl;
    std::cout << "Right Motor: " << right_position << " deg" << std::endl;
    std::cout << std::endl;
  } else {
    // Don't forget to turn motors off!
    rhi_ptr_->setMotorVoltageCommandPercent("left_motor", 0.0);
    rhi_ptr_->setMotorVoltageCommandPercent("right_motor", 0.0);

    rhi_ptr_->setMotorCurrentLimitMilliAmps("left_motor", 0.0);
    rhi_ptr_->setMotorCurrentLimitMilliAmps("right_motor", 0.0);
  }

  // Arcade drive if button b is pressed
  if (control_scheme == 'b') {
    // Joysticks go from -127 to 127, but motors take a value from -1.0 to 1.0.
    double forward_val = joy_data->left_y / 127.0;
    double angular_val = joy_data->right_x / 127.0;
    double threashold = 0.05;

    if(std::abs(forward_val) < threashold) {
      forward_val = 0.0;
    }

    if(std::abs(angular_val) < threashold) {
      angular_val = 0.0;
    }
    
    // setMotorVoltageCommandPercent maps -1.0 <-> 1.0 to -12000 <-> 12000 milliVolts behind the scenes.
    rhi_ptr_->setMotorVoltageCommandPercent("left_motor", forward_val + angular_val);
    rhi_ptr_->setMotorVoltageCommandPercent("right_motor", forward_val - angular_val);

    // Each motor has a current limit that defaults to zero.
    // This is so we can carefully allocate battery power between systems.
    // If we don't set these, the motors will be extremely weak, if they move at all.
    rhi_ptr_->setMotorCurrentLimitMilliAmps("left_motor", 2500.0);
    rhi_ptr_->setMotorCurrentLimitMilliAmps("right_motor", 2500.0);

    // Now we can get motor data and print it.
    double left_position = rhi_ptr_->getMotorPosition("left_motor");
    double right_position = rhi_ptr_->getMotorPosition("right_motor");

    // These are in degrees. Units and other data can be configured in example_hardware_config.yaml.
    std::cout << "Left Motor: " << left_position << " deg" << std::endl;
    std::cout << "Right Motor: " << right_position << " deg" << std::endl;
    std::cout << std::endl;
  } else {
    // Don't forget to turn motors off!
    rhi_ptr_->setMotorVoltageCommandPercent("left_motor", 0.0);
    rhi_ptr_->setMotorVoltageCommandPercent("right_motor", 0.0);

    rhi_ptr_->setMotorCurrentLimitMilliAmps("left_motor", 0.0);
    rhi_ptr_->setMotorCurrentLimitMilliAmps("right_motor", 0.0);
  }
  //pd forward 10 inches
  if(joy_data->btn_u) {
    double threashold = 0.5;
    do{
    double pos_degrees = rhi_ptr_->getMotorPosition * 3;
    double pos_in = pos_degrees / 360 * 4 * 3.141593;
    double vel_in = rhi_ptr_->getMotorVelocityRPM * 60 * 4 * 3.141593;
    double torque = est_acceleration(current_time) + kd * (vel_in - est_velocity(current_time)) + kp * (pos_in - est_position(current_time));
    double mortor_percentage = torque/(8.4375 + kd * 2.8125 - kp * 1.875);
    rhi_ptr_->setMotorVoltageCommandPercent("left_mortor", mortor_percentage);
    rhi_ptr_->setMotorVoltageCommandPercent("right_motor", mortor_percentage);
    } while(rhi_ptr_->abs(getMotorCurrentMA) > threashold);
  }
}
} // namespace ghost_example_robot

PLUGINLIB_EXPORT_CLASS(
  ghost_example_robot::GhostExampleRobot,
  ghost_ros_interfaces::V5RobotBase)