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
  static int loop_count = 0;
  if (loop_count++ % 100 == 0) {
    std::cout << "Teleop " << current_time << std::endl;
  }
  // Get joystick data
  auto joy_data = rhi_ptr_->getMainJoystickData();
  double left_vel = joy_data->left_y / 127.0;
  double right_vel = joy_data->right_y / 127.0;
  double right_velx = joy_data->right_x / 127.0;  // sets values of velocity from -1 to 1
  static double distance = 0.0;
  static double distance_before = 0.0;  // keeps track of how far the robot has moved.
  const double threshold = 0.05;
  bool istank = true; //tank controls vs arcade
    left_vel = (std::fabs(left_vel) < threshold) ? 0.0 : left_vel;
    right_vel = (std::fabs(right_vel) < threshold) ? 0.0 : right_vel;
    right_velx = (std::fabs(right_velx) < threshold) ? 0.0 : right_velx;  // deadzone for joystick
    rhi_ptr_->setMotorCurrentLimitMilliAmps("left_motor", 2500.0);
    rhi_ptr_->setMotorCurrentLimitMilliAmps("right_motor", 2500.0); //possible to move
  if (std::fabs(distance) < 0.1){  // stops robot when reaches goal
    distance -= rhi_ptr_->getMotorPosition("left_motor") - distance_before;
    distance_before = rhi_ptr_->getMotorPosition("left_motor");

}else{
  if (istank) { // tank controls
    rhi_ptr_->setMotorVoltageCommandPercent("left_motor", left_vel);
    rhi_ptr_->setMotorVoltageCommandPercent("right_motor", right_vel);  // sets motor power to joystick values
  } else {  // arcade controls
    right_velx/=2;
    right_vel = left_vel - right_velx;  // determines the best velocity.
    left_vel = left_vel + right_velx;
    if (left_vel > 1.0){
      double temp = left_vel - 1.0;   // keeps the velocity values between -1 and 1 by altering opposite wheel.
      left_vel = 1.0;
      right_vel -= temp;
      }
    if (left_vel < -1.0){
      double temp = -left_vel - 1.0;
      left_vel = -1.0;
      right_vel += temp;
      }
      if (right_vel > 1.0){
      double temp = right_vel - 1.0;
      right_vel = 1.0;
      left_vel -= temp;
      }
    if (right_vel < -1.0){
      double temp = -right_vel - 1.0;
      right_vel = -1.0;
      left_vel += temp;
      }
    rhi_ptr_->setMotorVoltageCommandPercent("left_motor", left_vel);
    rhi_ptr_->setMotorVoltageCommandPercent("right_motor", right_vel);
  }
  if (joy_data->btn_a) {
    std::cout << "Button A!" << std::endl;  //tank control button
    istank = true;
  } else if (joy_data->btn_b) {
    std::cout << "Button B!" << std::endl;  //arcade control button
    istank = false;    
  } else if (joy_data->btn_x) {
    std::cout << "Button X!" << std::endl;
  } else if (joy_data->btn_y) {
    std::cout << "Button Y!" << std::endl;
  } else if (joy_data->btn_u) {
    std::cout << "Button U!" << std::endl;
    distance_before = rhi_ptr_->getMotorPosition("left_motor");
    distance = 10.0 + distance_before; // moves robot forward 10 degrees from current position
    rhi_ptr_->setMotorVoltageCommandPercent("left_motor", 1.0);
    rhi_ptr_->setMotorVoltageCommandPercent("right_motor", 1.0);
  } else if (joy_data->btn_d) {
    std::cout << "Button D!" << std::endl;
    distance_before = rhi_ptr_->getMotorPosition("left_motor");
    distance = distance_before - 10.0; // moves robot backward 10 degrees from current position
    rhi_ptr_->setMotorVoltageCommandPercent("left_motor", -1.0);
    rhi_ptr_->setMotorVoltageCommandPercent("right_motor", -1.0);
  } else if (joy_data->btn_l) {
    std::cout << "Button L!" << std::endl;
    distance_before = rhi_ptr_->getMotorPosition("left_motor");
    distance = distance_before - 11.0; // turns robot 90 degrees left from current position
    rhi_ptr_->setMotorVoltageCommandPercent("left_motor", -1.0);
    rhi_ptr_->setMotorVoltageCommandPercent("right_motor", 1.0);
  } else if (joy_data->btn_r) {
    std::cout << "Button R!" << std::endl;
    distance_before = rhi_ptr_->getMotorPosition("left_motor");
    distance = distance_before + 11.0; // turns robot 90 degrees right from current position
    rhi_ptr_->setMotorVoltageCommandPercent("left_motor", 1.0);
    rhi_ptr_->setMotorVoltageCommandPercent("right_motor", -1.0);
  } else if (joy_data->btn_l1) {
    std::cout << "Button L1!" << std::endl;
  } else if (joy_data->btn_l2) {
    std::cout << "Button L2!" << std::endl;
  }
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
}
} // namespace ghost_example_robot

PLUGINLIB_EXPORT_CLASS(
  ghost_example_robot::GhostExampleRobot,
  ghost_ros_interfaces::V5RobotBase)
