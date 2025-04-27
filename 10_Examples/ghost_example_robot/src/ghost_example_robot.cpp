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
}

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
  static char control_scheme = ' ';
  static int loop_count = 0;
  static bool moving_forward = false;
  static bool moving_backward = false;
  static double move_start_time = 0.0;
  static double move_duration = 4.0; //complete pd move in 4 sec
  double kd = 0.01;
  double kp = 0.1;

  if (loop_count++ % 100 == 0) {
    std::cout << "Teleop " << current_time << std::endl;
  }

  auto joy_data = rhi_ptr_->getMainJoystickData();

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

  }
  else if (control_scheme == 'b') {
    double forward_val = joy_data->left_y / 127.0;
    double angular_val = joy_data->right_x / 127.0;
    double threshold = 0.05;

    if (std::abs(forward_val) < threshold) {
      forward_val = 0.0;
    }
    if (std::abs(angular_val) < threshold) {
      angular_val = 0.0;
    }

    rhi_ptr_->setMotorVoltageCommandPercent("left_motor", forward_val + angular_val);
    rhi_ptr_->setMotorVoltageCommandPercent("right_motor", forward_val - angular_val);

    rhi_ptr_->setMotorCurrentLimitMilliAmps("left_motor", 2500.0);
    rhi_ptr_->setMotorCurrentLimitMilliAmps("right_motor", 2500.0);
  }
  else {
    // Don't forget to turn motors off!
    rhi_ptr_->setMotorVoltageCommandPercent("left_motor", 0.0);
    rhi_ptr_->setMotorVoltageCommandPercent("right_motor", 0.0);

    rhi_ptr_->setMotorCurrentLimitMilliAmps("left_motor", 0.0);
    rhi_ptr_->setMotorCurrentLimitMilliAmps("right_motor", 0.0);
  }

  auto est_position = [](double t, bool forward) {
    double a = forward ? -5.0 / 16.0 : 5.0 / 16.0;
    double b = forward ? 15.0 / 8.0 : -15.0 / 8.0;
    double c = 0;
    double d = 0;
    return a * t * t * t + b * t * t + c * t + d;
  };

  auto est_velocity = [](double t, bool forward) {
    double a = forward ? -5.0 / 16.0 : 5.0 / 16.0;
    double b = forward ? 15.0 / 8.0 : -15.0 / 8.0;
    double c = 0;
    return 3 * a * t * t + 2 * b * t + c;
  };

  auto est_acceleration = [](double t, bool forward) {
    double a = forward ? -5.0 / 16.0 : 5.0 / 16.0;
    double b = forward ? 15.0 / 8.0 : -15.0 / 8.0;
    return 6 * a * t * t +  2 * b;
  };

  if(joy_data->btn_u && !moving_forward && !moving_backward) {
    moving_forward = true;
    move_start_time = current_time;
  }

  if(joy_data->btn_d && !moving_forward && !moving_backward) {
    moving_backward = true;
    move_start_time = current_time;
  }

  if(moving_forward || moving_backward) {
    double t = current_time - move_start_time;
    bool forward = moving_forward; //moving forward if true, moving backward if false.

    double goal_pos = est_position(t, forward);
    double goal_vel = est_velocity(t, forward);
    double goal_acc = est_acceleration(t, forward); 

    double pos_degrees = rhi_ptr_->getMotorPosition("left_motor");
    double pos_in = pos_degrees / 360.0 * 2.75 * 3.14159;
    double vel_rpm = rhi_ptr_->getMotorVelocityRPM("left_motor");
    double vel-in = vel_rpm / 60.0  * 2.75 * 3.14159;

    double error_pos = goal_pos - pos_in;
    double error_vel = goal_vel - vel_in; 

    double output = goal_acc + error_vel * kd + error_pos * kp;
    double factor = 3.75 + 0 * kd - kp * 10;
    double final_percentage = output/factor;
    double torque = std::max(std::min(final_percentage, 1.0), -1.0); //Limit percentage from -1.0 to 1.0

    rhi_ptr_->setMotorVoltageCommandPercent("left_motor", motor_percentage);
    rhi_ptr_->setMotorVoltageCommandPercent("right_motor", motor_percentage);

    //stop when motion profile reaches end
    if(t > move_duration) {
      moving_forward = false;
      moving_backward = false;
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