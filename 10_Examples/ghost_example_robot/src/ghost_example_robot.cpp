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

enum DriveMode {tank, arcade};

enum class ActionState {
    IDLE,
    DRIVE_FORWARD,
    DRIVE_BACKWARD,
    TURN_LEFT,
    TURN_RIGHT
};

ActionState action_state_ = ActionState::IDLE;
DriveMode drive_mode_ = DriveMode::tank; 

double action_start_time_ = 0.0;
double action_duration_ = 0.0;


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

if (joy_data->btn_a) {
        drive_mode_ = DriveMode::tank;
        std::cout << "Switched to TANK mode\n";
    }
if (joy_data->btn_b) {
        drive_mode_ = DriveMode::arcade;
        std::cout << "Switched to ARCADE mode\n";
    }

    // 10 forward
if (joy_data->btn_u){
  action_state_ = ActionState::DRIVE_FORWARD;
  action_start_time_ = current_time;
  action_duration_   = 1.0;   // placeholder duration
  std::cout << "Driving forward 10 in\n";
    }

    // 10 backward
if (joy_data->btn_d) {
  action_state_ = ActionState::DRIVE_BACKWARD;
  action_start_time_ = current_time;
  action_duration_   = 1.0;
  std::cout << "Driving backward 10 in\n";
    }

    // turn left 90
if (joy_data->btn_l) {
  action_state_ = ActionState::TURN_LEFT;
  action_start_time_ = current_time;
  action_duration_   = 1.0;
  std::cout << "Turning left 90°\n";
    }

    // turn right 90
if (joy_data->btn_r) {
  action_state_ = ActionState::TURN_RIGHT;
  action_start_time_ = current_time;
  action_duration_   = 1.0;
  std::cout << "Turning right 90°\n";
    }

  double left_cmd  = 0.0;
  double right_cmd = 0.0;

if (action_state_ != ActionState::IDLE){
        // action still active?
  if (current_time < action_start_time_ + action_duration_)
        {
    switch (action_state_) {
      case ActionState::DRIVE_FORWARD:
        left_cmd  =  0.3;
        right_cmd =  0.3;
        break;

      case ActionState::DRIVE_BACKWARD:
        left_cmd  = -0.3;
        right_cmd = -0.3;
        break;

      case ActionState::TURN_LEFT:
        left_cmd  = -0.3;
        right_cmd =  0.3;
        break;

      case ActionState::TURN_RIGHT:
        left_cmd  =  0.3;
        right_cmd = -0.3;
        break;

      default:
        break;
  }
  } else {
      action_state_ = ActionState::IDLE;
  }
} else {

// joystick control
        if (drive_mode_ == DriveMode::tank) {
            left_cmd  = joy_data->left_y  / 127.0;
            right_cmd = joy_data->right_y / 127.0;
        }
        else if (drive_mode_ == DriveMode::arcade) {
            double forward = joy_data->left_y  / 127.0;
            double turn    = joy_data->right_x / 127.0;

            left_cmd  = forward + turn;
            right_cmd = forward - turn;
        }
    }

    // motor command
    rhi_ptr_->setMotorVoltageCommandPercent("left_motor",  left_cmd);
    rhi_ptr_->setMotorVoltageCommandPercent("right_motor", right_cmd);

    rhi_ptr_->setMotorCurrentLimitMilliAmps("left_motor",  2500.0);
    rhi_ptr_->setMotorCurrentLimitMilliAmps("right_motor", 2500.0);


}


} // namespace ghost_example_robot


PLUGINLIB_EXPORT_CLASS(
  ghost_example_robot::GhostExampleRobot,
  ghost_ros_interfaces::V5RobotBase)
