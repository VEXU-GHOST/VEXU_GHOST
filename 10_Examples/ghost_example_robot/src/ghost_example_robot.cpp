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

bool modeA = true;
bool modeB = false;
bool modeU = false;
bool modeD = false;
bool modeL = false;
bool modeR = false;

double diameter = 4.0;
double wheelDist = 8.0;
double turnDeg = 90.0;
double inchToDeg = 360.0 / (M_PI * diameter);

double start_left_pos = 0.0;
double start_right_pos = 0.0;
bool driving = false;

double prev_error_left = 0.0;
double prev_error_right = 0.0;



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

    if (joy_data->btn_a) {
      modeA = true;
      modeB = false;
      modeU = false;
      modeD = false;
      modeL = false;
      modeR = false;
    }

    if (joy_data->btn_b) {
      modeA = false;
      modeB = true;
      modeU = false;
      modeD = false;
      modeL = false;
      modeR = false;
    }

    if (joy_data->btn_u) {
      modeA = false;
      modeB = false;
      modeU = true;
      modeD = false;
      modeL = false;
      modeR = false;
    }

    if (joy_data->btn_d) {
      modeA = false;
      modeB = false;
      modeU = false;
      modeD = true;
      modeL = false;
      modeR = false;
    }

    if (joy_data->btn_l) {
      modeA = false;
      modeB = false;
      modeU = false;
      modeD = false;
      modeL = true;
      modeR = false;
      driving = true;
      start_left_pos = rhi_ptr_->getMotorPosition("left_motor");
      start_right_pos = rhi_ptr_->getMotorPosition("right_motor");
    }

    if (joy_data->btn_r) {
      modeA = false;
      modeB = false;
      modeU = false;
      modeD = false;
      modeL = false;
      modeR = true;
      driving = true;
      start_left_pos = rhi_ptr_->getMotorPosition("left_motor");
      start_right_pos = rhi_ptr_->getMotorPosition("right_motor");
    }

    double left_wheel_power;
    double right_wheel_power;

    if (modeA) {
      left_wheel_power = joy_data->left_y / 127.0;
      right_wheel_power = joy_data->right_y / 127.0;
    }


    if (modeB) {
      double forward_vel = joy_data->left_y / 127.0;
      double angular_vel = joy_data->right_x / 127.0;


      double threshold = 0.05;
      forward_vel = (std::fabs(forward_vel) < threshold) ? 0.0 : forward_vel;
      angular_vel = (std::fabs(angular_vel) < threshold) ? 0.0 : angular_vel;


      left_wheel_power = forward_vel + angular_vel;
      right_wheel_power = forward_vel - angular_vel;


      if (left_wheel_power > 1.0) {
        left_wheel_power = 1.0;
      } else if (left_wheel_power < -1.0) {
        left_wheel_power = -1.0;
      }

      if (right_wheel_power > 1.0) {
        right_wheel_power = 1.0;

      } else if (right_wheel_power < -1.0) {
        right_wheel_power = -1.0;
      }

      // double temp = right_wheel_power;
      // right_wheel_power = left_wheel_power;
      // left_wheel_power = temp;
    }

    if (modeU) {

    }

    if (modeD) {

    }

    if (modeL && driving) {
      double turn_distance_in = (M_PI * wheelDist) * (turnDeg / 360.0); 
      double target_deg = turn_distance_in * inchToDeg;

      double current_left = rhi_ptr_->getMotorPosition("left_motor");
      double current_right = rhi_ptr_->getMotorPosition("right_motor");

      double delta_left = std::abs(current_left - start_left_pos);
      double delta_right = std::abs(current_right - start_right_pos);

      double error_left = target_deg - delta_left;
      double error_right = target_deg - delta_right;

      double derivative_left = error_left - prev_error_left;
      double derivative_right = error_right - prev_error_right;

      double kP = 0.003; // Temporary value, should change
      double kD = 0.003; // Temporary value, should change

      left_wheel_power = -(kP * error_left + kD * derivative_left);
      right_wheel_power = (kP * error_right + kD * derivative_right);

      if (left_wheel_power > 0.5) {
        left_wheel_power = 0.5;
      } else if (left_wheel_power < -0.5) {
        left_wheel_power = -0.5;
      }
      
      if (right_wheel_power > 0.5) {
        right_wheel_power = 0.5;
      } else if (right_wheel_power < -0.5) {
        right_wheel_power = -0.5;
      }

      prev_error_left = error_left;
      prev_error_right = error_right;

      // Chose random value, should change
      if (error_left < 3.0 && error_right < 3.0) {
          left_wheel_power = 0.0;
          right_wheel_power = 0.0;
          driving = false;
          modeL = false;

          prev_error_left = 0.0;
          prev_error_right = 0.0;
      }

        // double turn_distance_in = (M_PI * wheelDist) * (turnDeg/ 360.0); 
        // double target_deg = turn_distance_in * inchToDeg;
      
        // double current_left = rhi_ptr_->getMotorPosition("left_motor");
        // double current_right = rhi_ptr_->getMotorPosition("right_motor");
      
        // double delta_left = std::abs(start_left_pos - current_left);
        // double delta_right = std::abs(current_right - start_right_pos);
      
        // if (delta_left < target_deg && delta_right < target_deg) {
        //   left_wheel_power = -0.5;
        //   right_wheel_power = 0.5;
        // } else {
        //   left_wheel_power = 0.0;
        //   right_wheel_power = 0.0;
        //   driving = false;
        //   modeL = false;
        // }
    }


    if (modeR && driving) {
      double turn_distance_in = (M_PI * wheelDist) * (turnDeg / 360.0); 
      double target_deg = turn_distance_in * inchToDeg;

      double current_left = rhi_ptr_->getMotorPosition("left_motor");
      double current_right = rhi_ptr_->getMotorPosition("right_motor");

      double delta_left = std::abs(current_left - start_left_pos);
      double delta_right = std::abs(current_right - start_right_pos);

      double error_left = target_deg - delta_left;
      double error_right = target_deg - delta_right;

      double derivative_left = error_left - prev_error_left;
      double derivative_right = error_right - prev_error_right;

      double kP = 0.003; // Temporary value, should change
      double kD = 0.003; // Temporary value, should change

      left_wheel_power = (kP * error_left + kD * derivative_left);
      right_wheel_power = -(kP * error_right + kD * derivative_right);

      if (left_wheel_power > 0.5) {
        left_wheel_power = 0.5;
      } else if (left_wheel_power < -0.5) {
        left_wheel_power = -0.5;
      }
      
      if (right_wheel_power > 0.5) {
        right_wheel_power = 0.5;
      } else if (right_wheel_power < -0.5) {
        right_wheel_power = -0.5;
      }

      prev_error_left = error_left;
      prev_error_right = error_right;

      // Chose random value, should change
      if (error_left < 3.0 && error_right < 3.0) {
          left_wheel_power = 0.0;
          right_wheel_power = 0.0;
          driving = false;
          modeL = false;

          prev_error_left = 0.0;
          prev_error_right = 0.0;
      }
        // double turn_distance_in = (M_PI * wheelDist) * (turnDeg/ 360.0); 
        // double target_deg = turn_distance_in * inchToDeg;
      
        // double current_left = rhi_ptr_->getMotorPosition("left_motor");
        // double current_right = rhi_ptr_->getMotorPosition("right_motor");
      
        // double delta_left = std::abs(start_left_pos - current_left);
        // double delta_right = std::abs(current_right - start_right_pos);
      
        // if (delta_left < target_deg && delta_right < target_deg) {
        //   left_wheel_power = 0.5;
        //   right_wheel_power = -0.5;
        // } else {
        //   left_wheel_power = 0.0;
        //   right_wheel_power = 0.0;
        //   driving = false;
        //   modeR = false;
        // }
    }


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
