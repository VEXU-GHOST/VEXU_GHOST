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
#include <ghost_tank/pinky_plugin.hpp>
#include <ghost_util/angle_util.hpp>
#include <ghost_util/math_util.hpp>
#include <ghost_util/unit_conversion_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <ghost_util/read_path.hpp>
#include <ghost_control/models/v5_current_limiting.hpp>

using ghost_planners::RobotTrajectory;
using ghost_ros_interfaces::msg_helpers::fromROSMsg;
using std::placeholders::_1;
using namespace std::chrono_literals;

using ghost_v5_interfaces::devices::JoystickDeviceData;

using ghost_util::INCHES_TO_METERS;

using JoyPtr = std::shared_ptr<ghost_v5_interfaces::devices::JoystickDeviceData>;

namespace ghost_tank
{

PinkyPlugin::PinkyPlugin()
{
  std::cout << "PinkyPlugin::PinkyPlugin" << std::endl;
  populateMotorNames();
  populateDigitalIONames();
}

void PinkyPlugin::populateMotorNames()
{
  m_right_drive_motor_names = {
    "drive_r1",
    "drive_r2",
    "drive_r3",
    "drive_r4",
    "drive_r5",
    "drive_r6",
    "drive_r7",
    // "drive_r8",
  };
  m_left_drive_motor_names = {
    "drive_l1",
    "drive_l2",
    "drive_l3",
    "drive_l4",
    "drive_l5",
    "drive_l6",
    "drive_l7",
    // "drive_l8",
  };

  m_all_drive_motor_names.insert(
    m_all_drive_motor_names.end(),
    m_left_drive_motor_names.begin(),
    m_left_drive_motor_names.end());

  m_all_drive_motor_names.insert(
    m_all_drive_motor_names.end(),
    m_right_drive_motor_names.begin(),
    m_right_drive_motor_names.end());
}

void PinkyPlugin::populateDigitalIONames()
{
  digital_io_port_map["sorter"] = 0;
  digital_io_port_map["descorer"] = 1;
  digital_io_port_map["switcher"] = 2;
  digital_io_port_map["left_blocker"] = 3;
  digital_io_port_map["right_blocker"] = 4;
  digital_io_port_map["little_will"] = 7;

  // climb / shooter / goal_rush solenoids do not physically exist on pinky,
  // but the autonomous() pneumatics block still references them. Without explicit
  // entries, operator[] would default-insert them at port 0 and clobber the sorter
  // every loop. Park them on bit 5 (F), an output with no mechanism assigned, so
  // their (always-false) writes are a harmless no-op and port 0 stays the sorter's.
  digital_io_port_map["bite"] = 5;
  digital_io_port_map["clamp"] = 5;
  digital_io_port_map["climb"] = 5;
  digital_io_port_map["shooter"] = 5;
  digital_io_port_map["goal_rush_l"] = 5;
  digital_io_port_map["goal_rush_r"] = 5;
}

//////////////////////
/// Initialization ///
//////////////////////

void PinkyPlugin::initialize()
{
  std::cout << "PinkyPlugin::initialize" << std::endl;
  TankRobotPlugin::initialize();
  // TankRobotPlugin::initROSComms();
  // TankRobotPlugin::initEstimation();
  // TankRobotPlugin::initIntake();
  // TankRobotPlugin::initTankModel();
  // TankRobotPlugin::initAutonomy();
  // TankRobotPlugin::resetWorldPose();
}

void PinkyPlugin::disabled()
{
}

void PinkyPlugin::autonomous(double current_time)
{
  if (m_is_first_auton_loop) {
    // m_is_first_auton_loop = false;
    playTTS("starting autonomous");
    // m_odom_ptr->resetPose();
    // resetWorldPose();

    bt_->set_variable<bool>("clamp_closed", false);
    bt_->set_variable<bool>("bite_closed", false);
    bt_->set_variable<bool>("goal_rush_l_down", false);
    bt_->set_variable<bool>("goal_rush_r_down", false);
    bt_->set_variable<bool>("goal_rush_down", false);
    bt_->set_variable<bool>("climb_extended", false);
    bt_->set_variable<bool>("conveyor_active", false);
    bt_->set_variable<bool>("store_ring", false);
    bt_->set_variable<bool>("ring_detector_active", false);
    bt_->set_variable<bool>("shoot", false);
    bt_->set_variable<bool>("sorter_active", false);
    bt_->set_variable<bool>("switcher_active", false);
    bt_->set_variable<bool>("left_blocker_active", false);
    bt_->set_variable<bool>("right_blocker_active", false);
    bt_->set_variable<bool>("little_will_active", false);
  }

  TankRobotPlugin::autonomous(current_time);

  // Update Pneumatics
  rhi_ptr_->setDigitalOut(digital_io_port_map["climb"], (bt_->get_variable<bool>("climb_extended")));
  rhi_ptr_->setDigitalOut(digital_io_port_map["clamp"], (bt_->get_variable<bool>("clamp_closed")));
  rhi_ptr_->setDigitalOut(digital_io_port_map["shooter"], (bt_->get_variable<bool>("shoot")));
  if (m_mirrored) {
    rhi_ptr_->setDigitalOut(digital_io_port_map["goal_rush_l"], bt_->get_variable<int>("goal_rush_r_down") || bt_->get_variable<int>("goal_rush_down"));
    rhi_ptr_->setDigitalOut(digital_io_port_map["goal_rush_r"], bt_->get_variable<int>("goal_rush_l_down"));
  } else {
    rhi_ptr_->setDigitalOut(digital_io_port_map["goal_rush_l"], (bt_->get_variable<int>("goal_rush_l_down")));
    rhi_ptr_->setDigitalOut(digital_io_port_map["goal_rush_r"], (bt_->get_variable<int>("goal_rush_r_down") || bt_->get_variable<int>("goal_rush_down")));
  }
  rhi_ptr_->setDigitalOut(digital_io_port_map["bite"], m_bite_closed);
  rhi_ptr_->setDigitalOut(digital_io_port_map["sorter"], bt_->get_variable<bool>("sorter_active"));
  rhi_ptr_->setDigitalOut(digital_io_port_map["switcher"], bt_->get_variable<bool>("switcher_active"));
  rhi_ptr_->setDigitalOut(digital_io_port_map["left_blocker"], bt_->get_variable<bool>("left_blocker_active"));
  rhi_ptr_->setDigitalOut(digital_io_port_map["right_blocker"], bt_->get_variable<bool>("right_blocker_active"));
  rhi_ptr_->setDigitalOut(digital_io_port_map["little_will"], bt_->get_variable<bool>("little_will_active"));
}

void PinkyPlugin::teleop(double current_time)
{
  auto joy_data = rhi_ptr_->getMainJoystickData();
  bool shift_r = joy_data->btn_b;
  bool shift_l = joy_data->btn_d;

  TankRobotPlugin::teleop(current_time);

  updateScissor(joy_data->btn_l1, joy_data->btn_l2, shift_r);
  updateT1Climb(joy_data->btn_r1, joy_data->btn_r2, shift_r);
}

void PinkyPlugin::updateT1Climb(bool up, bool down, bool enabled)
{
  return;
  if (enabled) {
    if (up) {
      rhi_ptr_->setDigitalOut(digital_io_port_map["climb"], true);
    } else if (down) {
      rhi_ptr_->setDigitalOut(digital_io_port_map["climb"], false);
    }
  }
}


void PinkyPlugin::updateScissor(bool up, bool down, bool enabled)
{
  return;
  rhi_ptr_->setDigitalOut(digital_io_port_map["shooter"], false);
  if (enabled) {
    if (up && down) {
      rhi_ptr_->setDigitalOut(digital_io_port_map["shooter"], true);
      rhi_ptr_->setMotorCurrentLimitMilliAmps("scissor_motor", 0);
      rhi_ptr_->setMotorVoltageCommandPercent("scissor_motor", 0.0);
    } else if (up) {
      rhi_ptr_->setMotorCurrentLimitMilliAmps("scissor_motor", 2500);
      rhi_ptr_->setMotorVoltageCommandPercent("scissor_motor", 1.0);
    } else if (down) {
      rhi_ptr_->setMotorCurrentLimitMilliAmps("scissor_motor", 2500);
      rhi_ptr_->setMotorVoltageCommandPercent("scissor_motor", -1.0);
    } else {
      rhi_ptr_->setMotorCurrentLimitMilliAmps("scissor_motor", 0);
      rhi_ptr_->setMotorVoltageCommandPercent("scissor_motor", 0.0);
    }
  } else {
    rhi_ptr_->setMotorCurrentLimitMilliAmps("scissor_motor", 0);
    rhi_ptr_->setMotorVoltageCommandPercent("scissor_motor", 0.0);
  }
}

} // namespace ghost_tank

PLUGINLIB_EXPORT_CLASS(ghost_tank::PinkyPlugin, ghost_ros_interfaces::V5RobotBase)
