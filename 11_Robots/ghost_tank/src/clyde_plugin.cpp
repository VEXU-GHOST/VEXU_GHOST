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
#include <ghost_tank/clyde_plugin.hpp>
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

ClydePlugin::ClydePlugin()
{
  std::cout << "ClydePlugin::ClydePlugin" << std::endl;
  populateMotorNames();
  populateDigitalIONames();
}

void ClydePlugin::populateMotorNames()
{
  m_right_drive_motor_names = {
    "drive_r1",
    "drive_r2",  // port reused by lift/arm
    "drive_r3",
    "drive_r4",  // port reused by lift/arm
    "drive_r5",
    "drive_r6",  // port reused by lift/arm
    // "drive_r7",
    // "drive_r8",
  };
  m_left_drive_motor_names = {
    "drive_l1",
    "drive_l2",
    "drive_l3",
    "drive_l4",
    "drive_l5",
    "drive_l6",  // port reused by lift/arm
    // "drive_l7",
    // "drive_l8",
  };

  // The TankRobotPlugin constructor already filled this with the base motor list
  // (including motors that don't exist on clyde), so start fresh.
  m_all_drive_motor_names.clear();
  m_all_drive_motor_names.insert(
    m_all_drive_motor_names.end(),
    m_left_drive_motor_names.begin(),
    m_left_drive_motor_names.end());

  m_all_drive_motor_names.insert(
    m_all_drive_motor_names.end(),
    m_right_drive_motor_names.begin(),
    m_right_drive_motor_names.end());
}

void ClydePlugin::populateDigitalIONames()
{
  // No sorter on clyde; parked on F so it doesn't fight the clamp on port A.
  digital_io_port_map["sorter"] = 5;
  digital_io_port_map["descorer"] = 1;
  digital_io_port_map["switcher"] = 2;
  digital_io_port_map["left_blocker"] = 3;
  digital_io_port_map["right_blocker"] = 4;
  digital_io_port_map["little_will"] = 7;

  // climb / shooter / goal_rush solenoids do not physically exist on clyde,
  // but the autonomous() pneumatics block still references them. Without explicit
  // entries, operator[] would default-insert them at port 0 and clobber the sorter
  // every loop. Park them on bit 5 (F), an output with no mechanism assigned, so
  // their (always-false) writes are a harmless no-op and port 0 stays the sorter's.
  digital_io_port_map["bite"] = 5;
  digital_io_port_map["clamp"] = 0;  // port A
  // TankRobotPlugin::updateColorSwitcher writes "color_sorter" every teleop loop
  // and defaults to port 0, which would fight the clamp. Park it on F.
  digital_io_port_map["color_sorter"] = 5;
  digital_io_port_map["climb"] = 5;
  digital_io_port_map["shooter"] = 5;
  digital_io_port_map["goal_rush_l"] = 5;
  digital_io_port_map["goal_rush_r"] = 5;
}

//////////////////////
/// Initialization ///
//////////////////////

void ClydePlugin::initialize()
{
  std::cout << "ClydePlugin::initialize" << std::endl;
  TankRobotPlugin::initialize();

  node_ptr_->declare_parameter("tank_robot_plugin.auto_claw_topic", "/sensors/distance/auto_claw");
  node_ptr_->declare_parameter("tank_robot_plugin.auto_claw_close_mm", m_auto_claw_close_mm);
  std::string auto_claw_topic = node_ptr_->get_parameter("tank_robot_plugin.auto_claw_topic").as_string();
  m_auto_claw_close_mm = node_ptr_->get_parameter("tank_robot_plugin.auto_claw_close_mm").as_double();
  m_auto_claw_sub = node_ptr_->create_subscription<ghost_msgs::msg::DistanceSensorState>(
    auto_claw_topic, rclcpp::SensorDataQoS(), std::bind(&ClydePlugin::autoClawCallback, this, _1));

  // TankRobotPlugin::initROSComms();
  // TankRobotPlugin::initEstimation();
  // TankRobotPlugin::initIntake();
  // TankRobotPlugin::initTankModel();
  // TankRobotPlugin::initAutonomy();
  // TankRobotPlugin::resetWorldPose();
}

void ClydePlugin::disabled()
{
  // Don't let an L1 hold from before the disable close the clamp on re-enable
  m_clamp_grabbing = false;
}

void ClydePlugin::autonomous(double current_time)
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
  m_clamp_grabbing = false;

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

void ClydePlugin::teleop(double current_time)
{
  auto joy_data = rhi_ptr_->getMainJoystickData();
  bool shift_r = joy_data->btn_b;
  bool shift_l = joy_data->btn_d;

  TankRobotPlugin::teleop(current_time);

  updateScissor(joy_data->btn_l1, joy_data->btn_l2, shift_r);
  updateT1Climb(joy_data->btn_r1, joy_data->btn_r2, shift_r);
  updateClamp(joy_data->btn_l1, current_time);

  // D-pad down: send the lift to 800 deg. Hands the target to the teleop hold
  // logic, so R1/R2 still override it.
  static bool last_d_state = false;
  if (joy_data->btn_d && !last_d_state) {
    m_lift_hold_deg = 800.0;
    m_lift_hold_max_power = 1.0;
    m_lift_holding = true;
    m_lift_controller_ptr->reset();
    // Arm back to its zero position (arm_stage_positions_deg[0] = 0.0)
    m_arm_stage = 0;
    m_arm_custom_active = false;
    m_arm_max_power = 1.0;
    m_arm_controller_ptr->reset();
  }
  last_d_state = joy_data->btn_d;
}

void ClydePlugin::updateClamp(bool l1_pressed, double current_time)
{
  const double drop_deg = 700.0;
  const double min_deg = 850.0;
  const double open_delay_s = 0.25;
  const double closed_arm_deg = 200.0;

  const double clamp_arm_max_power = 0.5;  // half speed for the clamp arm moves

  // On close (L1 or auto-close): raise the arm to closed_arm_deg. L2 / D-pad
  // down still take the arm back to its stages. Stage reset to 0 so the next L2
  // goes to stage 1.
  auto raise_arm = [&]() {
    m_arm_stage = 0;
    m_arm_custom_deg = closed_arm_deg;
    m_arm_custom_active = true;
    m_arm_max_power = clamp_arm_max_power;
    m_arm_controller_ptr->reset();
  };

  auto close_clamp = [&]() {
    m_clamp_closed = true;
    m_clamp_open_time = -1.0;
    m_clamp_grabbing = false;
    raise_arm();
  };

  // L1 press with the clamp closed opens it. L1 press with the clamp open starts a
  // grab: while L1 is held the auto_claw sensor can close the clamp, and if it
  // hasn't by the time L1 is released, the clamp closes on release.
  if (l1_pressed && !m_clamp_btn_pressed) {
    if (!m_clamp_closed) {
      m_clamp_grabbing = true;
      std::cout << "[ClydePlugin::updateClamp] L1 pressed, grabbing" << std::endl;
    } else {
      m_clamp_closed = false;
      m_clamp_open_time = -1.0;
      std::cout << "[ClydePlugin::updateClamp] L1 pressed, opening (port "
                << digital_io_port_map["clamp"] << ")" << std::endl;

      // On open: drop the lift 700 deg, but never below 400 deg, and delay the clamp
      // opening so the lift starts moving first. Hands the target to the teleop hold
      // logic, so R1/R2 still override it. At/below 400 deg: no drop, open immediately.
      // Arm back to 0 (arm_stage_positions_deg[0]) unless it's up on a later stage.
      bool arm_up = !m_arm_custom_active && m_arm_stage > 0;
      if (!arm_up) {
        m_arm_stage = 0;
        m_arm_custom_active = false;
        m_arm_max_power = clamp_arm_max_power;
        m_arm_controller_ptr->reset();
      }

      double current_deg = getLiftPositionDeg();
      if (current_deg > min_deg) {
        m_lift_hold_deg = std::max(current_deg - drop_deg, min_deg);
        m_lift_hold_max_power = 0.5;  // half speed on the way down
        m_lift_holding = true;
        m_lift_controller_ptr->reset();
        m_clamp_open_time = current_time + open_delay_s;
      }
    }
  }
  m_clamp_btn_pressed = l1_pressed;

  if (m_clamp_grabbing) {
    // Auto-close from the auto_claw distance sensor. Readings older than 200 ms
    // (sensor unplugged / host down) are ignored.
    bool fresh = std::chrono::steady_clock::now() - m_auto_claw_msg_time < 200ms;
    if (l1_pressed && fresh && m_auto_claw_valid && m_auto_claw_distance_mm < m_auto_claw_close_mm) {
      close_clamp();
      std::cout << "[ClydePlugin::updateClamp] auto-close at " << m_auto_claw_distance_mm << " mm" << std::endl;
    } else if (!l1_pressed) {
      close_clamp();
      std::cout << "[ClydePlugin::updateClamp] L1 released, closing" << std::endl;
    }
  }

  // Stay closed until the open delay has elapsed
  bool clamp_output = m_clamp_closed ||
    (m_clamp_open_time >= 0.0 && current_time < m_clamp_open_time);
  rhi_ptr_->setDigitalOut(digital_io_port_map["clamp"], clamp_output);
}

void ClydePlugin::updateT1Climb(bool up, bool down, bool enabled)
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


void ClydePlugin::updateScissor(bool up, bool down, bool enabled)
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

PLUGINLIB_EXPORT_CLASS(ghost_tank::ClydePlugin, ghost_ros_interfaces::V5RobotBase)
