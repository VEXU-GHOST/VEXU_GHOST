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

  // Clamped low arm position comes from arm_stage_positions_deg[2] so the teleop
  // clamp macro and ArmPosCmd stage="2" stay in sync.
  if (m_arm_stage_positions_deg.size() > 2) {
    m_arm_clamped_low_deg = m_arm_stage_positions_deg[2];
  }

  node_ptr_->declare_parameter("tank_robot_plugin.arm_macro_extend_deg", m_arm_macro_extend_deg);
  m_arm_macro_extend_deg = node_ptr_->get_parameter("tank_robot_plugin.arm_macro_extend_deg").as_double();
  node_ptr_->declare_parameter("tank_robot_plugin.arm_macro_revert_deg", m_arm_macro_revert_deg);
  m_arm_macro_revert_deg = node_ptr_->get_parameter("tank_robot_plugin.arm_macro_revert_deg").as_double();

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
    bt_->set_variable<int>("matchload_cmd", -1);
    bt_->set_variable<bool>("matchload_auto_close", true);
    bt_->set_variable<double>("matchload_timeout_s", 0.0);
    m_matchload_grabbing = false;
    m_matchload_arm_active = false;
    m_matchload_lift_active = false;
  }

  TankRobotPlugin::autonomous(current_time);
  updateMatchloadAuton();
  m_clamp_grabbing = false;
  // Mirror the BT clamp state into the teleop state so the clamp stays where
  // auton left it when driver control takes over.
  m_clamp_closed = bt_->get_variable<bool>("clamp_closed");
  m_clamp_open_time = -1.0;

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

void ClydePlugin::updateMatchloadAuton()
{
  // Requests from the MatchloadCmd BT node: 1 = on, 0 = off, -1 = none.
  // MatchloadCmd has already opened the clamp and released the arm and lift
  // (arm_target_stage = lift_target_stage = -1) on the blackboard, so the macro
  // owns both until an ArmPosCmd / LiftPosCmd posts a stage.
  int matchload_cmd = -1;
  bt_->get_variable<int>("matchload_cmd", matchload_cmd);
  if (matchload_cmd >= 0) {
    bt_->set_variable<int>("matchload_cmd", -1);
  }

  if (matchload_cmd == 1) {
    m_matchload_grabbing = true;
    m_matchload_start_time = std::chrono::steady_clock::now();
    m_matchload_arm_active = true;
    m_arm_macro_extended = true;
    m_arm_custom_deg = m_arm_macro_extend_deg;
    m_arm_max_power = 1.0;
    m_arm_controller_ptr->reset();

    // Lift to its zero position, slow on the way down like LiftPosCmd
    m_matchload_lift_active = true;
    m_lift_hold_deg = m_lift_stage_positions_deg[0];
    m_lift_hold_max_power =
      (m_lift_hold_deg < getLiftPositionDeg()) ? m_lift_down_max_power : 1.0;
    m_lift_controller_ptr->reset();
  }

  // ArmPosCmd / LiftPosCmd (stage >= 0) take over from the macro. Checked before
  // the revert below so a LiftPosCmd / ArmPosCmd ticked on the same loop as the
  // clamp close (e.g. ClampCmd -> LiftPosCmd -> ArmPosCmd) wins over the revert.
  int arm_target_stage = -1;
  bt_->get_variable<int>("arm_target_stage", arm_target_stage);
  if (arm_target_stage >= 0) {
    m_matchload_arm_active = false;
  }
  int lift_target_stage = -1;
  bt_->get_variable<int>("lift_target_stage", lift_target_stage);
  if (lift_target_stage >= 0) {
    m_matchload_lift_active = false;
  }

  bool clamp_closed = bt_->get_variable<bool>("clamp_closed");

  // While on, the auto_claw sensor closes the clamp (teleop: L1 held); toggling
  // off closes it if it is still open (teleop: L1 released). A nonzero
  // matchload_timeout_s also closes it that long after the grab started, in the
  // background while the tree keeps running.
  if (m_matchload_grabbing && !clamp_closed) {
    bool auto_close = true;
    bt_->get_variable<bool>("matchload_auto_close", auto_close);
    double timeout_s = 0.0;
    bt_->get_variable<double>("matchload_timeout_s", timeout_s);
    auto now = std::chrono::steady_clock::now();
    bool fresh = now - m_auto_claw_msg_time < 200ms;
    bool sensor_close = auto_close && fresh && m_auto_claw_valid &&
      m_auto_claw_distance_mm < m_auto_claw_close_mm;
    bool timed_out = timeout_s > 0.0 &&
      std::chrono::duration<double>(now - m_matchload_start_time).count() > timeout_s;
    if (timed_out) {
      std::cout << "[MatchloadCmd] timed out, closing" << std::endl;
    }
    if (sensor_close || timed_out || matchload_cmd == 0) {
      clamp_closed = true;
      bt_->set_variable<bool>("clamp_closed", true);
    }
  }
  if (clamp_closed || matchload_cmd == 0) {
    m_matchload_grabbing = false;
  }

  // Any clamp close with the arm still extended (sensor, toggle off, ClampCmd,
  // AutoClawCmd) reverts the arm and raises the lift, same as teleop updateClamp.
  // Each half only applies if the macro still owns that mechanism.
  if (clamp_closed && m_arm_macro_extended &&
    (m_matchload_arm_active || m_matchload_lift_active))
  {
    m_arm_macro_extended = false;
    if (m_matchload_arm_active) {
      m_arm_custom_deg = m_arm_macro_revert_deg;
      m_arm_max_power = m_arm_macro_revert_max_power;
      m_arm_controller_ptr->reset();
    }
    if (m_matchload_lift_active) {
      m_lift_hold_deg = getLiftPositionDeg() + m_lift_macro_raise_deg;
      m_lift_hold_max_power = 1.0;
      m_lift_controller_ptr->reset();
    }
  }

  if (m_matchload_arm_active) {
    updateArmToPosition(m_arm_custom_deg, m_arm_max_power);
  }
  if (m_matchload_lift_active) {
    updateLiftToPosition(m_lift_hold_deg, m_lift_hold_max_power);
  }
}

void ClydePlugin::teleop(double current_time)
{
  auto joy_data = rhi_ptr_->getMainJoystickData();
  bool shift_r = joy_data->btn_b;
  bool shift_l = joy_data->btn_d;

  TankRobotPlugin::teleop(current_time);
  // Driver-triggered auton ran this loop; don't let the teleop clamp/mech logic
  // overwrite what autonomous() just wrote.
  if (m_running_auton) {
    return;
  }

  updateScissor(joy_data->btn_l1, joy_data->btn_l2, shift_r);
  updateT1Climb(joy_data->btn_r1, joy_data->btn_r2, shift_r);

  // D-pad down: send the lift to its second stage (lift_stage_positions_deg[1]),
  // open the clamp, and send the arm to 0.
  // Hands the lift target to the teleop hold logic, so R1/R2 still override it.
  // Runs before updateClamp so the clamp opens this loop.
  static bool last_d_state = false;
  if (joy_data->btn_d && !last_d_state) {
    m_lift_hold_deg = m_lift_stage_positions_deg[1];
    m_lift_hold_max_power = 1.0;
    m_lift_holding = true;
    m_lift_controller_ptr->reset();
    // Open the clamp immediately (cancels any pending delayed open / grab)
    m_clamp_closed = false;
    m_clamp_open_time = -1.0;
    m_clamp_grabbing = false;
    // Arm back to 0 (stage 0 with the clamp open = arm_stage_positions_deg[0])
    m_arm_stage = 0;
    m_arm_custom_active = false;
    m_arm_max_power = 1.0;
    m_arm_controller_ptr->reset();
  }
  last_d_state = joy_data->btn_d;

  // D-pad right: send the lift to its zero position (lift_stage_positions_deg[0]),
  // open the clamp, and extend the arm to m_arm_macro_extend_deg. The arm holds
  // there until the clamp closes, then reverts to m_arm_macro_revert_deg while the
  // lift rises m_lift_macro_raise_deg (see updateClamp). R1/R2 and L2 still
  // override the lift / arm.
  // (btn_r also toggles auto-sort in TankRobotPlugin::teleop, which is unused on clyde.)
  static bool last_r_state = false;
  if (joy_data->btn_r && !last_r_state) {
    m_lift_hold_deg = m_lift_stage_positions_deg[0];
    m_lift_hold_max_power = 1.0;
    m_lift_holding = true;
    m_lift_controller_ptr->reset();
    // Open the clamp immediately (cancels any pending delayed open / grab)
    m_clamp_closed = false;
    m_clamp_open_time = -1.0;
    m_clamp_grabbing = false;
    m_arm_macro_extended = true;
    m_arm_custom_deg = m_arm_macro_extend_deg;
    m_arm_custom_active = true;
    m_arm_max_power = 1.0;
    m_arm_controller_ptr->reset();
  }
  last_r_state = joy_data->btn_r;

  updateClamp(joy_data->btn_l1, current_time);
}

void ClydePlugin::updateClamp(bool l1_pressed, double current_time)
{
  const double drop_deg = 700.0;
  const double min_deg = 750.0;
  const double open_delay_s = 0.25;

  const double clamp_arm_max_power = 0.5;  // half speed for the clamp arm moves

  // Closing doesn't move the arm; it goes to m_arm_clamped_low_deg once the lift
  // moves (see updateArmFromJoystick). The exception is the d-pad right macro:
  // if the arm is still extended from it, closing sends the arm to
  // m_arm_macro_revert_deg and holds it there (L2 / R1 / R2 / opening the clamp
  // take over from it), and raises the lift m_lift_macro_raise_deg via the
  // teleop hold logic, so R1/R2 still override it.
  auto close_clamp = [&]() {
    m_clamp_closed = true;
    m_clamp_open_time = -1.0;
    m_clamp_grabbing = false;
    if (m_arm_custom_active && m_arm_macro_extended) {
      m_arm_macro_extended = false;
      m_arm_custom_deg = m_arm_macro_revert_deg;
      m_arm_max_power = m_arm_macro_revert_max_power;
      m_arm_controller_ptr->reset();

      m_lift_hold_deg = getLiftPositionDeg() + m_lift_macro_raise_deg;
      m_lift_hold_max_power = 1.0;
      m_lift_holding = true;
      m_lift_controller_ptr->reset();
    }
  };

  // L1 press with the clamp closed opens it. L1 press with the clamp open starts a
  // grab: while L1 is held the auto_claw sensor can close the clamp, and if it
  // hasn't by the time L1 is released, the clamp closes on release.
  if (l1_pressed && !m_clamp_btn_pressed) {
    if (!m_clamp_closed) {
      m_clamp_grabbing = true;
      // std::cout << "[ClydePlugin::updateClamp] L1 pressed, grabbing" << std::endl;
    } else {
      m_clamp_closed = false;
      m_clamp_open_time = -1.0;
      // std::cout << "[ClydePlugin::updateClamp] L1 pressed, opening (port "
                // << digital_io_port_map["clamp"] << ")" << std::endl;

      // On open: drop the lift 700 deg, but never below 400 deg, and delay the clamp
      // opening so the lift starts moving first. Hands the target to the teleop hold
      // logic, so R1/R2 still override it. At/below 400 deg: no drop, open immediately.
      // If the arm is at its low position it drops from m_arm_clamped_low_deg back
      // to 0 (arm_stage_positions_deg[0]) at half speed; an arm that's up stays up.
      // An arm holding the macro's revert position counts as low and drops too.
      if (m_arm_custom_active && !m_arm_macro_extended) {
        m_arm_custom_active = false;
        m_arm_stage = 0;
      }
      if (m_arm_stage == 0) {
        m_arm_max_power = clamp_arm_max_power;
        m_arm_controller_ptr->reset();
      }

      double current_deg = getLiftPositionDeg();
      if (current_deg > min_deg) {
        m_lift_hold_deg = std::max(current_deg - drop_deg, min_deg);
        m_lift_hold_max_power = m_lift_down_max_power;  // slow on the way down
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
      // std::cout << "[ClydePlugin::updateClamp] auto-close at " << m_auto_claw_distance_mm << " mm" << std::endl;
    } else if (!l1_pressed) {
      close_clamp();
      // std::cout << "[ClydePlugin::updateClamp] L1 released, closing" << std::endl;
    }
  }

  // Stay closed until the open delay has elapsed
  bool clamp_output = m_clamp_closed ||
    (m_clamp_open_time >= 0.0 && current_time < m_clamp_open_time);
  rhi_ptr_->setDigitalOut(digital_io_port_map["clamp"], clamp_output);
}

void ClydePlugin::updateArmFromJoystick(JoyPtr joy_data)
{
  const double lift_arm_max_power = 0.5;  // half speed for the lift-triggered move

  // Each L2 press toggles the arm between low (stage 0) and up (stage 1, 1650 deg).
  static bool last_l2_state = false;
  if (joy_data->btn_l2 && !last_l2_state) {
    m_arm_stage = (m_arm_stage == 0) ? 1 : 0;
    m_arm_custom_active = false;
    m_arm_max_power = 1.0;
    m_arm_controller_ptr->reset();
  }
  last_l2_state = joy_data->btn_l2;

  // Opening the clamp drops the arm's low position back to 0
  if (!m_clamp_closed) {
    m_arm_clamped_low_active = false;
  }

  // When the lift starts moving (R1/R2) with the clamp closed, send the arm to its
  // low position (m_arm_clamped_low_deg). Edge-triggered so L2 still works mid-move.
  static bool last_lift_moving_clamped = false;
  bool lift_moving_clamped = m_clamp_closed && (joy_data->btn_r1 || joy_data->btn_r2);
  if (lift_moving_clamped && !last_lift_moving_clamped) {
    m_arm_clamped_low_active = true;
    m_arm_stage = 0;
    m_arm_custom_active = false;
    m_arm_max_power = lift_arm_max_power;
    m_arm_controller_ptr->reset();
  }
  last_lift_moving_clamped = lift_moving_clamped;

  // Low position is m_arm_clamped_low_deg once R1/R2 was pressed with the clamp
  // closed, otherwise 0 (closing the clamp alone doesn't move the arm). The d-pad
  // right macro's extension (m_arm_custom_active) overrides both.
  double low_deg = m_arm_clamped_low_active ? m_arm_clamped_low_deg : m_arm_stage_positions_deg[0];
  double target_deg = m_arm_custom_active ? m_arm_custom_deg :
    (m_arm_stage == 0) ? low_deg : m_arm_stage_positions_deg[1];
  updateArmToPosition(target_deg, m_arm_max_power);
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
