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
#include <ghost_tank/inky_plugin.hpp>
#include <ghost_util/angle_util.hpp>
#include <ghost_util/math_util.hpp>
#include <ghost_util/unit_conversion_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <ghost_util/read_path.hpp>
#include <ghost_control/models/v5_current_limiting.hpp>

using ghost_planners::RobotTrajectory;
using ghost_ros_interfaces::msg_helpers::fromROSMsg;
using std::placeholders::_1;

using ghost_v5_interfaces::devices::JoystickDeviceData;

using ghost_util::INCHES_TO_METERS;

using JoyPtr = std::shared_ptr<ghost_v5_interfaces::devices::JoystickDeviceData>;

namespace ghost_tank
{

InkyPlugin::InkyPlugin()
{
  populateMotorNames();
  populateDigitalIONames();
}

void InkyPlugin::populateMotorNames()
{
  m_right_drive_motor_names = {
    // "drive_r1",
    "drive_r2",
    "drive_r3",
    "drive_r4",
    "drive_r5",
    "drive_r6",
    "drive_r7",
  };
  m_left_drive_motor_names = {
    "drive_l1",
    "drive_l2",
    "drive_l3",
    "drive_l4",
    // "drive_l5",
    "drive_l6",
    "drive_l7",
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

void InkyPlugin::populateDigitalIONames()
{
  // digital_io_port_map["goal_rush_sensor"] = 4;
  digital_io_port_map["goal_rush_l"] = 0;
  digital_io_port_map["climb"] = 1;
  digital_io_port_map["goal_rush_r"] = 2;
  digital_io_port_map["bite"] = 3;
  digital_io_port_map["clamp"] = 4;
  digital_io_port_map["buddy"] = 5;
}

//////////////////////
/// Initialization ///
//////////////////////

void InkyPlugin::initialize()
{
  TankRobotPlugin::initROSComms();
  TankRobotPlugin::initEstimation();
  TankRobotPlugin::initIntake();
  initNeutralStakeArm();
  TankRobotPlugin::initTankModel();
  TankRobotPlugin::initAutonomy();
  TankRobotPlugin::resetWorldPose();
}

void InkyPlugin::initNeutralStakeArm()
{
  std::cout << "[InkyPlugin::initNeutralStakeArm]" << std::endl;

  node_ptr_->declare_parameter("tank_robot_plugin.neutral_stake_arm_kp", 0.0);
  node_ptr_->declare_parameter("tank_robot_plugin.neutral_stake_arm_gear_ratio", 0.0);
  node_ptr_->declare_parameter("tank_robot_plugin.neutral_stake_arm_rest_pos_deg", 0.0);
  node_ptr_->declare_parameter("tank_robot_plugin.neutral_stake_arm_loading_pos_deg", 0.0);
  node_ptr_->declare_parameter("tank_robot_plugin.neutral_stake_arm_loaded_pos_deg", 0.0);
  node_ptr_->declare_parameter("tank_robot_plugin.neutral_stake_arm_score_neutral_pos_deg", 0.0);
  node_ptr_->declare_parameter("tank_robot_plugin.neutral_stake_arm_score_alliance_pos_deg", 0.0);
  node_ptr_->declare_parameter("tank_robot_plugin.neutral_stake_arm_down_pos_deg", 0.0);
  node_ptr_->declare_parameter("tank_robot_plugin.neutral_stake_arm_settled_threshold_deg", 0.0);

  m_neutral_stake_arm_kp = node_ptr_->get_parameter("tank_robot_plugin.neutral_stake_arm_kp").as_double();
  m_neutral_stake_arm_gear_ratio = node_ptr_->get_parameter("tank_robot_plugin.neutral_stake_arm_gear_ratio").as_double();
  m_neutral_stake_arm_rest_pos_deg = node_ptr_->get_parameter("tank_robot_plugin.neutral_stake_arm_rest_pos_deg").as_double();
  m_neutral_stake_arm_loading_pos_deg = node_ptr_->get_parameter("tank_robot_plugin.neutral_stake_arm_loading_pos_deg").as_double();
  m_neutral_stake_arm_loaded_pos_deg = node_ptr_->get_parameter("tank_robot_plugin.neutral_stake_arm_loaded_pos_deg").as_double();
  m_neutral_stake_arm_score_neutral_pos_deg = node_ptr_->get_parameter("tank_robot_plugin.neutral_stake_arm_score_neutral_pos_deg").as_double();
  m_neutral_stake_arm_score_alliance_pos_deg = node_ptr_->get_parameter("tank_robot_plugin.neutral_stake_arm_score_alliance_pos_deg").as_double();
  m_neutral_stake_arm_down_pos_deg = node_ptr_->get_parameter("tank_robot_plugin.neutral_stake_arm_down_pos_deg").as_double();
  m_neutral_stake_arm_settled_threshold_deg = node_ptr_->get_parameter("tank_robot_plugin.neutral_stake_arm_settled_threshold_deg").as_double();

  m_neutral_stake_arm_des_pos = m_neutral_stake_arm_rest_pos_deg;
}

void InkyPlugin::disabled()
{
}

void InkyPlugin::autonomous(double current_time)
{
  if (m_is_first_auton_loop) {
    // m_is_first_auton_loop = false;
    playTTS("starting autonomous");
    m_odom_ptr->resetPose();
    // resetWorldPose();

    bt_->set_variable<bool>("clamp_closed", false);
    bt_->set_variable<bool>("bite_closed", false);
    bt_->set_variable<bool>("goal_rush_down", false);
    bt_->set_variable<bool>("goal_rush_l_down", false);
    bt_->set_variable<bool>("goal_rush_r_down", false);
    bt_->set_variable<bool>("conveyor_active", false);
    bt_->set_variable<bool>("store_ring", false);
    bt_->set_variable<bool>("ring_detector_active", false);
    bt_->set_variable<bool>("neutral_stake_settled", false);
  }

  TankRobotPlugin::autonomous(current_time);

  int neutral_stake_pos = 0;
  if (bt_->get_variable<int>("neutral_stake_pos", neutral_stake_pos)) {
    bool is_settled = updateNeutralStakeArmPosition(neutral_stake_pos);
    bt_->set_variable<bool>("neutral_stake_settled", is_settled);
  }

  // Update Pneumatics
  rhi_ptr_->setDigitalOut(digital_io_port_map["clamp"], bt_->get_variable<int>("clamp_closed"));
  if (m_mirrored) {
    rhi_ptr_->setDigitalOut(digital_io_port_map["goal_rush_l"], bt_->get_variable<int>("goal_rush_r_down") || bt_->get_variable<int>("goal_rush_down"));
    rhi_ptr_->setDigitalOut(digital_io_port_map["goal_rush_r"], bt_->get_variable<int>("goal_rush_l_down"));
  } else {
    rhi_ptr_->setDigitalOut(digital_io_port_map["goal_rush_l"], bt_->get_variable<int>("goal_rush_l_down"));
    rhi_ptr_->setDigitalOut(digital_io_port_map["goal_rush_r"], bt_->get_variable<int>("goal_rush_r_down") || bt_->get_variable<int>("goal_rush_down"));
  }
  rhi_ptr_->setDigitalOut(digital_io_port_map["bite"], bt_->get_variable<int>("bite_closed"));
}

void InkyPlugin::teleop(double current_time)
{
  auto joy_data = rhi_ptr_->getMainJoystickData();
  bool shift1 = joy_data->btn_b;
  bool shift2 = joy_data->btn_d;

  TankRobotPlugin::teleop(current_time);
  if (m_running_auton) {
    return;
  }

  updateNeutralStakeArmController(joy_data->btn_l1, joy_data->btn_l2, shift1); // Y-held mode

  if (joy_data->btn_r && joy_data->btn_y && !m_buddy_pressed) {
    m_buddy_pressed = true;
    m_buddy_extended = !m_buddy_extended;
  } else if (!joy_data->btn_r && !joy_data->btn_y) {m_buddy_pressed = false;}
  rhi_ptr_->setDigitalOut(digital_io_port_map["buddy"], m_buddy_extended);
}

bool InkyPlugin::updateNeutralStakeArmPosition(int arm_mode)
{
  // Neutral stake arm motors not present on this robot
  (void)arm_mode;
  return true;
}

void InkyPlugin::updateNeutralStakeArmController(bool up_btn, bool down_btn, bool active)
{
  // Neutral stake arm motors not present on this robot
  (void)up_btn;
  (void)down_btn;
  (void)active;
}

} // namespace ghost_tank

PLUGINLIB_EXPORT_CLASS(ghost_tank::InkyPlugin, ghost_ros_interfaces::V5RobotBase)
