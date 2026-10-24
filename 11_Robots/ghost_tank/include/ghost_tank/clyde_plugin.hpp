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

#pragma once

#include <ghost_planners/robot_trajectory.hpp>
#include <ghost_ros_interfaces/competition/v5_robot_base.hpp>
#include <ghost_ros_interfaces/msg_helpers/msg_helpers.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <ghost_msgs/msg/distance_sensor_state.hpp>
#include <ghost_msgs/msg/drivetrain_command.hpp>
#include <ghost_msgs/msg/robot_trajectory.hpp>
#include <ghost_msgs/srv/start_recorder.hpp>
#include <ghost_msgs/srv/stop_recorder.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/int64.hpp>

#include <ghost_tank/tank_tree.hpp>
#include <ghost_tank/tank_odom.hpp>
#include <ghost_tank/control/tank_pid_controller.hpp>
#include <ghost_tank/tank_robot_plugin.hpp>

namespace ghost_tank
{

class ClydePlugin : public ghost_tank::TankRobotPlugin
{
public:
  using JoyPtr = std::shared_ptr<ghost_v5_interfaces::devices::JoystickDeviceData>;

  ClydePlugin();

  void initialize() override;
  void disabled() override;
  void autonomous(double current_time) override;
  void teleop(double current_time) override;
  
protected:
  // Construction
  void populateMotorNames();
  void populateDigitalIONames();

  // // Initialization
  // void initROSComms();
  // void initEstimation();
  // void initIntake();
  // void initTankModel();
  // void initAutonomy();

  void updateScissor(bool up, bool down, bool enabled);
  void updateT1Climb(bool up, bool down, bool enabled);
  void updateClamp(bool l1_pressed, double current_time);

  bool m_clamp_closed = false;
  bool m_clamp_btn_pressed = false;
  // Time at which a delayed clamp open takes effect (< 0 = no open pending)
  double m_clamp_open_time = -1.0;
  // L1 was pressed with the clamp open and is still held, waiting to close
  bool m_clamp_grabbing = false;

  // Auto-close: while grabbing (L1 held), the clamp closes when the auto_claw
  // distance sensor reads below m_auto_claw_close_mm.
  rclcpp::Subscription<ghost_msgs::msg::DistanceSensorState>::SharedPtr m_auto_claw_sub;
  void autoClawCallback(const ghost_msgs::msg::DistanceSensorState::SharedPtr msg)
  {
    m_auto_claw_distance_mm = msg->distance_mm;
    m_auto_claw_valid = (msg->range_status == 0);
    m_auto_claw_msg_time = std::chrono::steady_clock::now();
  }
  uint16_t m_auto_claw_distance_mm = 0;
  bool m_auto_claw_valid = false;
  std::chrono::steady_clock::time_point m_auto_claw_msg_time{};
  double m_auto_claw_close_mm = 75.0;

};

} // namespace ghost_tank
