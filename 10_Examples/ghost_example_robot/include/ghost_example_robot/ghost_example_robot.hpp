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

#include <memory>
#include <ghost_planners/robot_trajectory.hpp>
#include <ghost_ros_interfaces/competition/v5_robot_base.hpp>
#include <ghost_ros_interfaces/msg_helpers/msg_helpers.hpp>
#include <ghost_tank/tank_odom.hpp>
#include <nav_msgs/msg/odometry.hpp>


namespace ghost_example_robot
{

class GhostExampleRobot : public ghost_ros_interfaces::V5RobotBase
{
public:
  GhostExampleRobot();

  void initialize() override;
  void disabled() override;
  void autonomous(double current_time) override;
  void teleop(double current_time) override;
  void onNewSensorData() override;

private:
  std::unique_ptr<ghost_tank::TankOdometry> odom_;
  bool encoder_baseline_set_ = false;
  double previous_left_position_ = 0.0;
  double previous_right_position_ = 0.0;
  double left_remainder_ = 0.0;
  double right_remainder_ = 0.0;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

} // namespace ghost_example_robot
