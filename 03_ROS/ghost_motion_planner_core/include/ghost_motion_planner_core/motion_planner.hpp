/*
 *   Copyright (c) 2024 Jake Wendling
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

#include <rclcpp/rclcpp.hpp>

#include "ghost_msgs/msg/drivetrain_command.hpp"
#include "ghost_msgs/msg/robot_trajectory.hpp"
#include "ghost_util/angle_util.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <ghost_v5_interfaces/robot_hardware_interface.hpp>
#include <ghost_v5_interfaces/util/device_config_factory_utils.hpp>

namespace ghost_motion_planner
{

class MotionPlanner
{
public:
  MotionPlanner() = default;
  virtual ~MotionPlanner() = default;

  ///////////////////////////
  ///// Virtual Methods /////
  ///////////////////////////
  /**
   * @brief Called directly after instantiation, when the robot is configured.
   *
   * Member variables provide access to ROS Node for adding ROS Interfaces (topics, services, etc.) and for loading
   * ROS Params at runtime.
   *
   * This will block any other competition functionality until it completes.
   * This method will not update actuator commands.
   */
  virtual void initialize() = 0;

  // Blocking, ignore sensor updates while making new plan
  // overload for teleop and auton?
  /**
   * @brief Called when a DrivetrainCommand msg is recieved
   *
   * Generates and publishes a RobotTrajectory msg when completed
   *
   */
  virtual void generateMotionPlan(const ghost_msgs::msg::DrivetrainCommand::SharedPtr cmd) = 0;


  //////////////////////////////
  ///// Base Class Methods /////
  //////////////////////////////

  /**
   * @brief Called for all motion planner classes after construction. Calls user-defined intialize method internally.
   */
  void configure(std::string node_name);

  /**
   * @brief Returns a shared pointer to the ROS node for this robot instance
   *
   * @return std::shared_ptr<rclcpp::Node>
   */
  std::shared_ptr<rclcpp::Node> getROSNodePtr() const
  {
    if (!configured_) {
      throw std::runtime_error(
              "[motion_planner::getROSNodePtr] Error: This plugin has not been configured yet!");
    }
    return node_ptr_;
  }

protected:
  rclcpp::Publisher<ghost_msgs::msg::RobotTrajectory>::SharedPtr trajectory_pub_;
  std::shared_ptr<rclcpp::Node> node_ptr_;
  double current_x_ = 0.0;
  double current_y_ = 0.0;
  double current_x_vel_ = 0.0;
  double current_y_vel_ = 0.0;
  double current_theta_rad_ = 0.0;
  double current_theta_vel_rad_ = 0.0;

private:
  void setNewCommand(const ghost_msgs::msg::DrivetrainCommand::SharedPtr cmd);
  void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg);

  bool configured_ = false;
  std::atomic_bool planning_ = false;
  rclcpp::Subscription<ghost_msgs::msg::DrivetrainCommand>::SharedPtr pose_command_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

} // namespace ghost_motion_planner
