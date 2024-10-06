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

#include <ghost_ros_interfaces/msg_helpers/msg_helpers.hpp>
#include "ghost_motion_planner_core/motion_planner.hpp"

using ghost_ros_interfaces::msg_helpers::fromROSMsg;
using ghost_v5_interfaces::devices::hardware_type_e;
using ghost_v5_interfaces::RobotHardwareInterface;
using ghost_v5_interfaces::util::loadRobotConfigFromYAMLFile;
using std::placeholders::_1;

namespace ghost_motion_planner
{

void MotionPlanner::configure(std::string node_name)
{
  std::cout << "Configuring Motion Planner" << std::endl;
  node_ptr_ = std::make_shared<rclcpp::Node>(node_name);

  node_ptr_->declare_parameter("command_topic", "/motion_planner/command");
  std::string command_topic = node_ptr_->get_parameter("command_topic").as_string();

  node_ptr_->declare_parameter("sensor_update_topic", "/v5/sensor_update");
  std::string sensor_update_topic = node_ptr_->get_parameter("sensor_update_topic").as_string();

  node_ptr_->declare_parameter("trajectory_topic", "/motion_planner/trajectory");
  std::string trajectory_topic = node_ptr_->get_parameter("trajectory_topic").as_string();

  node_ptr_->declare_parameter("odom_topic", "/map_ekf/odometry");
  std::string odom_topic = node_ptr_->get_parameter("odom_topic").as_string();

  pose_command_sub_ = node_ptr_->create_subscription<ghost_msgs::msg::DrivetrainCommand>(
    command_topic,
    10,
    std::bind(&MotionPlanner::setNewCommand, this, _1)
  );

  trajectory_pub_ = node_ptr_->create_publisher<ghost_msgs::msg::RobotTrajectory>(
    trajectory_topic,
    10);

  odom_sub_ = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic,
    10,
    std::bind(&MotionPlanner::odomCallback, this, _1)
  );

  initialize();
  configured_ = true;
}

void MotionPlanner::setNewCommand(const ghost_msgs::msg::DrivetrainCommand::SharedPtr cmd)
{
  planning_ = true;
  RCLCPP_INFO(node_ptr_->get_logger(), "Received Pose");
  generateMotionPlan(cmd);
  planning_ = false;
}

void MotionPlanner::odomCallback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;
  current_x_vel_ = msg->twist.twist.linear.x;
  current_y_vel_ = msg->twist.twist.linear.y;

  current_theta_rad_ = ghost_util::quaternionToYawRad(
    msg->pose.pose.orientation.w,
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z);
  current_theta_vel_rad_ = msg->twist.twist.angular.z;
}

} // namespace ghost_motion_planner
