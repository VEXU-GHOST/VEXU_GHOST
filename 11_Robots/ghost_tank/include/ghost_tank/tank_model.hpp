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

#include <map>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "eigen3/Eigen/Geometry"
#include <ghost_util/angle_util.hpp>
#include <ghost_ros_interfaces/competition/v5_robot_base.hpp>
#include <ghost_util/unit_conversion_utils.hpp>
#include "math/line2d.h"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <ghost_v5_interfaces/robot_hardware_interface.hpp>


namespace ghost_tank
{

struct TankConfig
{
  std::vector<std::string> motor_list;
  double wheel_radius;
  double wheel_gear_ratio;
  double wheel_dist;
};

class TankModel
{
public:
  TankModel(std::shared_ptr<rclcpp::Node> node_ptr,
      std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr,
      TankConfig config);

  /**
   * @brief Get the Tank Model Configration
   *
   * @return const TankConfig&
   */
  const TankConfig & getConfig()
  {
    return m_config;
  }

  /**
   * @brief Get the max linear velocity of the robot base at nominal motor speed.
   *
   * @return double
   */
  double getMaxBaseLinearVelocity() const
  {
    return m_max_base_lin_vel;
  }

  /**
   * @brief Get the max angular velocity of the robot base at nominal motor speed.
   *
   * @return double
   */
  double getMaxBaseAngularVelocity() const
  {
    return m_max_base_ang_vel;
  }

  // Base States
  const Eigen::Vector3d & getOdometryPose()
  {
    return m_odom_pose;
  }

  double getOdometryAngle() const
  {
    return m_odom_pose.z();
  }

  const Eigen::Vector3d & getWorldPose()
  {
    return m_world_pose;
  }

  void setWorldPose(const double x, const double y, const double theta)
  {
    m_world_pose.x() = x;
    m_world_pose.y() = y;
    m_world_pose.z() = theta;
  }

  double getWorldAngleDeg() const
  {
    return m_world_pose.z() * ghost_util::RAD_TO_DEG;
  }

  double getWorldAngleRad() const
  {
    return m_world_pose.z();
  }

  void setWorldAngleRad(const double theta)
  {
    m_world_pose.z() = theta;
  }

  const Eigen::Vector3d & getWorldTwist()
  {
    return m_world_twist;
  }

  void setWorldTwist(const double x, const double y, const double theta)
  {
    m_world_twist.x() = x;
    m_world_twist.y() = y;
    m_world_twist.z() = theta;
  }

  const double getWorldAngularVelocity()
  {
    return m_world_twist.z();
  }

  void setWorldAngularVelocity(const double omega)
  {
    m_world_twist.z() = omega;
  }

  void driveCommand(double fwd_vel, double ang_vel);
  void driveCommandJoystick(double fwd_vel, double ang_vel, double deadzone);

protected:
  // Initialization
  void validateConfig();
  void calculateMaxBaseTwist();
  std::shared_ptr<rclcpp::Node> node_ptr_;
  std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr_;

  // Configuration
  TankConfig m_config;
  double m_max_base_lin_vel = 0;
  double m_max_base_ang_vel = 0;
  double LIN_VEL_TO_RPM;

  // Odometry
  Eigen::Vector3d m_odom_pose;

  Eigen::Vector3d m_world_pose;

  Eigen::Vector3d m_world_twist;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    m_particle_filter_set_pose_publisher;

  // Command Setpoints
  Eigen::Vector3d m_base_vel_cmd;
};

} // namespace ghost_tank
