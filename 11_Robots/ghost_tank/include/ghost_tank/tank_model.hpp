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

#include "eigen3/Eigen/Geometry"
#include <ghost_util/angle_util.hpp>
#include <ghost_util/unit_conversion_utils.hpp>
#include "math/line2d.h"

namespace ghost_tank
{

struct TankConfig
{


};

struct ModuleState
{
  double wheel_position;
  double wheel_velocity;
  double wheel_acceleration;
  double steering_angle;
  double steering_velocity;
  double steering_acceleration;

  ModuleState() = default;

  ModuleState(
    double wheel_pos, double steering_ang, double wheel_vel, double steering_vel,
    double wheel_accel = 0.0, double steering_accel = 0.0)
  {
    wheel_position = wheel_pos;
    steering_angle = ghost_util::WrapAngle360(steering_ang);
    wheel_velocity = wheel_vel;
    steering_velocity = steering_vel;
    wheel_acceleration = wheel_accel;
    steering_acceleration = steering_accel;
  }

  bool operator==(const ModuleState & rhs) const
  {
    return (std::fabs(wheel_position - rhs.wheel_position) <
           std::numeric_limits<double>::epsilon()) &&
           (std::fabs(wheel_velocity - rhs.wheel_velocity) <
           std::numeric_limits<double>::epsilon()) &&
           (std::fabs(wheel_acceleration - rhs.wheel_acceleration) <
           std::numeric_limits<double>::epsilon()) &&
           (std::fabs(steering_angle - rhs.steering_angle) <
           std::numeric_limits<double>::epsilon()) &&
           (std::fabs(steering_velocity - rhs.steering_velocity) <
           std::numeric_limits<double>::epsilon()) &&
           (std::fabs(steering_acceleration - rhs.steering_acceleration) <
           std::numeric_limits<double>::epsilon());
  }
};

struct ModuleCommand
{
  double wheel_velocity_command;
  double wheel_voltage_command;
  double steering_angle_command;
  double steering_velocity_command;
  double steering_voltage_command;

  Eigen::Vector2d wheel_velocity_vector = Eigen::Vector2d(0.0, 0.0);
  Eigen::Vector2d actuator_velocity_commands = Eigen::Vector2d(0.0, 0.0);
  Eigen::Vector2d actuator_voltage_commands = Eigen::Vector2d(0.0, 0.0);

  ModuleCommand() = default;

  bool operator==(const ModuleCommand & rhs) const
  {
    return (std::fabs(wheel_velocity_command - rhs.wheel_velocity_command) <
           std::numeric_limits<double>::epsilon()) &&
           (std::fabs(wheel_voltage_command - rhs.wheel_voltage_command) <
           std::numeric_limits<double>::epsilon()) &&
           (std::fabs(steering_angle_command - rhs.steering_angle_command) <
           std::numeric_limits<double>::epsilon()) &&
           (std::fabs(steering_velocity_command - rhs.steering_velocity_command) <
           std::numeric_limits<double>::epsilon()) &&
           (std::fabs(steering_voltage_command - rhs.steering_voltage_command) <
           std::numeric_limits<double>::epsilon()) &&
           (actuator_velocity_commands.isApprox(rhs.actuator_velocity_commands)) &&
           (actuator_voltage_commands.isApprox(rhs.actuator_voltage_commands));
  }
};

class TankModel
{
public:
  TankModel(TankConfig config);

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
   * @brief Calculates various attributes of the tank model based on the current Module States. Call after updating
   * all modules with new sensor data.
   */
  void updateTankModel();

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

  const Eigen::Vector3d & getBaseVelocityCommand()
  {
    return m_base_vel_cmd;
  }

  const Eigen::Vector3d & getBaseVelocityCurrent()
  {
    return m_base_vel_curr;
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

  void setAutoStatus(bool state)
  {
    m_auto_status = state;
  }

  bool getAutoStatus()
  {
    return m_auto_status;
  }

  void setAutonTime(double time)
  {
    m_auton_time = time;
  }
  double getAutonTime()
  {
    return m_auton_time;
  }

protected:
  // Initialization
  void validateConfig();
  void calculateMaxBaseTwist();

  // Model Updates
  void calculateOdometry();
  void updateBaseTwist();

  // Configuration
  TankConfig m_config;
  double m_max_base_lin_vel = 0;
  double m_max_base_ang_vel = 0;
  double LIN_VEL_TO_RPM;

  // Odometry
  Eigen::Vector3d m_odom_pose;

  Eigen::Vector3d m_world_pose;

  Eigen::Vector3d m_world_twist;

  // Steering Integral
  std::unordered_map<std::string, double> m_error_sum_map;

  // Current centroidal states
  double m_curr_angle;
  Eigen::Vector3d m_base_vel_curr;

  // Command Setpoints
  Eigen::Vector3d m_base_vel_cmd;

  // Auton
  bool m_auto_status = false;
  double m_auton_time = 0.0;
};

} // namespace ghost_tank
