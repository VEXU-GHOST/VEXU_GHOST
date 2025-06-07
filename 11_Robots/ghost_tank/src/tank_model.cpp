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

#include <ghost_tank/tank_model.hpp>
#include <ghost_util/angle_util.hpp>
#include <ghost_util/math_util.hpp>
#include <ghost_util/vector_util.hpp>
#include "ghost_util/unit_conversion_utils.hpp"
#include <cmath>

using geometry::Line2d;
using ghost_util::angleBetweenVectorsRadians;
namespace ghost_tank
{

  TankModel::TankModel(
      std::shared_ptr<rclcpp::Node> node_ptr,
      std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr,
      TankConfig config)
      : node_ptr_(node_ptr),
        rhi_ptr_(rhi_ptr)
  {
    m_config = config;

    validateConfig();
    calculateMaxBaseTwist();

    node_ptr_->declare_parameter("particle_filter.rviz_set_pose_topic", "/set_pf_pose");
    std::string particle_filter_set_pose_topic = node_ptr_->get_parameter(
                                                              "particle_filter.rviz_set_pose_topic")
                                                     .as_string();

    m_particle_filter_set_pose_publisher =
        node_ptr_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            particle_filter_set_pose_topic,
            10);
  }

  double TankModel::getCurrentHighestWheelLinearVelocity()
  {

    double max_vel = 0.0;

    for (const auto motor_name : m_config.motor_list_left)
    {
      double wheel_vel = rhi_ptr_->getMotorVelocityRPM(motor_name);
      std::cout << "\t" << motor_name << ": " << wheel_vel << std::endl;
      max_vel = std::max<double>(std::fabs(wheel_vel), max_vel);
    }

    for (const auto motor_name : m_config.motor_list_right)
    {
      double wheel_vel = rhi_ptr_->getMotorVelocityRPM(motor_name);
      std::cout << "\t" << motor_name << ": " << wheel_vel << std::endl;
      max_vel = std::max<double>(std::fabs(wheel_vel), max_vel);
    }

    std::cout << "[TankModel::getCurrentHighestWheelLinearVelocity] Max Wheel RPM: " << max_vel << std::endl;

    max_vel *= M_2PI * m_config.wheel_radius_in * ghost_util::INCHES_TO_METERS * m_config.wheel_gear_ratio / 60.0;

    return max_vel;
  }

  void TankModel::validateConfig()
  {
    std::unordered_map<std::string, double> larger_than_zero_params{
        {"wheel_radius_in", m_config.wheel_radius_in},
    };

    for (const auto &[key, val] : larger_than_zero_params)
    {
      if (val <= 0)
      {
        std::string err_string =
            std::string("[TankModel::validateConfig] Error: ") + key +
            " must be non-zero and positive!";
        throw std::runtime_error(err_string);
      }
    }

    std::unordered_map<std::string, double> larger_or_equal_to_zero_params{};

    for (const auto &[key, val] : larger_or_equal_to_zero_params)
    {
      if (val < 0)
      {
        std::string err_string =
            std::string("[TankModel::validateConfig] Error: ") + key + " must be positive!";
        throw std::runtime_error(err_string);
      }
    }

    // Initialize Base States
    m_odom_pose = Eigen::Vector3d::Zero();

    m_world_pose = Eigen::Vector3d::Zero();

    m_world_twist = Eigen::Vector3d::Zero();
  }

  void TankModel::calculateMaxBaseTwist()
  {
    // max motor speed is 11.4 is about 680ish RPM
    m_max_base_lin_vel = M_2PI * m_config.wheel_radius_in * ghost_util::INCHES_TO_METERS * m_config.wheel_gear_ratio * 11.4;
    m_max_base_ang_vel = m_max_base_lin_vel / (m_config.wheel_dist_in * ghost_util::INCHES_TO_METERS);
  }

  Eigen::Vector2d TankModel::wheelVelocitiesToChassisTwist(Eigen::Vector2d wheel_velocities) const
  {
    double left_vel = wheel_velocities.x();
    double right_vel = wheel_velocities.y();

    double linear_x_vel = (left_vel + right_vel) / 2.0;
    double angular_z_vel = (right_vel - left_vel) / (2.0 * m_config.wheel_dist_in * ghost_util::INCHES_TO_METERS);

    return Eigen::Vector2d(linear_x_vel, angular_z_vel);
  }

  Eigen::Vector2d TankModel::chassisTwistToWheelVelocities(Eigen::Vector2d chassis_twist) const
  {
    double linear_x_vel = chassis_twist.x();
    double angular_z_vel = chassis_twist.y();

    double left_vel = linear_x_vel - (angular_z_vel * m_config.wheel_dist_in * ghost_util::INCHES_TO_METERS);
    double right_vel = linear_x_vel + (angular_z_vel * m_config.wheel_dist_in * ghost_util::INCHES_TO_METERS);

    return Eigen::Vector2d(left_vel, right_vel);
  }

  double TankModel::getMaxLinearVelocityFromAngularVelocity(double desired_angular_velocity_rad_s) const
  {
    double angular_vel_component = desired_angular_velocity_rad_s * m_config.wheel_dist_in * ghost_util::INCHES_TO_METERS;
    double max_allowed_linear_vel = m_max_base_lin_vel - std::fabs(angular_vel_component);
    return std::max(0.0, max_allowed_linear_vel);
  }

  void TankModel::normalizeArcadeCommand(Eigen::Vector2d &cmd)
  {
    auto &fwd_cmd = cmd.x();
    auto &ang_cmd = cmd.y();

    auto left_cmd = fwd_cmd - ang_cmd;
    auto right_cmd = fwd_cmd + ang_cmd;

    auto max_magnitude = std::max(std::fabs(left_cmd), std::fabs(right_cmd));
    auto scale = 1.0 / std::max(1.0, max_magnitude);

    cmd *= scale;
  }

  void TankModel::driveCommandArcade(double fwd_pct, double ang_pct)
  {
    ghost_util::clamp(fwd_pct, -1.0, 1.0);
    ghost_util::clamp(ang_pct, -1.0, 1.0);
    double left_cmd = fwd_pct - ang_pct;
    double right_cmd = fwd_pct + ang_pct;

    for (const auto motor_name : m_config.motor_list_left)
    {
      rhi_ptr_->setMotorCurrentLimitMilliAmps(motor_name, 2500);
      rhi_ptr_->setMotorVoltageCommandPercent(motor_name, left_cmd);
    }

    for (const auto motor_name : m_config.motor_list_right)
    {
      rhi_ptr_->setMotorCurrentLimitMilliAmps(motor_name, 2500);
      rhi_ptr_->setMotorVoltageCommandPercent(motor_name, right_cmd);
    }
  }

  void TankModel::driveCommandTank(double left_pct, double right_pct)
  {
    ghost_util::clamp(left_pct, -1.0, 1.0);
    ghost_util::clamp(right_pct, -1.0, 1.0);

    for (const auto motor_name : m_config.motor_list_left)
    {
      rhi_ptr_->setMotorCurrentLimitMilliAmps(motor_name, 2500);
      rhi_ptr_->setMotorVoltageCommandPercent(motor_name, left_pct);
    }

    for (const auto motor_name : m_config.motor_list_right)
    {
      rhi_ptr_->setMotorCurrentLimitMilliAmps(motor_name, 2500);
      rhi_ptr_->setMotorVoltageCommandPercent(motor_name, right_pct);
    }
  }

  void TankModel::driveCommandJoystick(double fwd, double ang, double deadzone)
  {
    double forward_vel = fwd / 127.0;
    double angular_vel = ang / 127.0;

    forward_vel = (std::fabs(forward_vel) < deadzone) ? 0.0 : forward_vel;
    angular_vel = (std::fabs(angular_vel) < deadzone) ? 0.0 : angular_vel;

    driveCommandArcade(forward_vel, angular_vel);
  }

} // namespace ghost_tank
