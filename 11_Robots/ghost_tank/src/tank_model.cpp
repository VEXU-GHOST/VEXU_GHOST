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

using geometry::Line2d;
using ghost_util::angleBetweenVectorsRadians;
namespace ghost_tank
{

TankModel::TankModel(std::shared_ptr<rclcpp::Node> node_ptr, 
  std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr,
  TankConfig config)
  :node_ptr_(node_ptr), 
  rhi_ptr_(rhi_ptr)
{
  m_config = config;

  validateConfig();

  node_ptr_->declare_parameter("particle_filter.rviz_set_pose_topic", "/set_pf_pose");
  std::string particle_filter_set_pose_topic = node_ptr_->get_parameter(
    "particle_filter.rviz_set_pose_topic").as_string();

  m_particle_filter_set_pose_publisher =
    node_ptr_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    particle_filter_set_pose_topic,
    10);
}

void TankModel::validateConfig()
{
  std::unordered_map<std::string, double> larger_than_zero_params{
    {"wheel_radius", m_config.wheel_radius},
  };

  for (const auto & [key, val] : larger_than_zero_params) {
    if (val <= 0) {
      std::string err_string =
        std::string("[TankModel::validateConfig] Error: ") + key +
        " must be non-zero and positive!";
      throw std::runtime_error(err_string);
    }
  }

  std::unordered_map<std::string, double> larger_or_equal_to_zero_params{
  };

  for (const auto & [key, val] : larger_or_equal_to_zero_params) {
    if (val < 0) {
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
  // max motor speed is 600 rpm = 10 rps
  m_max_base_lin_vel = M_2PI * m_config.wheel_radius * m_config.wheel_gear_ratio * 10 *
    ghost_util::INCHES_TO_METERS;
  m_max_base_ang_vel = m_max_base_lin_vel / m_config.wheel_dist / ghost_util::INCHES_TO_METERS;
}

void TankModel::driveCommand(double fwd_pct, double ang_pct){
    double left_cmd = fwd_pct + ang_pct;
    double right_cmd = fwd_pct - ang_pct;

    for (const auto motor_name: m_config.motor_list) {
      rhi_ptr_->setMotorCurrentLimitMilliAmps(motor_name, 2500);
    }

    for (int i = 0; i < 6; i++) {
      rhi_ptr_->setMotorVoltageCommandPercent(m_config.motor_list[i], left_cmd);
    }

    for (int i = 6; i < 12; i++) {
      rhi_ptr_->setMotorVoltageCommandPercent(m_config.motor_list[i], right_cmd);
    }
}

void TankModel::driveCommandJoystick(double fwd, double ang, double deadzone){
    double forward_vel = fwd / 127.0;
    double angular_vel = ang / 127.0;

    forward_vel = (std::fabs(forward_vel) < deadzone) ? 0.0 : forward_vel;
    angular_vel = (std::fabs(angular_vel) < deadzone) ? 0.0 : angular_vel;

    driveCommand(forward_vel, angular_vel);
}

} // namespace ghost_tank
