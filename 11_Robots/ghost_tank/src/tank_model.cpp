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

TankModel::TankModel(TankConfig config)
{
  m_config = config;

  validateConfig();
  calculateMaxBaseTwist();

  // Initialize module specific data
  // for (const auto & [name, _] : m_config.module_positions) {
  //   m_current_module_states[name] = ModuleState();
  //   m_previous_module_states[name] = ModuleState();
  //   m_module_commands[name] = ModuleCommand();
  //   m_error_sum_map[name] = 0.0;
  // }
}

void TankModel::validateConfig()
{
  std::unordered_map<std::string, double> larger_than_zero_params{
    // {"max_wheel_lin_vel", m_config.max_wheel_lin_vel},
    // {"steering_ratio", m_config.steering_ratio},
    // {"wheel_ratio", m_config.wheel_ratio},
    // {"wheel_radius", m_config.wheel_radius},
    // {"controller_dt", m_config.controller_dt},
    // {"max_wheel_actuator_vel", m_config.max_wheel_actuator_vel}
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
    // {"steering_kd", m_config.steering_kd},
    // {"steering_ki", m_config.steering_ki},
    // {"steering_ki_limit", m_config.steering_ki_limit},
    // {"steering_control_deadzone", m_config.steering_control_deadzone},
  };

  for (const auto & [key, val] : larger_or_equal_to_zero_params) {
    if (val < 0) {
      std::string err_string =
        std::string("[TankModel::validateConfig] Error: ") + key + " must be positive!";
      throw std::runtime_error(err_string);
    }
  }

  // LIN_VEL_TO_RPM = ghost_util::METERS_TO_INCHES / m_config.wheel_radius *
  //   ghost_util::RAD_PER_SEC_TO_RPM;

  // Initialize Base States
  m_odom_pose = Eigen::Vector3d::Zero();

  m_world_pose = Eigen::Vector3d::Zero();

  m_world_twist = Eigen::Vector3d::Zero();
}

void TankModel::calculateMaxBaseTwist()
{
  // // Get Max Base Speeds
  // double max_wheel_dist = 0.0;
  // for (const auto & [key, val] : m_config.module_positions) {
  //   max_wheel_dist = std::max(max_wheel_dist, (double) val.norm());
  // }

  // m_max_base_lin_vel = m_config.max_wheel_lin_vel;
  // m_max_base_ang_vel = m_max_base_lin_vel / max_wheel_dist;
}

// Assumes all module states have been updated prior to update
void TankModel::updateTankModel()
{
  updateBaseTwist();
  calculateOdometry();
}

void TankModel::updateBaseTwist()
{
  // Eigen::VectorXd module_velocity_vector(2 * m_num_modules);
  // int n = 0;
  // for (const auto & [name, state] : m_current_module_states) {
  //   module_velocity_vector[2 * n] = state.wheel_velocity / LIN_VEL_TO_RPM * cos(
  //     state.steering_angle * ghost_util::DEG_TO_RAD);
  //   module_velocity_vector[2 * n + 1] = state.wheel_velocity / LIN_VEL_TO_RPM * sin(
  //     state.steering_angle * ghost_util::DEG_TO_RAD);
  //   n++;
  // }

  // m_base_vel_curr = m_task_space_jacobian * module_velocity_vector;
  // m_ls_error_metric =
  //   (module_velocity_vector - m_task_space_jacobian_inverse * m_base_vel_curr).norm();

  // m_base_vel_curr[0] = (std::fabs(m_base_vel_curr[0]) > 0.01) ? m_base_vel_curr[0] : 0.0;
  // m_base_vel_curr[1] = (std::fabs(m_base_vel_curr[1]) > 0.01) ? m_base_vel_curr[1] : 0.0;
  // m_base_vel_curr[2] = (std::fabs(m_base_vel_curr[2]) > 0.02) ? m_base_vel_curr[2] : 0.0;
}

void TankModel::calculateOdometry()
{
  // auto rotate_base_to_odom = Eigen::Rotation2D<double>(m_odom_angle).toRotationMatrix();
  // m_odom_loc += rotate_base_to_odom *
  //   Eigen::Vector2d(m_base_vel_curr.x(), m_base_vel_curr.y()) * 0.01;
  // m_odom_angle += m_base_vel_curr.z() * 0.01;
  // m_odom_angle = ghost_util::WrapAngle2PI(m_odom_angle);
}

} // namespace ghost_tank
