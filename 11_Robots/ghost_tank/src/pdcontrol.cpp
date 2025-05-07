#include <iostream>
#include <ghost_tank/tank_model.hpp>
#include <ghost_tank/tank_robot_plugin.hpp>
#include <ghost_util/angle_util.hpp>
#include <ghost_util/unit_conversion_utils.hpp>
#include <ghost_tank/pdcontrol.hpp>
#include <cmath>

namespace ghost_tank
{

PDControl::PDControl(
  float kp_xy,
  float kd_xy,
  float kp_theta,
  float kd_theta,
  float ki_theta,
  float integral_limit,
  float dt)
: kp_xy_(kp_xy), kd_xy_(kd_xy), kp_theta_(kp_theta), kd_theta_(kd_theta), ki_theta_(ki_theta), integral_limit_(integral_limit), dt_(dt)
{
  integral_theta_ = 0.0;
  prev_error_theta_ = 0.0;
  last_twist_z_ = 0.0;
}

Eigen::Vector2d PDControl::tank_pid(const Eigen::Vector3d & cur_pos, const Eigen::Vector3d & cur_twist, const Eigen::Vector3d & carrot_pos, const Eigen::Vector3d & final_pos, bool backwards, bool ignore_lateral_error)
{
  float output_linear = 0.0;
  float output_angular = 0.0;

  // Angular PD, chases Carrot Pose
  Eigen::Vector2d carrot_rel = carrot_pos.head<2>() - cur_pos.head<2>();
  float des_theta = atan2(carrot_rel.y(), carrot_rel.x());
  float curr_angle = (backwards) ? 3.1415 + cur_pos.z() : cur_pos.z();
  float error_theta = ghost_util::SmallestAngleDistRad(des_theta, curr_angle);

  // Translational PD, always goes to Final Pose
  float dist_err = (final_pos.head<2>() - cur_pos.head<2>()).norm();
  float derivative_err_linear = -cur_twist.head<2>().norm();
  if (ignore_lateral_error) {
    dist_err *= cos(error_theta);
    derivative_err_linear *= cos(error_theta);
  }
  output_linear = kp_xy_ * dist_err + kd_xy_ * derivative_err_linear;
  output_linear = ghost_util::clamp(output_linear, -1.0f, 1.0f);
  output_linear *= (backwards) ? -1.0 : 1.0;

  float derivative_theta = -cur_twist.z();
  output_angular = kp_theta_ * error_theta + kd_theta_ * derivative_theta;
  output_angular = ghost_util::clamp(output_angular, -1.0f, 1.0f);

  return Eigen::Vector2d(output_linear, output_angular);
}

Eigen::Vector2d PDControl::theta_pd(const Eigen::Vector3d & cur_pos, const Eigen::Vector3d & cur_twist, const Eigen::Vector3d & end_pos)
{
  float error_theta = ghost_util::SmallestAngleDistRad(end_pos.z(), cur_pos.z());
  float derivative_theta = -cur_twist.z();
  float output_angular = kp_theta_ * error_theta + kd_theta_ * derivative_theta;
  output_angular = ghost_util::clamp(output_angular, -1.0f, 1.0f);

  return Eigen::Vector2d(0.0, output_angular);
}

Eigen::Vector2d PDControl::theta_pid(const Eigen::Vector3d & cur_pos, const Eigen::Vector3d & cur_twist, const Eigen::Vector3d & end_pos)
{
  if (prev_final_pos_ != end_pos) {
    integral_theta_ = 0.0;
    prev_final_pos_ = end_pos;
  }

  float error_theta = ghost_util::SmallestAngleDistRad(end_pos.z(), cur_pos.z());
  float derivative_theta = -cur_twist.z();

  integral_theta_ += error_theta * dt_;

  // Check for sign change reset
  if (prev_error_theta_ * error_theta < 0) {
    integral_theta_ = 0.0;
  }

  prev_error_theta_ = error_theta;

  float integral_contribution = ghost_util::clamp(ki_theta_ * integral_theta_, -integral_limit_, integral_limit_);

  float output_angular = kp_theta_ * error_theta + kd_theta_ * derivative_theta + integral_contribution;
  output_angular = ghost_util::clamp(output_angular, -1.0f, 1.0f);

  return Eigen::Vector2d(0.0, output_angular);
}

} // namespace ghost_tank
