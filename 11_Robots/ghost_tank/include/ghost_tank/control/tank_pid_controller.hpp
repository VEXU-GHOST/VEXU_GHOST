#pragma once
#include <vector>
#include <iostream>
#include <eigen3/Eigen/Core>

#include <ghost_tank/control/tank_robot_state.hpp>
#include <ghost_util/angle_util.hpp>
#include <ghost_util/math_util.hpp>
#include <ghost_util/unit_conversion_utils.hpp>
#include <cmath>

namespace ghost_tank
{

struct PIDGains
{
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double integral_limit = 0.0;
};

class PDControl
{
public:
  PDControl(PIDGains linear_gains, PIDGains angular_gains, float dt = 0.01)
  : angular_gains_(linear_gains), linear_gains_(angular_gains), dt_(dt)
  {
    reset();
  }

  void reset()
  {
    angular_integral_sum_ = 0.0;
    linear_integral_sum_ = 0.0;
    last_linear_error_ = 0.0;
    last_angular_error_ = 0.0;
  }

  Eigen::Vector2d tank_pid(const Eigen::Vector3d & cur_pos, const Eigen::Vector3d & cur_twist, const Eigen::Vector3d & carrot_pos, const Eigen::Vector3d & final_pos, bool backwards, bool ignore_lateral_error = false)
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
    output_linear = linear_gains_.kp * dist_err + linear_gains_.kd * derivative_err_linear;
    output_linear = ghost_util::clamp(output_linear, -1.0f, 1.0f);
    output_linear *= (backwards) ? -1.0 : 1.0;

    float derivative_theta = -cur_twist.z();
    output_angular = angular_gains_.kp * error_theta + angular_gains_.kd * derivative_theta;
    output_angular = ghost_util::clamp(output_angular, -1.0f, 1.0f);

    return Eigen::Vector2d(output_linear, output_angular);
  }

  Eigen::Vector2d theta_pid(const Eigen::Vector3d & cur_pos, const Eigen::Vector3d & cur_twist, const Eigen::Vector3d & end_pos)
  {
    float error_theta = ghost_util::SmallestAngleDistRad(end_pos.z(), cur_pos.z());
    float derivative_theta = -cur_twist.z();
    float output_angular = angular_gains_.kp * error_theta + angular_gains_.kd * derivative_theta;
    output_angular = ghost_util::clamp(output_angular, -1.0f, 1.0f);

    return Eigen::Vector2d(0.0, output_angular);
  }

  Eigen::Vector2d theta_pd(const Eigen::Vector3d & cur_pos, const Eigen::Vector3d & cur_twist, const Eigen::Vector3d & end_pos)
  {
    float error_theta = ghost_util::SmallestAngleDistRad(end_pos.z(), cur_pos.z());
    float derivative_theta = -cur_twist.z();

    angular_integral_sum_ += error_theta * dt_;

    // Check for sign change reset
    if (last_angular_error_ * error_theta < 0) {
      angular_integral_sum_ = 0.0;
    }

    last_angular_error_ = error_theta;

    float integral_contribution = ghost_util::clamp(angular_gains_.ki * angular_integral_sum_, -angular_gains_.integral_limit, angular_gains_.integral_limit);

    float output_angular = angular_gains_.kp * error_theta + angular_gains_.kd * derivative_theta + integral_contribution;
    output_angular = ghost_util::clamp(output_angular, -1.0f, 1.0f);

    return Eigen::Vector2d(0.0, output_angular);
  }

private:
  PIDGains angular_gains_;
  PIDGains linear_gains_;

  double angular_integral_sum_{0.0};
  double linear_integral_sum_{0.0};

  double last_linear_error_{0.0};
  double last_angular_error_{0.0};

  float dt_ = 0.01;
};

} // namespace ghost_tank
