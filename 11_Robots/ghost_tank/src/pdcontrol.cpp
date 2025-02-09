#include <iostream>
#include <ghost_tank/tank_model.hpp>
#include <ghost_tank/tank_robot_plugin.hpp>
#include <ghost_util/angle_util.hpp>
#include <ghost_util/unit_conversion_utils.hpp>
#include <ghost_tank/pdcontrol.hpp>
#include <cmath>

namespace ghost_tank
{

PDControl::PDControl(float kp_xy,
    float kd_xy,
    float kp_theta,
    float kd_theta):
    kp_xy_(kp_xy), kd_xy_(kd_xy), kp_theta_(kp_theta), kd_theta_(kd_theta)
{
  prev_time_ = 0;
  prev_error_xy_ = 0;
  prev_error_theta_ = 0;
}

Eigen::Vector2d PDControl::tank_pid(Eigen::Vector3d cur_pos, Eigen::Vector2d end_pos, float time)
{
  float error_x = end_pos.x() - cur_pos.x();
  float error_y = end_pos.y() - cur_pos.y();
  float error_xy = sqrt(pow(error_x, 2) + pow(error_y, 2));
  float end_theta = atan2(error_y, error_x);
  float error_theta = ghost_util::SmallestAngleDistRad(end_theta, cur_pos.z());

  float bias = 0;
  float derivative;

  float bias_theta = 0;
  float derivative_theta;
  
  float delta_time = time - prev_time_;
  prev_time_ = time;

  float output_linear = 0.0;
  float output_angular = 0.0;

  derivative_theta = (error_theta - prev_error_theta_) / delta_time;
  output_angular = kp_theta_ * error_theta + kd_theta_ * derivative_theta + bias_theta;
  output_angular = ghost_util::clamp(output_angular, -1.0f, 1.0f);

  derivative = (error_xy - prev_error_xy_) / delta_time;
  output_linear = kp_xy_ * error_xy + kd_xy_* derivative + bias;
  output_linear = ghost_util::clamp(output_linear, -1.0f, 1.0f);

  prev_error_xy_ = error_xy; //in meters
  prev_error_theta_ = error_theta; //in radians

  return Eigen::Vector2d(output_linear, output_angular);
}

} // namespace ghost_tank