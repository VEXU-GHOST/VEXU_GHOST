#pragma once
#include <vector>
#include <iostream>
#include <eigen3/Eigen/Core>

namespace ghost_tank
{

class PDControl
{

public:
  //constructor
  PDControl(
    float kp_xy,
    float kd_xy,
    float kp_theta,
    float kd_theta,
    float ki_theta,
    float integral_limit,
    float dt = 0.01);
  Eigen::Vector2d tank_pid(const Eigen::Vector3d & cur_pos, const Eigen::Vector3d & cur_twist, const Eigen::Vector3d & carrot_pos, const Eigen::Vector3d & final_pos, bool backwards, bool ignore_lateral_error = false);
  Eigen::Vector2d theta_pid(const Eigen::Vector3d & cur_pos, const Eigen::Vector3d & cur_twist, const Eigen::Vector3d & end_pos);
  Eigen::Vector2d theta_pd(const Eigen::Vector3d & cur_pos, const Eigen::Vector3d & cur_twist, const Eigen::Vector3d & end_pos);

private:
  float kp_xy_;
  float kd_xy_;
  float kp_theta_;
  float kd_theta_;
  float ki_theta_;
  float integral_limit_;
  float integral_theta_ = 0.0;
  float prev_error_theta_ = 0.0;
  float dt_ = 0.01;
  float last_twist_z_;
  Eigen::Vector3d prev_final_pos_;
};

} // namespace ghost_tank
