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
  PDControl(float kp_xy_,
    float kd_xy_,
    float kp_theta_,
    float kd_theta_);
  Eigen::Vector2d tank_pid(Eigen::Vector3d cur_pos, Eigen::Vector2d end_pos, float time);

private:
  float prev_time_;
  float prev_error_xy_;
  float prev_error_theta_;

  float kp_xy_;
  float kd_xy_;
  float kp_theta_;
  float kd_theta_;
};

} // namespace ghost_tank