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
    float kp_xy_,
    float kd_xy_,
    float kp_theta_,
    float kd_theta_);
  Eigen::Vector2d tank_pid(const Eigen::Vector3d & cur_pos, const Eigen::Vector3d & cur_twist, const Eigen::Vector3d & carrot_pos, const Eigen::Vector3d & final_pos, bool backwards);
  Eigen::Vector2d theta_pid(const Eigen::Vector3d & cur_pos, const Eigen::Vector3d & cur_twist, const Eigen::Vector3d & end_pos);

private:
  float kp_xy_;
  float kd_xy_;
  float kp_theta_;
  float kd_theta_;
};

} // namespace ghost_tank
