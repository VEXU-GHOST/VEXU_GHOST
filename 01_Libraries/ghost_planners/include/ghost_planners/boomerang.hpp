#pragma once
#include <iostream>
#include <ghost_util/angle_util.hpp>
#include <ghost_util/unit_conversion_utils.hpp>
#include <eigen3/Eigen/Core>

#include <cmath>
#include <vector>

//class for Boomerang control, to travel from point a to b, maintaining knowledge of location.

namespace ghost_planners
{

class Boomerang {
public:
  Boomerang();

  //determine carrot point
  void find_carrot(Eigen::Vector3d cur_pos);
  void map_curve(Eigen::Vector3d cur_pos);

  void set_end_point(float x, float y, float radians);
  void set_lead(float lead);

  std::vector<Eigen::Vector2d> get_points();

private:
  float end_x_;
  float end_y_;
  float end_radians_;

  float lead_;     //value of 0.00 - 1.00, how much you want to lead the curve.
  float carrot_x_;
  float carrot_y_;

  std::vector<Eigen::Vector2d> points_;
};

} // namespace ghost_planners