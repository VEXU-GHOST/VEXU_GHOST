#include <iostream>
#include <ghost_util/angle_util.hpp>
#include <ghost_util/unit_conversion_utils.hpp>
#include <ghost_planners/bezier_curve.hpp>

#include <cmath>
#include <vector>

//class for BezierCurve control, to travel from point a to b, maintaining knowledge of location.

namespace ghost_planners
{

BezierCurve::BezierCurve()
{
  lead_ = 0.7;
}

// determine carrot point
void BezierCurve::find_carrot(Eigen::Vector3d cur_pos)
{
  float hyp = sqrt(pow(cur_pos.x() - end_x_, 2) + pow(cur_pos.y() - end_y_, 2));
  carrot1_x_ = cur_pos.x() + hyp * std::cos(cur_pos.z()) * lead_;
  carrot1_y_ = cur_pos.y() + hyp * std::sin(cur_pos.z()) * lead_;
  carrot2_x_ = end_x_ - hyp * std::cos(end_radians_) * lead_;
  carrot2_y_ = end_y_ - hyp * std::sin(end_radians_) * lead_;
}

// map the curve with 10 points
void BezierCurve::map_curve(Eigen::Vector3d cur_pos)
{
  find_carrot(cur_pos);
  points_.clear();

  float x_next;
  float y_next;

  for (float t = 0; t <= 1; t += 1.0/250.0) {
    x_next = (1 - t) * ((1 - t) * ((1 - t) * cur_pos.x() + t * carrot1_x_) + t * ((1 - t) * carrot1_x_ + t * carrot2_x_)) 
      + t * ((1 - t) * ((1 - t) * carrot1_x_ + t * carrot2_x_) + t * ((1 - t) * carrot2_x_ + t * end_x_));
    y_next = (1 - t) * ((1 - t) * ((1 - t) * cur_pos.y() + t * carrot1_y_) + t * ((1 - t) * carrot1_y_ + t * carrot2_y_)) 
      + t * ((1 - t) * ((1 - t) * carrot1_y_ + t * carrot2_y_) + t * ((1 - t) * carrot2_y_ + t * end_y_));
    points_.push_back({x_next, y_next});
  }
}

// sets the end point
void BezierCurve::set_end_point(float x, float y, float radians)
{
  end_x_ = x;
  end_y_ = y;
  end_radians_ = radians;
}

// sets the lead distance
void BezierCurve::set_lead(float lead)
{
  lead_ = lead;
}

// returns the points of the curve
std::vector<Eigen::Vector2d> BezierCurve::get_points()
{
  return points_;
}

}  // namespace ghost_planners