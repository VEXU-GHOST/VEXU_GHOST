#include <iostream>
#include <../include/ghost_tank/tank_model.hpp>
#include <ghost_tank/tank_robot_plugin.hpp>
#include <ghost_util/angle_util.hpp>
#include <ghost_util/unit_conversion_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
//#include <pluginlib/class_list_macros.hpp>

#include <cmath>
#include <vector>

//class for boomerang control, to travel from point a to b, maintaining knowledge of location.


class boomerang
{

private:
  double st_x;
  float st_y;
  float st_radians;

  float end_x;
  float end_y;
  float end_radians;

  float lead;     //value of 0.00 - 1.00, how much you want to lead the curve.
  float carrot_x;
  float carrot_y;

  float hyp = sqrt(pow(st_x - end_x, 2) + pow(st_y - end_y, 2));


  struct XYD
  {
    float x;
    float y;
    //float radians;
  };

  std::vector<XYD> points;

public:
  float cur_x;
  float cur_y;
  float cur_theta;

  float next_point_x;
  float next_point_y;
  float next_theta;

//determine carrot point
  void find_carrot()
  {
    carrot_x = end_x - hyp * std::sin(end_radians) * lead;
    carrot_y = end_y - hyp * std::cos(end_radians) * lead;
  }

  void map_curve()
  {
    float x_next;
    float y_next;
    //float r_next;
    for (int t = 0; t <= 1; t += 0.1) {
      x_next = (1 - t) * ((1 - t) * st_x + t * carrot_x) + t * ((1 - t) * carrot_x + t * end_x);
      y_next = (1 - t) * ((1 - t) * st_y + t * carrot_y) + t * ((1 - t) * carrot_y + t * end_y);

      points.push_back({x_next, y_next});
    }
  }

  void find_next_point()
  {

    //finds next angle the robot needs to be oriented in to travel to next point.

    float slope_y = points[1].y - st_y;
    float slope_x = points[1].x - st_x;
    //writes next point into public varibles, to be accessed in autonomous
    next_point_x = points[1].x;
    next_point_y = points[1].x;
    if (slope_x == 0) {
      next_theta = (slope_y >= 0) ? 3.141 / 2 : -3.141 / 2;
    } else {
      //this is the direction the robot must go
      next_theta = std::atan(std::abs(slope_y / slope_x));
    }

  }

  boomerang(
    double st_x, float st_y, float st_radians,
    float end_x, float end_y,
    float end_radians, float lead)
  {
    this->st_x = st_x;
    this->st_y = st_y;
    this->st_radians = st_radians;
    this->end_x = end_x;
    this->end_y = end_y;
    this->end_radians = end_radians;

    this->lead = lead;

    map_curve();
    find_next_point();


  }


};
