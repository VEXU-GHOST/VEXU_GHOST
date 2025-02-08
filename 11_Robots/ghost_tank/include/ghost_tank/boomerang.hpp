#pragma once
#include <iostream>
#include <ghost_tank/tank_model.hpp>
#include <ghost_tank/tank_robot_plugin.hpp>
#include <ghost_util/angle_util.hpp>
#include <ghost_util/unit_conversion_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <eigen3/Eigen/Core>

#include <cmath>
#include <vector>

//class for Boomerang control, to travel from point a to b, maintaining knowledge of location.

namespace ghost_tank
{

class Boomerang {
public:
  // Boomerang(float lead);
  Boomerang(
  float lead, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_viz_pub, rclcpp::Node::SharedPtr node_ptr);


  //determine carrot point
  void find_carrot(Eigen::Vector3d cur_pos);
  void map_curve(Eigen::Vector3d cur_pos);

  void find_next_point(Eigen::Vector3d cur_pos);
  Eigen::Vector2d get_next_point(Eigen::Vector3d cur_pos);
  void set_end_point(float x, float y, float radians);
  void set_lead(float lead);


private:
  float end_x_;
  float end_y_;
  float end_radians_;

  float lead_;     //value of 0.00 - 1.00, how much you want to lead the curve.
  float carrot_x_;
  float carrot_y_;

  float next_point_x_;
  float next_point_y_;
  float next_theta_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_trajectory_viz_pub;
  rclcpp::Node::SharedPtr node_ptr_;

  struct XYD
  {
    float x;
    float y;
    //float radians;
  };

  std::vector<XYD> points_;
};

} // namespace ghost_tank