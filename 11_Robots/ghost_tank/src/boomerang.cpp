#include <iostream>
#include <ghost_tank/tank_model.hpp>
#include <ghost_tank/tank_robot_plugin.hpp>
#include <ghost_util/angle_util.hpp>
#include <ghost_util/unit_conversion_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <ghost_tank/boomerang.hpp>
//#include <pluginlib/class_list_macros.hpp>

#include <cmath>
#include <vector>

//class for Boomerang control, to travel from point a to b, maintaining knowledge of location.

namespace ghost_tank
{

Boomerang::Boomerang(
  float lead, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_viz_pub, rclcpp::Node::SharedPtr node_ptr):
  lead_(lead),
  m_trajectory_viz_pub(trajectory_viz_pub),
  node_ptr_(node_ptr)
{
}

// determine carrot point
void Boomerang::find_carrot(Eigen::Vector3d cur_pos)
{
  float hyp = sqrt(pow(cur_pos.x() - end_x_, 2) + pow(cur_pos.y() - end_y_, 2));
  carrot_x_ = end_x_ - hyp * std::sin(end_radians_) * lead_;
  carrot_y_ = end_y_ - hyp * std::cos(end_radians_) * lead_;
}

// map the curve with 10 points
void Boomerang::map_curve(Eigen::Vector3d cur_pos)
{
  find_carrot(cur_pos);
  points_.clear();

  float x_next;
  float y_next;

  auto msg = visualization_msgs::msg::MarkerArray{};

  auto marker_msg = visualization_msgs::msg::Marker{};
  marker_msg.header.frame_id = "map";
  marker_msg.header.stamp = node_ptr_->get_clock()->now();
  marker_msg.id = 0;
  marker_msg.action = 0;
  marker_msg.type = 8; // points type
  marker_msg.scale.x = 0.1;
  marker_msg.scale.y = 0.1;
  marker_msg.scale.z = 0.1;
  marker_msg.color.a = 1;
  for (float t = 0; t <= 1; t += 0.1) {
    x_next = (1 - t) * ((1 - t) * cur_pos.x() + t * carrot_x_) + t * ((1 - t) * carrot_x_ + t * end_x_);
    y_next = (1 - t) * ((1 - t) * cur_pos.y() + t * carrot_y_) + t * ((1 - t) * carrot_y_ + t * end_y_);

    geometry_msgs::msg::Point p0{};
    p0.x = x_next;
    p0.y = y_next;
    p0.z = 0.0;
    marker_msg.points.push_back(p0);
    points_.push_back({x_next, y_next});
  }
  msg.markers.push_back(marker_msg);
  m_trajectory_viz_pub->publish(msg);
}

// sets the end point
void Boomerang::set_end_point(float x, float y, float radians)
{
  end_x_ = x;
  end_y_ = y;
  end_radians_ = radians;
}

// sets the lead distance
void Boomerang::set_lead(float lead)
{
  lead_ = lead;
}

// returns the points of the curve
std::vector<Eigen::Vector2d> Boomerang::get_points()
{
  return points_;
}

}  // namespace ghost_tank