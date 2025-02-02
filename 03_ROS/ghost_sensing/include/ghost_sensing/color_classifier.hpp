#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>

#include <map>
#include <string>
#include <vector>
#include <set>  // Added to support std::set


namespace ghost_sensing
{
class ColorClassifier : public rclcpp::Node
{
  rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr m_hsv_sub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_category_pub;

  struct ColorThresholds
  {
    int hue_center;
    int hue_range;
    double sat_min;
    double sat_max;
    double val_min;
    double val_max;
  };
  std::map<std::string, ColorThresholds> m_color_map;


  void load_color_parameters();

public:
  ColorClassifier();
 


  void callback(const std_msgs::msg::ColorRGBA::SharedPtr color);

};

}
