#pragma once

#include <rclcpp/rclcpp.hpp>

#include <ghost_msgs/msg/color_sensor_state.hpp>
#include <ghost_msgs/msg/toggle_color.hpp>

namespace ghost_sensing
{

// Classifies a colour sensor's reading as a red / blue / yellow toggle (or none)
// from the raw R/B and G/B ratios, and republishes on <input_topic>/class.
// Thresholds come from parameters (set per sensor in the robot's
// *_sensor_host_config.yaml); the input topic is per-instance.
class ToggleColorClassifier : public rclcpp::Node
{
public:
  ToggleColorClassifier();

private:
  void callback(const ghost_msgs::msg::ColorSensorState::SharedPtr msg);

  rclcpp::Subscription<ghost_msgs::msg::ColorSensorState>::SharedPtr sub_;
  rclcpp::Publisher<ghost_msgs::msg::ToggleColor>::SharedPtr pub_;

  double yellow_rb_;       // R/B >= this AND G/B >= yellow_gb -> YELLOW
  double yellow_gb_;
  double red_rb_;          // R/B >= this (and not yellow) -> RED
  double blue_rb_;         // R/B <= this -> BLUE
  int yellow_min_level_;   // a yellow-ratio reading with max(r,g,b) below this -> NONE
  int red_min_level_;      // a red-ratio reading with max(r,g,b) below this -> NONE
  int blue_min_level_;     // a blue-ratio reading with max(r,g,b) below this -> NONE
};

}  // namespace ghost_sensing
