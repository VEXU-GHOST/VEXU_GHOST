#pragma once

#include <rclcpp/rclcpp.hpp>

#include <ghost_msgs/msg/ball_color.hpp>
#include <ghost_msgs/msg/color_sensor_state.hpp>

namespace ghost_sensing
{

// Classifies a colour sensor's reading as a red / blue ball (or none / unsure)
// from the raw R/B ratio, and republishes on <input_topic>/class. Thresholds
// come from parameters (shared via base_ros_config.yaml); the input topic is
// per-instance.
class BallColorClassifier : public rclcpp::Node
{
public:
  BallColorClassifier();

private:
  void callback(const ghost_msgs::msg::ColorSensorState::SharedPtr msg);

  rclcpp::Subscription<ghost_msgs::msg::ColorSensorState>::SharedPtr sub_;
  rclcpp::Publisher<ghost_msgs::msg::BallColor>::SharedPtr pub_;

  double red_rb_;          // R/B >= this -> RED
  double blue_rb_;         // R/B <= this -> BLUE
  int red_min_level_;      // a red-ratio reading with max(r,g,b) below this -> NONE
  int blue_min_level_;     // a blue-ratio reading with max(r,g,b) below this -> NONE
};

}  // namespace ghost_sensing
