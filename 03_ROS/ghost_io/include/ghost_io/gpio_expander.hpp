#pragma once

#include "PCF8575.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int64.hpp>

#include <map>
#include <string>
#include <vector>
#include <set>  // Added to support std::set


namespace ghost_io
{
struct GPIODevice
{
  std::vector<int64_t> pins;
  bool output; // output to world
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub;
};

class GPIOExpander : public rclcpp::Node
{
  std::unique_ptr<PCF8575> chip;

  double m_poll_freq;

  std::map<std::string, GPIODevice> m_gpio_map;
  uint16_t output;

  rclcpp::TimerBase::SharedPtr m_publish_timer;

  void load_gpio_parameters();

public:
  GPIOExpander();


  void poll();
  void callback(const std_msgs::msg::Int64::SharedPtr in, std::string name);

};

}
