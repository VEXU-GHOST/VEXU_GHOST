#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>

namespace will_onboarding
{

class PubNode : public rclcpp::Node
{
public:
  PubNode();

  std::string create_hi_msg();
  std::string create_bye_msg();

private:
  void timer_callback();
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

class SubNode : public rclcpp::Node
{
public:
  SubNode();

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg);
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

} // namespace will_onboarding
