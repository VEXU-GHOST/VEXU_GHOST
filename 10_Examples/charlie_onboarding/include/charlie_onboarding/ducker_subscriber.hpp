#ifndef CHARLIE_ONBOARDING__DUCKER_SUBSCRIBER_HPP_
#define CHARLIE_ONBOARDING__DUCKER_SUBSCRIBER_HPP_

#include "charlie_onboarding/rick_roll.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
  explicit MinimalSubscriber(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void topic_callback(const std_msgs::msg::String & msg);
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  RickRoll rick_roll_;
};

#endif  // CHARLIE_ONBOARDING__DUCKER_SUBSCRIBER_HPP_
