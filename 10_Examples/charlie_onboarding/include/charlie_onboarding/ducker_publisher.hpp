#ifndef CHARLIE_ONBOARDING__DUCKER_PUBLISHER_HPP_
#define CHARLIE_ONBOARDING__DUCKER_PUBLISHER_HPP_

#include "charlie_onboarding/long_amogus.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalPublisher : public rclcpp::Node
{
public:
  explicit MinimalPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  LongAmogus amogus_;
};

#endif  // CHARLIE_ONBOARDING__DUCKER_PUBLISHER_HPP_
