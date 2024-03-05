
#pragma once

#include <memory>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;


namespace annie_onboarding{
class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber();

    int add_ints(int a, int b);
    float add_floats(float a, float b);
    // bool comparing_strings(std::string a, std::string b);

  private:
    void topic_callback(std_msgs::msg::String::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
}
