#pragma once

#include <memory>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

namespace onboarding{
    class Subscriber : public rclcpp::Node {
    public:
    Subscriber();
    void topic_callback(const std_msgs::msg::Float32::SharedPtr msg);
    int add(int a, int b);
    int subtract(int a, int b);
    void do_nothing();


    private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
    };

   

}