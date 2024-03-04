#pragma once

#include <memory>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

namespace myonboarding{
    class TrivMethodSubscriber : public rclcpp::Node {
    public:
    TrivMethodSubscriber();
    void topic_callback(const std_msgs::msg::String::SharedPtr msg);
     float add_floats(float a, float b);


    private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    };

   

}