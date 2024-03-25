#pragma once

#include <memory>
#include "visualization_msgs/msg/marker.hpp"
#include "rclcpp/rclcpp.hpp"


namespace rviz{
    class ROSSubscriber : public rclcpp::Node {
    public:
    ROSSubscriber();
    void topic_callback(const visualization_msgs::msg::Marker::SharedPtr msg);

    private:
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr subscription_;
    };
}
