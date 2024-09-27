#pragma once

#include <memory>
#include "visualization_msgs/msg/marker_array.hpp"
// #include "ghost_msgs/msg/labeled_vector.hpp"
// #include "ghost_msgs/msg/labeled_vector_map.hpp"
#include "rclcpp/rclcpp.hpp"


namespace rviz{
    class ROSSubscriber : public rclcpp::Node {
    public:
    ROSSubscriber();
    void topic_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    // void vector_topic_callback(const ghost_msgs::msg::LabeledVectorMap::SharedPtr msg);

    private:
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscription_;
    // rclcpp::Subscription<ghost_msgs::msg::LabeledVectorMap>::SharedPtr vector_subscription_;
    };
}
