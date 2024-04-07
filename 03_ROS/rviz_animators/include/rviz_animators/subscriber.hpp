#pragma once

#include <memory>
#include "visualization_msgs/msg/marker.hpp"
// #include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rclcpp/rclcpp.hpp"



using std::placeholders::_1;

namespace rviz {

class AnimationSubscriber : public rclcpp::Node {
public:

	AnimationSubscriber();

	// void topic_callback(visualization_msgs::msg::Marker::SharedPtr msg);
	


private:
	// void topic_callback(std_msgs::msg::String::SharedPtr msg);
	void topic_callback(visualization_msgs::msg::Marker::SharedPtr msg);
	rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr subscription_;
};

}