#include "rviz_animators/subscriber.hpp"

namespace rviz {

ROSSubscriber::ROSSubscriber() :
    Node("marker_subscriber"){
	subscription_ = this->create_subscription<visualization_msgs::msg::Marker>(
		"marker_topic", 10, std::bind(&ROSSubscriber::topic_callback, this, std::placeholders::_1));
}

void ROSSubscriber::topic_callback(const visualization_msgs::msg::Marker::SharedPtr msg){
	auto now = this->get_clock()->now();
	auto diff = now - msg->header.stamp;
	RCLCPP_INFO(this->get_logger(), "%ld us", diff.nanoseconds() / 1000);
}
}
