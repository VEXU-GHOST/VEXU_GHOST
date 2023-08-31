#include "ghost_examples/ros_subscriber_example.hpp"

namespace ghost_examples {

ROSSubscriberExample::ROSSubscriberExample() :
	Node("latency_test_subscriber"){
	subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
		"latency_test_topic", 10, std::bind(&ROSSubscriberExample::topic_callback, this, _1));
}

void ROSSubscriberExample::topic_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
	auto now = this->get_clock()->now();
	auto diff = now - msg->header.stamp;
	RCLCPP_INFO(this->get_logger(), "%ld us", diff.nanoseconds() / 1000);
}

} // namespace ghost_examples