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

int ROSSubscriberExample::add_ints(int a, int b){
	return a + b;
}

float ROSSubscriberExample::add_floats(float a, float b){
	return a + b;
}

void ROSSubscriberExample::function_that_throws_error(){
	throw(std::runtime_error("Yup, thats what this function does!"));
}

void ROSSubscriberExample::do_nothing(){
}

} // namespace ghost_examples