#pragma once
#include <memory>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

namespace ghost_examples {

class ROSSubscriberExample : public rclcpp::Node {
public:
	ROSSubscriberExample();
	void topic_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

	// These functions are for our GTest example, and have nothing to do with the ROS Subscriber :)
	int add_ints(int a, int b);
	float add_floats(float a, float b);
	void function_that_throws_error();
	void do_nothing();
private:
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
};

} // namespace ghost_examples