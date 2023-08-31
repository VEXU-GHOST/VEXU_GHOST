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

private:
	void topic_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
};

} // namespace ghost_examples