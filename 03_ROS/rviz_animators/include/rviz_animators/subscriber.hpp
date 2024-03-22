#pragma once

#include <memory>
#include "visualization_msgs/msg/marker.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

namespace rviz_animators {

class AnimationSubscriber : public rclcpp::Node {
public:

	AnimationSubscriber();

	void topic_callback(visualization_msgs::msg::Marker::SharedPtr msg);


protected:

private:

	rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr subscription_;
};

}