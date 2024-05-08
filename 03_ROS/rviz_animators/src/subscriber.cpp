#include "rviz_animators/subscriber.hpp"
// #include "ghost_msgs/msg/LabeledVectorMap.msg"

namespace rviz {

AnimationSubscriber::AnimationSubscriber() :
	Node("animation_subscriber") {
	// subscription_ = this->create_subscription<visualization_msgs::msg::Marker>(
	// "animation_topic", 10, std::bind(&AnimationSubscriber::topic_callback, this, std::placeholders:: _1));
	// declare_parameter("mode", "blah blah blah");
	// std::string test_param = get_parameter("mode").as_string();

	// declare_parameter("bool_test", false);
	// bool bool_test = get_parameter("bool_test").as_bool();

	// RCLCPP_INFO(get_logger(), test_param.c_str());
	// if(bool_test){
	// 	RCLCPP_INFO(get_logger(), "yay");
	// }

	subscription_ = this->create_subscription<visualization_msgs::msg::Marker>(
		"topic", 10, std::bind(&AnimationSubscriber::topic_callback, this, std::placeholders::_1));
}

void AnimationSubscriber::topic_callback(visualization_msgs::msg::Marker::SharedPtr msg) {
	// RCLCPP_INFO(this->get_logger(), "Received marker message with ID %d", msg->id);
	// RCLCPP_INFO(this->get_logger(), "Point position: x = %f, y = %f", msg->pose.position.x, msg->pose.position.y);
}

} // End of namespace rviz