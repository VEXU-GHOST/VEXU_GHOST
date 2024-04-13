#include "rviz_animators/subscriber.hpp"
// #include "ghost_msgs/msg/LabeledVectorMap.msg"

namespace rviz {

AnimationSubscriber::AnimationSubscriber() :
	Node("animation_subscriber"){
	// subscription_ = this->create_subscription<visualization_msgs::msg::Marker>(
	// "animation_topic", 10, std::bind(&AnimationSubscriber::topic_callback, this, std::placeholders:: _1));
	declare_parameter("mode", "default");
	std::string test_param = get_parameter("mode").as_string();
	
	RCLCPP_INFO(get_logger(), test_param.c_str());

	subscription_ = this->create_subscription<visualization_msgs::msg::Marker>(
    "topic", 10, std::bind(&AnimationSubscriber::topic_callback, this, std::placeholders:: _1));
}

// void AnimationSubscriber::topic_callback(visualization_msgs::msg::Marker::SharedPtr msg){
// 	RCLCPP_INFO(this->get_logger(), "Received marker message with ID %d", msg->id);
//     }
// }

void AnimationSubscriber::topic_callback(visualization_msgs::msg::Marker::SharedPtr msg) {
    // Process the received message here
    // for (auto& pair : msg->data) {
    //     std::string& name = pair.first;
    //     std::vector<double>& values = pair.second;
        // Process the named time series as needed
        // For example, print the name and the values


        RCLCPP_INFO(rclcpp::get_logger("subscriber"), "Name: %s", msg->name.c_str());
        // RCLCPP_INFO(rclcpp::get_logger("subscriber"), "Value: %f", msg->values);
//     }
}



// void AnimationSubscriber::topic_callback(visualization_msgs::msg::Marker::SharedPtr msg){
// 	// RCLCPP_INFO(this->get_logger(), "Received marker message with ID %d", msg->id);
// 	RCLCPP_INFO(this->get_logger(), "Point position: x = %f, y = %f", msg->pose.position.x, msg->pose.position.y);
// }

// void modeSwitchCallback(visualization_msgs::msg::Marker::SharedPtr response){
// 	RCLCPP_INFO(this-`>get_logger(), "please work", response);
// }

} 