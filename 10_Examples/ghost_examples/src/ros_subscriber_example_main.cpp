#include "ghost_examples/ros_subscriber_example.hpp"

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ghost_examples::ROSSubscriberExample>());
	rclcpp::shutdown();
	return 0;
}