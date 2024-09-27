#include "linh-onboarding/subscriber.hpp"

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<onboarding::Subscriber>());
	rclcpp::shutdown();
	return 0;
}