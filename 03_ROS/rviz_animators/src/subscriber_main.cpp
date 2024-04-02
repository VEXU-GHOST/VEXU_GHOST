#include "rviz_animators/subscriber.hpp"

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<rviz::AnimationSubscriber>());
	rclcpp::shutdown();
	return 0;
}
