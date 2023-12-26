#include "will_onboarding1/Mather.hpp"

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Math_Space::Mather>(1,1));
	rclcpp::shutdown();
	return 0;
}