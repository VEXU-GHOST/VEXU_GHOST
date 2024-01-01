#include "zaara-onboarding/trivialmethodssubscriber.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<myonboarding::TrivMethodSubscriber>());
    rclcpp::shutdown();
    return 0;
}