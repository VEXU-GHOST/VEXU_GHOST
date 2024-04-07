#include "annie-onboarding/subscriber_member_function.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<annie_onboarding::MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
