#include "charlie_onboarding/ducker_subscriber.hpp"

#include <functional>
#include <memory>

using std::placeholders::_1;

MinimalSubscriber::MinimalSubscriber(const rclcpp::NodeOptions & options)
: Node("minimal_subscriber", options)
{
  subscription_ = create_subscription<std_msgs::msg::String>(
    "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
}

void MinimalSubscriber::topic_callback(const std_msgs::msg::String & msg)
{
  RCLCPP_INFO(get_logger(), "I heard: '%s'", msg.data.c_str());
  RCLCPP_INFO(get_logger(), "\n%s", rick_roll_.next().c_str());
}

#ifdef DUCKER_STANDALONE
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
#endif
