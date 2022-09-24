#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
using std::placeholders::_1;

class LatencyTestSubscriber : public rclcpp::Node
{
  public:
    LatencyTestSubscriber()
    : Node("latency_test_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "latency_test_topic", 10, std::bind(&LatencyTestSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
      auto now = this->get_clock()->now();
      auto diff = now - msg->header.stamp;
      RCLCPP_INFO(this->get_logger(), "%d us", diff.nanoseconds()/1000);
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LatencyTestSubscriber>());
  rclcpp::shutdown();
  return 0;
}