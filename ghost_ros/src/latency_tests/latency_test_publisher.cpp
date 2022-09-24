#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class LatencyTestPublisher : public rclcpp::Node
{
  public:
    LatencyTestPublisher()
    : Node("latency_test_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("latency_test_topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&LatencyTestPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = sensor_msgs::msg::LaserScan();
      message.header.stamp = this->get_clock()->now();
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LatencyTestPublisher>());
  rclcpp::shutdown();
  return 0;
}