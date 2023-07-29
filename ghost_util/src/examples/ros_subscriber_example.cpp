#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

using std::placeholders::_1;

class ROSSubscriberExample : public rclcpp::Node
{
  public:
    ROSSubscriberExample()
    : Node("latency_test_subscriber")
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "latency_test_topic", 10, std::bind(&ROSSubscriberExample::topic_callback, this, _1));
    }

  private:
    void topic_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
      auto now = this->get_clock()->now();
      auto diff = now - msg->header.stamp;
      RCLCPP_INFO(this->get_logger(), "%ld us", diff.nanoseconds()/1000);
    }
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ROSSubscriberExample>());
  rclcpp::shutdown();
  return 0;
}