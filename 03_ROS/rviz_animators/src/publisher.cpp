#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "visualization_msgs/msg/marker.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class AnimationPublisher : public rclcpp::Node
{

  visualization_msgs::msg::Marker marker_msg_;

  public:
    AnimationPublisher()
    : Node("animation_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("topic", 10);

        marker_msg_.header.frame_id = "base_link";
        marker_msg_.header.stamp = this->now();
        marker_msg_.ns = "markers";
        marker_msg_.id = 0;
        marker_msg_.type = visualization_msgs::msg::Marker::ARROW;
        marker_msg_.action = visualization_msgs::msg::Marker::ADD;
        marker_msg_.pose.position.x = 0.0;
        marker_msg_.pose.position.y = 0.0;
        marker_msg_.pose.position.z = 0.0;
        marker_msg_.pose.orientation.x = 0.0;
        marker_msg_.pose.orientation.y = 0.0;
        marker_msg_.pose.orientation.z = 0.0;
        marker_msg_.pose.orientation.w = 1.0;
        marker_msg_.scale.x = 1.0;
        marker_msg_.scale.y = 0.1;
        marker_msg_.scale.z = 0.1;
        marker_msg_.color.a = 1.0; // Alpha
        marker_msg_.color.r = 1.0; // Red
        marker_msg_.color.g = 0.0; // Green
        marker_msg_.color.b = 0.0; // Blue

      timer_ = this->create_wall_timer(500ms, std::bind(&AnimationPublisher::timer_callback, this));
    }

  private:

    void timer_callback()
    {
      auto message = visualization_msgs::msg::Marker();
      // message.data = std::to_string(count_++);
      // RCLCPP_INFO(this->get_logger(), "Publishing: ", message);
      message.header.stamp = this->get_clock()->now();
      publisher_->publish(message);
    }

      void publishMarker()
    {
        publisher_->publish(marker_msg_);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AnimationPublisher>());
  rclcpp::shutdown();
  return 0;
}