#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class AmyPubSubNode : public rclcpp::Node
{
public:
  AmyPubSubNode()
  : Node("amy_pubsub_node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>(
      "amy_output", 10);

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "amy_input",
      10,
      [this](const std_msgs::msg::String::SharedPtr msg)
      {
        RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());

        std_msgs::msg::String output;
        output.data = "Amy received: " + msg->data;

        publisher_->publish(output);
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AmyPubSubNode>());
  rclcpp::shutdown();

  return 0;
}