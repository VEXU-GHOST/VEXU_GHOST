#include "push_back_cv/msg/field_block_array.hpp"
#include "rclcpp/rclcpp.hpp"

class GoalReaderNode : public rclcpp::Node {
public:
  GoalReaderNode() : Node("goal_reader") {
    sub_ = create_subscription<push_back_cv::msg::FieldBlockArray>(
      "/field/blocks", 10,
      [this](const push_back_cv::msg::FieldBlockArray::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "got %zu blocks", msg->blocks.size());
      });
    RCLCPP_INFO(get_logger(), "Starting goal_reader");
  }
private:
  rclcpp::Subscription<push_back_cv::msg::FieldBlockArray>::SharedPtr sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalReaderNode>());
  rclcpp::shutdown();
  return 0;
}