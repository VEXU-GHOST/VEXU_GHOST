#include <ghost_sensing/toggle_color_classifier.hpp>

#include <algorithm>
#include <string>

namespace ghost_sensing
{

using ghost_msgs::msg::ColorSensorState;
using ghost_msgs::msg::ToggleColor;

ToggleColorClassifier::ToggleColorClassifier()
: Node("toggle_color_classifier")
{
  // Placeholder defaults -- tune per sensor in the robot's sensor host config.
  std::string input_topic = declare_parameter<std::string>("input_topic", "color");
  yellow_rb_ = declare_parameter<double>("yellow_rb", 1.4);
  yellow_gb_ = declare_parameter<double>("yellow_gb", 1.4);
  red_rb_ = declare_parameter<double>("red_rb", 1.4);
  blue_rb_ = declare_parameter<double>("blue_rb", 0.7);
  yellow_min_level_ = declare_parameter<int>("yellow_min_level", 1500);
  red_min_level_ = declare_parameter<int>("red_min_level", 1500);
  blue_min_level_ = declare_parameter<int>("blue_min_level", 1500);

  std::string output_topic = input_topic + "/class";

  // Match the sensor host publisher's SensorDataQoS (best effort).
  sub_ = create_subscription<ColorSensorState>(
    input_topic, rclcpp::SensorDataQoS(),
    std::bind(&ToggleColorClassifier::callback, this, std::placeholders::_1));
  pub_ = create_publisher<ToggleColor>(output_topic, 10);

  RCLCPP_INFO(get_logger(),
    "toggle_color_classifier: %s -> %s (yellow R/B>=%.2f G/B>=%.2f min_level=%d, "
    "red R/B>=%.2f min_level=%d, blue R/B<=%.2f min_level=%d)",
    input_topic.c_str(), output_topic.c_str(), yellow_rb_, yellow_gb_, yellow_min_level_,
    red_rb_, red_min_level_, blue_rb_, blue_min_level_);
}

void ToggleColorClassifier::callback(const ColorSensorState::SharedPtr msg)
{
  ToggleColor out;
  out.name = msg->name;

  int level = std::max({static_cast<int>(msg->r), static_cast<int>(msg->g),
      static_cast<int>(msg->b)});
  int b = std::max(1, static_cast<int>(msg->b));
  double rb = static_cast<double>(msg->r) / b;
  double gb = static_cast<double>(msg->g) / b;
  // Yellow reflects both red and green, so it is checked first: a red-ratio
  // reading that also has a high G/B is yellow, otherwise red. Each class has
  // its own brightness gate (max(r,g,b)), since the colours reflect different
  // amounts of light.
  if (rb >= yellow_rb_ && gb >= yellow_gb_) {
    out.color = (level >= yellow_min_level_) ? ToggleColor::YELLOW : ToggleColor::NONE;
  } else if (rb >= red_rb_) {
    out.color = (level >= red_min_level_) ? ToggleColor::RED : ToggleColor::NONE;
  } else if (rb <= blue_rb_) {
    out.color = (level >= blue_min_level_) ? ToggleColor::BLUE : ToggleColor::NONE;
  } else {
    out.color = ToggleColor::NONE;
  }
  pub_->publish(out);
}

}  // namespace ghost_sensing

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ghost_sensing::ToggleColorClassifier>());
  rclcpp::shutdown();
  return 0;
}
