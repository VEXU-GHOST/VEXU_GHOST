#include <ghost_sensing/ball_color_classifier.hpp>

#include <algorithm>
#include <string>

namespace ghost_sensing
{

using ghost_msgs::msg::BallColor;
using ghost_msgs::msg::ColorSensorState;

BallColorClassifier::BallColorClassifier()
: Node("ball_color_classifier")
{
  std::string input_topic = declare_parameter<std::string>("input_topic", "color");
  red_rb_ = declare_parameter<double>("red_rb", 1.4);
  blue_rb_ = declare_parameter<double>("blue_rb", 0.7);
  min_level_ = declare_parameter<int>("min_level", 1500);

  std::string output_topic = input_topic + "/class";

  // Match the sensor host publisher's SensorDataQoS (best effort).
  sub_ = create_subscription<ColorSensorState>(
    input_topic, rclcpp::SensorDataQoS(),
    std::bind(&BallColorClassifier::callback, this, std::placeholders::_1));
  pub_ = create_publisher<BallColor>(output_topic, 10);

  RCLCPP_INFO(get_logger(),
    "ball_color_classifier: %s -> %s (red R/B>=%.2f, blue R/B<=%.2f, min_level=%d)",
    input_topic.c_str(), output_topic.c_str(), red_rb_, blue_rb_, min_level_);
}

void BallColorClassifier::callback(const ColorSensorState::SharedPtr msg)
{
  BallColor out;
  out.name = msg->name;

  int level = std::max({static_cast<int>(msg->r), static_cast<int>(msg->g),
      static_cast<int>(msg->b)});
  if (level < min_level_) {
    out.color = BallColor::NONE;
  } else {
    // Green is ignored; red vs blue separates cleanly on the R/B ratio.
    double rb = static_cast<double>(msg->r) / std::max(1, static_cast<int>(msg->b));
    if (rb >= red_rb_) {
      out.color = BallColor::RED;
    } else if (rb <= blue_rb_) {
      out.color = BallColor::BLUE;
    } else {
      out.color = BallColor::UNSURE;
    }
  }
  pub_->publish(out);
}

}  // namespace ghost_sensing

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ghost_sensing::BallColorClassifier>());
  rclcpp::shutdown();
  return 0;
}
