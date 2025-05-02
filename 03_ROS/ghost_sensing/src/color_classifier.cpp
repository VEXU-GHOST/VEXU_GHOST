#include <ghost_sensing/color_classifier.hpp>

namespace ghost_sensing
{
ColorClassifier::ColorClassifier()
: Node("color_classifier", "",
    rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true)
)
{
  RCLCPP_INFO(
    this->get_logger(), "Starting Node ColorClassifier");


  load_color_parameters();
  // ROS Topics
  m_hsv_sub = this->create_subscription<std_msgs::msg::ColorRGBA>(
    "hsv", 10,
    std::bind(&ColorClassifier::callback, this, std::placeholders::_1));

  m_category_pub = this->create_publisher<std_msgs::msg::String>(
    "color", 10);
}


void ColorClassifier::load_color_parameters()
{
  // List all parameter names with the prefix "color_classification"
  auto param_list = this->list_parameters({"", "color_classification"}, 3);
  std::set<std::string> color_names;
  const std::string prefix = "color_classification.";

  // The parameters are flattened. For example:
  //   "color_classification.blue.hue_center"
  //   "color_classification.red.hue_range"
  // Extract the color names from these keys.
  for (const auto & full_name : param_list.names) {
    if (full_name.rfind(prefix, 0) == 0) { // starts with prefix
      std::string remainder = full_name.substr(prefix.size());
      auto dot_pos = remainder.find('.');
      if (dot_pos != std::string::npos) {
        std::string color = remainder.substr(0, dot_pos);
        color_names.insert(color);
      }
    }
  }

  RCLCPP_INFO(this->get_logger(), "Loading colors: ");

  // For each detected color, build the parameter keys and retrieve values.
  for (const auto & color : color_names) {
    ColorThresholds thresholds;
    std::string base = "color_classification." + color + ".";

    // Retrieve parameters. These calls assume that the parameters are declared and set.
    if (!this->has_parameter(base + "hue_center")) {
      RCLCPP_WARN(
        this->get_logger(), "Parameter %s not found, skipping color %s",
        (base + "hue_center").c_str(), color.c_str());
      continue;
    }
    thresholds.hue_center = this->get_parameter(base + "hue_center").as_int();
    thresholds.hue_range = this->get_parameter(base + "hue_range").as_int();
    thresholds.sat_min = this->get_parameter(base + "sat_min").as_double();
    thresholds.sat_max = this->get_parameter(base + "sat_max").as_double();
    thresholds.val_min = this->get_parameter(base + "val_min").as_double();
    thresholds.val_max = this->get_parameter(base + "val_max").as_double();

    m_color_map[color] = thresholds;

    RCLCPP_INFO(
      this->get_logger(),
      "Loaded color '%s': Hue(center=%d, range=%d), Saturation(%.2f-%.2f), Value(%.2f-%.2f)",
      color.c_str(),
      thresholds.hue_center, thresholds.hue_range,
      thresholds.sat_min, thresholds.sat_max,
      thresholds.val_min, thresholds.val_max);
  }
}


void ColorClassifier::callback(const std_msgs::msg::ColorRGBA::SharedPtr msg)
{
  // Map the ColorRGBA components to HSV:
  //   - r: Hue (in degrees [0, 360))
  //   - g: Saturation (in [0, 1])
  //   - b: Value (in [0, 1])
  //   - a: Ignored
  double h = msg->r;
  double s = msg->g;
  double v = msg->b;

  std::string matched_color = "unknown";

  for (const auto &[color, thresholds] : m_color_map) {
    bool hue_match = false;
    // Check for hue wrap-around cases at 0° and 360°
    if (thresholds.hue_center - thresholds.hue_range < 0 ||
      thresholds.hue_center + thresholds.hue_range >= 360)
    {
      // For wrap-around, adjust the lower bound.
      int lower_bound = (thresholds.hue_center - thresholds.hue_range + 360) % 360;
      int upper_bound = (thresholds.hue_center + thresholds.hue_range) % 360;
      hue_match = (h >= lower_bound) || (h <= upper_bound);
    } else {
      hue_match = (h >= thresholds.hue_center - thresholds.hue_range) &&
        (h <= thresholds.hue_center + thresholds.hue_range);
    }

    // Check saturation and value ranges.
    if (hue_match &&
      s >= thresholds.sat_min && s <= thresholds.sat_max &&
      v >= thresholds.val_min && v <= thresholds.val_max)
    {
      matched_color = color;
      break;
    }
  }

  // Publish the matched color as a string message.
  std_msgs::msg::String color_msg;
  color_msg.data = matched_color;
  m_category_pub->publish(color_msg);

  //RCLCPP_INFO(
  //  this->get_logger(), "HSV: (%.1f, %.2f, %.4f) -> Classified as: %s",
  //  h, s, v, matched_color.c_str());
}
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ghost_sensing::ColorClassifier>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
