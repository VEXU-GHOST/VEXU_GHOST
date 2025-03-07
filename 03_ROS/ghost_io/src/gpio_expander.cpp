#include <ghost_io/gpio_expander.hpp>
#include <time.h>
#include <iostream>
#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace ghost_io
{
GPIOExpander::GPIOExpander()
: Node("gpio_expander", "",
    rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true)
)
{
  //declare_parameter("system_i2c_bus_path", "/dev/i2c-null"); // required argument
  declare_parameter("address", 0x20); // required argument
  declare_parameter("poll_frequency", 25.);

  auto i2c_bus_path = get_parameter("system_i2c_bus_path").as_string();
  auto address = get_parameter("address").as_int();
  m_poll_freq = get_parameter("poll_frequency").as_double();

  chip = std::make_unique<PCF8575>();
  if (!chip->open(i2c_bus_path, address)) {
    RCLCPP_ERROR(this->get_logger(), "CANNOT OPEN I2C BUS %s", i2c_bus_path.c_str());
    rclcpp::shutdown();
  }

  load_gpio_parameters();

  for (auto & device : m_gpio_map) {
    auto topic_name = device.first;
    if (device.second.output) {
    //  device.second.sub = this->create_subscription<std_msgs::msg::Int64>(
    //    topic_name, 10,
    //    std::bind(&GPIOExpander::callback, this, std::placeholders::_1, topic_name));
    std::cout << "subscribe to " << topic_name << std::endl;
        device.second.sub = this->create_subscription<std_msgs::msg::Int64>(
  topic_name, 10,
  [this, topic_name](const std_msgs::msg::Int64::SharedPtr msg) {
    this->callback(msg, topic_name);
  });


   } else {
    std::cout << "pub " << topic_name << std::endl;
       device.second.pub = this->create_publisher<std_msgs::msg::Int64>(
         topic_name, 10);
 
    }
  }

  poll();

  m_publish_timer =
    this->create_wall_timer(
    std::chrono::seconds(1) / m_poll_freq,
    std::bind(&GPIOExpander::poll, this));

  RCLCPP_INFO(this->get_logger(), "Init Finished");
}


void GPIOExpander::load_gpio_parameters()
{
  // List all parameter names with the prefix "gpio"
  auto param_list = this->list_parameters({"gpio"}, 4);

  //std::cout << param_list.names.size() << std::endl;
  //std::cout << param_list.prefixes.size() << std::endl;

  std::set<std::string> device_names;
  const std::string prefix = "gpio.";


  // The parameters are flattened. For example:
  //   "gpio_expander.gpio./io/led.pins"
  //   "gpio_expander.gpio./io/button.output"
  // Extract the device names from these keys.
  for (const auto & full_name : param_list.names) {
    //std::cout << full_name << std::endl;
    if (full_name.rfind(prefix, 0) == 0) { // starts with prefix
      std::string remainder = full_name.substr(prefix.size());
      auto dot_pos = remainder.find('.');
      if (dot_pos != std::string::npos) {
        std::string device = remainder.substr(0, dot_pos);
        device_names.insert(device);
      }
    }
  }
  for (const auto & prefix : param_list.prefixes) {

    //std::cout << prefix << std::endl;
  }

  RCLCPP_INFO(this->get_logger(), "Loading GPIO devices:");

  // For each detected device, build the parameter keys and retrieve values.
  for (const auto & device : device_names) {
    GPIODevice gpio_dev;

    std::string base = "gpio." + device + ".";

    // Retrieve pins parameter. Expecting an array of integers.
    if (!this->has_parameter(base + "pins")) {
      RCLCPP_WARN(
        this->get_logger(), "Parameter %s not found, skipping device %s",
        (base + "pins").c_str(), device.c_str());
      continue;
    }
    // Assume pins is stored as an integer array.
    gpio_dev.pins = this->get_parameter(base + "pins").as_integer_array();
    gpio_dev.output = this->get_parameter(base + "output").as_bool();

    m_gpio_map[device] = gpio_dev;

    // Build the pins string
    std::ostringstream oss;
    for (size_t i = 0; i < gpio_dev.pins.size(); ++i) {
      oss << gpio_dev.pins[i]
          << (i != gpio_dev.pins.size() - 1 ? ", " : "");
    }
    std::string pins_str = oss.str();

    // Log the information
    RCLCPP_INFO(
      this->get_logger(),
      "Loaded device '%s': pins = [%s], output = %s",
      device.c_str(),
      pins_str.c_str(),
      gpio_dev.output ? "true" : "false");
  }
}


void GPIOExpander::callback(const std_msgs::msg::Int64::SharedPtr in, std::string name)
{
  int64_t data = in->data;
  auto pins = m_gpio_map[name].pins;
  std::cout << name << " " << std::endl;
  for (int i = pins.size() - 1; i >= 0; i--) {
  std::cout << pins[i] << " " << std::to_string(data & 1) << std::endl;
    chip->setPin(pins[i], !(data & 1));
    data >>= 1;
  }
}

void GPIOExpander::poll()
{
  chip->poll();

  for (auto & device : m_gpio_map) {
    if (!device.second.output) { // input into the world
      int64_t data = 0;

      auto pins = device.second.pins;

      for (auto & p : pins) {
        bool value;
        chip->getPin(p, value);
        data <<= 1;
        data |= value;
      }

      std_msgs::msg::Int64 msg;
      msg.data = data;
      device.second.pub->publish(msg);
    }
  }
}


}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ghost_io::GPIOExpander>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
