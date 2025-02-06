/*
 *   Copyright (c) 2024 Maxx Wilson
 *   All rights reserved.

 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ghost_msgs/msg/v5_sensor_update.hpp"

namespace sensor_update_spoofer
{

class SensorUpdateSpoofer : public rclcpp::Node
{
public:
  SensorUpdateSpoofer()
  : rclcpp::Node("sensor_update_spoofer")
  {
    sensor_update_publisher = this->create_publisher<ghost_msgs::msg::V5SensorUpdate>(
      "/v5/sensor_update",
      10);

    auto publish_message =
      [this]() -> void
      {
        auto msg = ghost_msgs::msg::V5SensorUpdate();
        RCLCPP_INFO(this->get_logger(), "Publishing: sensor update");

        // Put the message into a queue to be processed by the middleware.
        // This call is non-blocking.
        sensor_update_publisher->publish(msg);
      };

    this->create_wall_timer(
      std::chrono::milliseconds(10),
      publish_message);
  }
private:
  rclcpp::Publisher<ghost_msgs::msg::V5SensorUpdate>::SharedPtr sensor_update_publisher;
};

} // namespace sensor_update_spoofer

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sensor_update_spoofer::SensorUpdateSpoofer>());
  rclcpp::shutdown();
  return 0;
}
