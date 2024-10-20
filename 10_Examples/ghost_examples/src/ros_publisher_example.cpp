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

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class ROSPublisherExample : public rclcpp::Node
{
public:
  ROSPublisherExample()
  : Node("latency_test_publisher"),
    count_(0)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "latency_test_topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&ROSPublisherExample::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = geometry_msgs::msg::PoseWithCovarianceStamped();
    message.header.stamp = this->get_clock()->now();
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ROSPublisherExample>());
  rclcpp::shutdown();
  return 0;
}
