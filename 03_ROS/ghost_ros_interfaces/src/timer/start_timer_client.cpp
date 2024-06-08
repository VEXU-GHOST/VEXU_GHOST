/*
 *   Copyright (c) 2024 Xander Wilson
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

#include "ghost_msgs/srv/start_timer.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("start_timer_client");
  int num_timers = 11;

  for (int i = 0; i < num_timers; i++) {
    rclcpp::Client<ghost_msgs::srv::StartTimer>::SharedPtr start_timer_client =
      node->create_client<ghost_msgs::srv::StartTimer>("start_timer");
    auto request = std::make_shared<ghost_msgs::srv::StartTimer::Request>();
    request->timer_name = "test_timer_" + std::to_string(i);
    request->duration_ns = 5UL * 1000000000UL;

    while (!start_timer_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          rclcpp::get_logger(
            "rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return 0;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }

    auto response = start_timer_client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, response) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service start_timer");
    }

    auto data = response.get();
    RCLCPP_INFO(
      rclcpp::get_logger("rclcpp"),
      std::string("success: " + std::to_string(data->success)).c_str());
  }

  rclcpp::shutdown();
}
