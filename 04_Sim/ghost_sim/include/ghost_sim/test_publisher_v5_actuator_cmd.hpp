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

#ifndef GHOST_TEST_PUBLISHER_V5_ACTUATOR_CMD_HPP_
#define GHOST_TEST_PUBLISHER_V5_ACTUATOR_CMD_HPP_

#include <chrono>
#include <memory>
#include <vector>
#include <math.h>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/vector3.hpp"
#include "ghost_msgs/msg/v5_actuator_command.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace ghost_sim
{

class testPublisherV5ActuatorCmd : public rclcpp::Node
{
public:
  /// Constructor
  testPublisherV5ActuatorCmd();

  void publishData();
  rclcpp::Publisher<ghost_msgs::msg::V5ActuatorCommand>::SharedPtr output_pub_;
  std::array<std::string, 8> motors_{"DRIVE_LEFT_FRONT_LEFT_MOTOR",
    "DRIVE_LEFT_FRONT_RIGHT_MOTOR", "DRIVE_LEFT_BACK_LEFT_MOTOR", "DRIVE_LEFT_BACK_RIGHT_MOTOR",
    "DRIVE_RIGHT_FRONT_LEFT_MOTOR", "DRIVE_RIGHT_FRONT_RIGHT_MOTOR", "DRIVE_RIGHT_BACK_LEFT_MOTOR",
    "DRIVE_RIGHT_BACK_RIGHT_MOTOR"};
  ghost_msgs::msg::V5MotorCommand populateMotorCmd(const int loop_index);

  // private:
};

}  // namespace test_publisher_v5_actuator_cmd

#endif  // GHOST_TEST_PUBLISHER_V5_ACTUATOR_CMD_HPP_
