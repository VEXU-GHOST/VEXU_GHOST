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
  std::array<std::string, 4> motors_{"DRIVE_LEFT_FRONT_LEFT_MOTOR",
                "DRIVE_RIGHT_FRONT_LEFT_MOTOR", 
                "DRIVE_LEFT_BACK_LEFT_MOTOR",
                "DRIVE_RIGHT_BACK_LEFT_MOTOR"};
  ghost_msgs::msg::V5MotorCommand populateMotorCmd(const int loop_index);

  // private:
};

}  // namespace test_publisher_v5_actuator_cmd

#endif  // GHOST_TEST_PUBLISHER_V5_ACTUATOR_CMD_HPP_