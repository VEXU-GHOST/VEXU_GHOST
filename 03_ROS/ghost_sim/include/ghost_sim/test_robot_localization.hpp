/*
 * Filename: test_publisher_v5_actuator_cmd.hpp
 * Created Date: Friday December 29th 2023
 * Author: Melissa Cruz
 * Author Email: melissajecruz@utexas.edu
 *
 * Last Modified: Friday December 29th 2023 5:14 pm
 * Modified By: Melissa Cruz
 */

#ifndef GHOST_TEST_PUBLISHER_V5_ACTUATOR_CMD_HPP_
#define GHOST_TEST_PUBLISHER_V5_ACTUATOR_CMD_HPP_

#include <chrono>
#include <memory>
#include <vector>
#include <math.h>
#include <rclcpp/rclcpp.hpp>

#include "gazebo_msgs/msg/model_states.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float32.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

namespace ghost_sim {

class testRobotLocalization : public rclcpp::Node {
public:
	/// Constructor
	testRobotLocalization();

	void modelStatesCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg);
	rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr output_pub_;
	rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr gz_model_state_sub_;

	geometry_msgs::msg::Pose gz_model_state_input_;
};

}  // namespace test_publisher_v5_actuator_cmd

#endif  // GHOST_TEST_PUBLISHER_V5_ACTUATOR_CMD_HPP_