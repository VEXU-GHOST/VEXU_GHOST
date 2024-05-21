#include <gtest/gtest.h>
#include "ghost_v5_interfaces/devices/device_config_map.hpp"
#include "ghost_v5_interfaces/devices/joystick_device_interface.hpp"
#include "ghost_v5_interfaces/devices/motor_device_interface.hpp"
#include "ghost_v5_interfaces/devices/rotation_sensor_device_interface.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"
#include "ghost_v5_interfaces/test/device_test_utils.hpp"

#include "ghost_ros_interfaces/msg_helpers/msg_helpers.hpp"

using namespace ghost_ros_interfaces::msg_helpers;
using namespace ghost_ros_interfaces;
using namespace ghost_v5_interfaces::devices;
using namespace ghost_v5_interfaces::test_util;
using namespace ghost_v5_interfaces;

TEST(TestDeviceInterfaces, testMotorCommandDoesntSetStateData){
	auto motor_input = getRandomMotorData(false); // This gets state/sensor data, not command data.
	auto msg = std::make_shared<ghost_msgs::msg::V5MotorCommand>();
	auto motor_output = std::make_shared<MotorDeviceData>("");

	// Convert to ROS Msg
	toROSMsg(*motor_input, *msg);
	fromROSMsg(*motor_output, *msg);

	// Sensor data should not be propogated when given a command msg, thus, we expect these are not equal.
	EXPECT_FALSE(*motor_input == *motor_output);
}

TEST(TestDeviceInterfaces, testMotorStateDoesntSetCommandData){
	auto motor_input = getRandomMotorData(true); // This gets command data, not sensor/state data.
	auto msg = std::make_shared<ghost_msgs::msg::V5MotorState>();
	auto motor_output = std::make_shared<MotorDeviceData>("");

	// Convert to ROS Msg
	toROSMsg(*motor_input, *msg);
	fromROSMsg(*motor_output, *msg);

	// Command data should not be propogated when given a state msg, thus, we expect these are not equal.
	EXPECT_FALSE(*motor_input == *motor_output);
}

TEST(TestDeviceInterfaces, testRobotTrajectoryCallback){
	auto rt_input = std::make_shared<ghost_planners::RobotTrajectory>();
	rt_input->motor_names.push_back("motor");
	auto mt_input = std::make_shared<ghost_planners::RobotTrajectory::MotorTrajectory>();
	mt_input->position_vector.push_back(0);
	rt_input->motor_trajectories.push_back(*mt_input);

	auto msg = std::make_shared<ghost_msgs::msg::RobotTrajectory>();

	std::unordered_map<std::string, ghost_planners::RobotTrajectory::MotorTrajectory> trajectory_motor_map_, expected_map;
	expected_map["motor"] = *mt_input;

	toROSMsg(*rt_input, *msg);

	// trajectory_start_time_ = getTimeFromStart();
	for(int i = 0; i < msg->motor_names.size(); i++){
		auto motor_trajectory = std::make_shared<ghost_planners::RobotTrajectory::MotorTrajectory>();
		fromROSMsg(*motor_trajectory, msg->trajectories[i]);
		trajectory_motor_map_[msg->motor_names[i]] = *motor_trajectory;
	}

	// Convert to ROS Msg
	// toROSMsg(*rt_input, *msg);
	// fromROSMsg(*rt_output, *msg);

	EXPECT_EQ(trajectory_motor_map_, expected_map);
}