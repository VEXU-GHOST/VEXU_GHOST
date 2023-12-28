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

TEST(TestDeviceInterfaces, testMotorStateMsg){
	auto motor_input = getRandomMotorData(false);
	auto msg = std::make_shared<ghost_msgs::msg::V5MotorState>();
	auto motor_output = std::make_shared<MotorDeviceData>();

	// Convert to ROS Msg
	toROSMsg(*motor_input, *msg);
	fromROSMsg(*msg, *motor_output);

	EXPECT_EQ(*motor_input, *motor_output);
}

TEST(TestDeviceInterfaces, testMotorCommandMsg){
	auto motor_input = getRandomMotorData(true);
	auto msg = std::make_shared<ghost_msgs::msg::V5MotorCommand>();
	auto motor_output = std::make_shared<MotorDeviceData>();

	// Convert to ROS Msg
	toROSMsg(*motor_input, *msg);
	fromROSMsg(*msg, *motor_output);

	EXPECT_EQ(*motor_input, *motor_output);
}

TEST(TestDeviceInterfaces, testMotorCommandDoesntSetStateData){
	auto motor_input = getRandomMotorData(false); // This gets state/sensor data, not command data.
	auto msg = std::make_shared<ghost_msgs::msg::V5MotorCommand>();
	auto motor_output = std::make_shared<MotorDeviceData>();

	// Convert to ROS Msg
	toROSMsg(*motor_input, *msg);
	fromROSMsg(*msg, *motor_output);

	// Sensor data should not be propogated when given a command msg, thus, we expect these are not equal.
	EXPECT_FALSE(*motor_input == *motor_output);
}

TEST(TestDeviceInterfaces, testMotorStateDoesntSetCommandData){
	auto motor_input = getRandomMotorData(true); // This gets command data, not sensor/state data.
	auto msg = std::make_shared<ghost_msgs::msg::V5MotorState>();
	auto motor_output = std::make_shared<MotorDeviceData>();

	// Convert to ROS Msg
	toROSMsg(*motor_input, *msg);
	fromROSMsg(*msg, *motor_output);

	// Command data should not be propogated when given a state msg, thus, we expect these are not equal.
	EXPECT_FALSE(*motor_input == *motor_output);
}

TEST(TestDeviceInterfaces, testRotationSensorStateMsg){
	auto rotation_input = getRandomRotationSensorData();
	auto msg = std::make_shared<ghost_msgs::msg::V5RotationSensorState>();
	auto rotation_output = std::make_shared<RotationSensorDeviceData>();

	toROSMsg(*rotation_input, *msg);
	fromROSMsg(*msg, *rotation_output);

	EXPECT_EQ(*rotation_input, *rotation_output);
}

TEST(TestDeviceInterfaces, testJoystickStateMsg){
	auto joy_input = getRandomJoystickData(false);
	auto msg = std::make_shared<ghost_msgs::msg::V5JoystickState>();
	auto joy_output = std::make_shared<JoystickDeviceData>();

	// Convert to ROS Msg
	toROSMsg(*joy_input, *msg);
	fromROSMsg(*msg, *joy_output);

	EXPECT_EQ(*joy_input, *joy_output);
}