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
	auto motor_output = std::make_shared<MotorDeviceData>("");

	// Convert to ROS Msg
	toROSMsg(*motor_input, *msg);
	fromROSMsg(*motor_output, *msg);

	EXPECT_EQ(*motor_input, *motor_output);
}

TEST(TestDeviceInterfaces, testMotorCommandMsg){
	auto motor_input = getRandomMotorData(true);
	auto msg = std::make_shared<ghost_msgs::msg::V5MotorCommand>();
	auto motor_output = std::make_shared<MotorDeviceData>("");

	// Convert to ROS Msg
	toROSMsg(*motor_input, *msg);
	fromROSMsg(*motor_output, *msg);

	EXPECT_EQ(*motor_input, *motor_output);
}

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

TEST(TestDeviceInterfaces, testRotationSensorStateMsg){
	auto rotation_input = getRandomRotationSensorData();
	auto msg = std::make_shared<ghost_msgs::msg::V5RotationSensorState>();
	auto rotation_output = std::make_shared<RotationSensorDeviceData>("");

	toROSMsg(*rotation_input, *msg);
	fromROSMsg(*rotation_output, *msg);

	EXPECT_EQ(*rotation_input, *rotation_output);
}

TEST(TestDeviceInterfaces, testInertialSensorStateMsg){
	auto inertial_input = getRandomInertialSensorData();
	auto msg = std::make_shared<ghost_msgs::msg::V5InertialSensorState>();
	auto inertial_output = std::make_shared<InertialSensorDeviceData>("");

	toROSMsg(*inertial_input, *msg);
	fromROSMsg(*inertial_output, *msg);

	EXPECT_EQ(*inertial_input, *inertial_output);
}

TEST(TestDeviceInterfaces, testJoystickStateMsg){
	auto joy_input = getRandomJoystickData();
	auto msg = std::make_shared<ghost_msgs::msg::V5JoystickState>();
	auto joy_output = std::make_shared<JoystickDeviceData>("");

	// Convert to ROS Msg
	toROSMsg(*joy_input, *msg);
	fromROSMsg(*joy_output, *msg);

	EXPECT_EQ(*joy_input, *joy_output);
}


TEST(TestDeviceInterfaces, testTrajectoryMsg){
	auto mt_input = std::make_shared<ghost_planners::RobotTrajectory::Trajectory>();
	mt_input->position_vector.push_back(0);

	auto mt_output = std::make_shared<ghost_planners::RobotTrajectory::Trajectory>();
	mt_output->position_vector.push_back(0);

	auto msg = std::make_shared<ghost_msgs::msg::Trajectory>();

	// Convert to ROS Msg
	toROSMsg(*mt_input, *msg);
	fromROSMsg(*mt_output, *msg);

	EXPECT_EQ(*mt_input, *mt_output);
}

TEST(TestDeviceInterfaces, testRobotTrajectoryMsg){
	auto rt_input = std::make_shared<ghost_planners::RobotTrajectory>();
	auto mt_input = std::make_shared<ghost_planners::RobotTrajectory::Trajectory>();
	mt_input->position_vector.push_back(0);
	rt_input->x_trajectory = *mt_input;


	auto msg = std::make_shared<ghost_msgs::msg::RobotTrajectory>();
	auto rt_output = std::make_shared<ghost_planners::RobotTrajectory>();

	// Convert to ROS Msg
	toROSMsg(*rt_input, *msg);
	fromROSMsg(*rt_output, *msg);

	EXPECT_EQ(*rt_input, *rt_output);
}