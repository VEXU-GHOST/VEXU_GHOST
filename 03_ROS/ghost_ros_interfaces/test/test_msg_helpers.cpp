#include <gtest/gtest.h>
#include "ghost_ros_interfaces/msg_helpers/msg_helpers.hpp"
#include "ghost_v5_interfaces/test/device_test_utils.hpp"

using namespace ghost_ros_interfaces::msg_helpers;
using namespace ghost_ros_interfaces;
using namespace ghost_v5_interfaces::devices;
using namespace ghost_v5_interfaces::test_util;
using namespace ghost_v5_interfaces;


TEST(TestMsgHelpers, testMotorStateMsg){
	auto motor_input = getRandomMotorData(false);
	auto msg = std::make_shared<ghost_msgs::msg::V5MotorState>();
	auto motor_output = std::make_shared<MotorDeviceData>("");

	// Convert to ROS Msg
	toROSMsg(*motor_input, *msg);
	fromROSMsg(*motor_output, *msg);

	EXPECT_EQ(*motor_input, *motor_output);
}

TEST(TestMsgHelpers, testMotorCommandMsg){
	auto motor_input = getRandomMotorData(true);
	auto msg = std::make_shared<ghost_msgs::msg::V5MotorCommand>();
	auto motor_output = std::make_shared<MotorDeviceData>("");

	// Convert to ROS Msg
	toROSMsg(*motor_input, *msg);
	fromROSMsg(*motor_output, *msg);

	EXPECT_EQ(*motor_input, *motor_output);
}

TEST(TestMsgHelpers, testRotationSensorStateMsg){
	auto rotation_input = getRandomRotationSensorData();
	auto msg = std::make_shared<ghost_msgs::msg::V5RotationSensorState>();
	auto rotation_output = std::make_shared<RotationSensorDeviceData>("");

	toROSMsg(*rotation_input, *msg);
	fromROSMsg(*rotation_output, *msg);

	EXPECT_EQ(*rotation_input, *rotation_output);
}

TEST(TestMsgHelpers, testInertialSensorStateMsg){
	auto inertial_input = getRandomInertialSensorData();
	auto msg = std::make_shared<ghost_msgs::msg::V5InertialSensorState>();
	auto inertial_output = std::make_shared<InertialSensorDeviceData>("");

	toROSMsg(*inertial_input, *msg);
	fromROSMsg(*inertial_output, *msg);

	EXPECT_EQ(*inertial_input, *inertial_output);
}

TEST(TestMsgHelpers, testJoystickStateMsg){
	auto joy_input = getRandomJoystickData();
	auto msg = std::make_shared<ghost_msgs::msg::V5JoystickState>();
	auto joy_output = std::make_shared<JoystickDeviceData>("");

	// Convert to ROS Msg
	toROSMsg(*joy_input, *msg);
	fromROSMsg(*joy_output, *msg);

	EXPECT_EQ(*joy_input, *joy_output);
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

TEST(TestMsgHelpers, testLabeledVectorMapConversion){
	std::unordered_map<std::string, std::vector<double> > expected_map{
		{"1", std::vector<double>{0.0}},
		{"2", std::vector<double>{1.0, 2.5}},
		{"3", std::vector<double>{1.18, 6.8}}
	};

	ghost_msgs::msg::LabeledVectorMap expected_msg;

	ghost_msgs::msg::LabeledVector e1;
	e1.label = "1";
	e1.data_array = std::vector<double>{0.0};
	expected_msg.entries.push_back(e1);

	ghost_msgs::msg::LabeledVector e2;
	e2.label = "2";
	e2.data_array = std::vector<double>{1.0, 2.5};
	expected_msg.entries.push_back(e2);

	ghost_msgs::msg::LabeledVector e3;
	e3.label = "3";
	e3.data_array = std::vector<double>{1.18, 6.8};
	expected_msg.entries.push_back(e3);

	std::unordered_map<std::string, std::vector<double> > blank_map;
	ghost_msgs::msg::LabeledVectorMap blank_msg;

	toROSMsg(expected_map, blank_msg);
	EXPECT_EQ(expected_msg.entries.size(), blank_msg.entries.size());

	// Maps are unordered, so need custom equality operator
	for(const auto& entry_1 : expected_msg.entries){
		int found_count = 0;
		// Ensure each entry in first msg has a corresponding unique entry in second msg
		for(const auto& entry_2 : blank_msg.entries){
			if(entry_1.label == entry_2.label){
				found_count += 1;
				EXPECT_EQ(entry_1.data_array, entry_2.data_array);
			}
		}
		EXPECT_EQ(found_count, 1);
	}

	fromROSMsg(blank_map, expected_msg);
	EXPECT_EQ(expected_map, blank_map);
}