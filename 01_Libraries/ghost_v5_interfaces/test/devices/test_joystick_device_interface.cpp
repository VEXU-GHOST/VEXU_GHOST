#include "ghost_v5_interfaces/devices/joystick_device_interface.hpp"
#include "ghost_v5_interfaces/test/device_test_utils.hpp"

#include <stdlib.h>
#include "gtest/gtest.h"

using ghost_v5_interfaces::JoystickDeviceData;
using namespace ghost_v5_interfaces::test_utils;

TEST(TestJoystickDeviceInterface, testEqualityOperator){
	auto j1 = getRandomJoystickData();
	auto j2 = std::make_shared<JoystickDeviceData>(*j1);
	EXPECT_EQ(*j1, *j2);
}

TEST(TestJoystickDeviceInterface, testSerializationCoprocessorToV5){
	auto j1 = getRandomJoystickData();
	auto j2 = std::make_shared<JoystickDeviceData>();
	auto j3 = std::make_shared<JoystickDeviceData>();
	j2->name = j1->name;
	j3->name = j1->name;

	// Test Serialization from Coprocessor -> V5
	// We expect this to do nothing, joysticks dont go from Coprocessor to V5
	auto serial_stream_1 = j1->serialize(true);
	j2->deserialize(serial_stream_1, true);

	EXPECT_EQ(*j2, *j3);
}

TEST(TestJoystickDeviceInterface, testSerializationV5ToCoprocessor){
	auto j1 = getRandomJoystickData();
	auto j2 = std::make_shared<JoystickDeviceData>();
	j2->name = j1->name;

	// Test Serialization from Coprocessor -> V5
	auto serial_stream_1 = j1->serialize(false);
	j2->deserialize(serial_stream_1, false);

	EXPECT_EQ(j2->name, j1->name);
	EXPECT_EQ(j2->type, j1->type);
	EXPECT_EQ(j2->left_x, j1->left_x);
	EXPECT_EQ(j2->left_y, j1->left_y);
	EXPECT_EQ(j2->right_x, j1->right_x);
	EXPECT_EQ(j2->right_y, j1->right_y);
	EXPECT_EQ(j2->btn_a, j1->btn_a);
	EXPECT_EQ(j2->btn_b, j1->btn_b);
	EXPECT_EQ(j2->btn_x, j1->btn_x);
	EXPECT_EQ(j2->btn_y, j1->btn_y);
	EXPECT_EQ(j2->btn_r1, j1->btn_r1);
	EXPECT_EQ(j2->btn_r2, j1->btn_r2);
	EXPECT_EQ(j2->btn_l1, j1->btn_l1);
	EXPECT_EQ(j2->btn_l2, j1->btn_l2);
	EXPECT_EQ(j2->btn_u, j1->btn_u);
	EXPECT_EQ(j2->btn_l, j1->btn_l);
	EXPECT_EQ(j2->btn_r, j1->btn_r);
	EXPECT_EQ(j2->btn_d, j1->btn_d);
	EXPECT_EQ(j2->is_master, j1->is_master);

	EXPECT_EQ(*j1, *j2);
}

TEST(TestJoystickDeviceInterface, testClone){
	auto data_1 = getRandomJoystickData();
	auto data_2 = data_1->clone()->as<JoystickDeviceData>();
	EXPECT_EQ(*data_1, *data_2);
}