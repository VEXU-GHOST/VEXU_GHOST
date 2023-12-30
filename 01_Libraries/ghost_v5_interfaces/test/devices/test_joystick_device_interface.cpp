#include "ghost_v5_interfaces/devices/joystick_device_interface.hpp"
#include "ghost_v5_interfaces/test/device_test_utils.hpp"

#include <stdlib.h>
#include "gtest/gtest.h"

using ghost_v5_interfaces::devices::hardware_type_e;
using ghost_v5_interfaces::devices::JoystickDeviceData;
using namespace ghost_v5_interfaces::test_util;

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
	auto serial_stream_1 = j1->serialize(hardware_type_e::COPROCESSOR);
	j2->deserialize(serial_stream_1, hardware_type_e::V5_BRAIN);

	EXPECT_EQ(*j2, *j3);
}

TEST(TestJoystickDeviceInterface, testSerializationV5ToCoprocessor){
	auto j1 = getRandomJoystickData();
	auto j2 = std::make_shared<JoystickDeviceData>();
	j2->name = j1->name;

	// Test Serialization from Coprocessor -> V5
	auto serial_stream_1 = j1->serialize(hardware_type_e::V5_BRAIN);
	j2->deserialize(serial_stream_1, hardware_type_e::COPROCESSOR);

	EXPECT_EQ(*j1, *j2);
}

TEST(TestJoystickDeviceInterface, testClone){
	auto data_1 = getRandomJoystickData();
	auto data_2 = data_1->clone()->as<JoystickDeviceData>();
	EXPECT_EQ(*data_1, *data_2);
}