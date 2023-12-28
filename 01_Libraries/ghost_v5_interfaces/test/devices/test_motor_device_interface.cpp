#include "ghost_v5_interfaces/devices/motor_device_interface.hpp"
#include "ghost_v5_interfaces/test/device_test_utils.hpp"

#include <stdlib.h>
#include "gtest/gtest.h"

using ghost_v5_interfaces::devices::hardware_type_e;
using ghost_v5_interfaces::devices::MotorDeviceData;
using ghost_v5_interfaces::test_utils::getRandomMotorData;

TEST(TestMotorDeviceInterface, testSerializationV5ToCoprocessor){
	int NUM_TESTS = 50;
	for(int i = 0; i < NUM_TESTS; i++){
		auto data_2 = std::make_shared<MotorDeviceData>();
		auto data_1 = getRandomMotorData(false);
		auto serial_stream_2 = data_1->serialize(hardware_type_e::V5_BRAIN);
		data_2->deserialize(serial_stream_2, hardware_type_e::COPROCESSOR);

		EXPECT_EQ(*data_1, *data_2);
	}
}

TEST(TestMotorDeviceInterface, testSerializationCoprocessorToV5){
	int NUM_TESTS = 50;
	for(int i = 0; i < NUM_TESTS; i++){
		auto data_2 = std::make_shared<MotorDeviceData>();
		auto data_1 = getRandomMotorData(true);
		auto serial_stream_1 = data_1->serialize(hardware_type_e::COPROCESSOR);
		data_2->deserialize(serial_stream_1, hardware_type_e::V5_BRAIN);

		EXPECT_EQ(*data_1, *data_2);
	}
}