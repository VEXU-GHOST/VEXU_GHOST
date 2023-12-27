#include "ghost_v5_interfaces/devices/rotation_sensor_device_interface.hpp"
#include "ghost_v5_interfaces/test/device_test_utils.hpp"


#include <stdlib.h>
#include "gtest/gtest.h"

using ghost_v5_interfaces::hardware_type_e;
using ghost_v5_interfaces::RotationSensorDeviceData;
using ghost_v5_interfaces::test_utils::getRandomRotationSensorData;

TEST(TestRotationSensorDeviceInterface, testSerialization){
	int NUM_TESTS = 50;
	for(int i = 0; i < NUM_TESTS; i++){
		auto data_1 = getRandomRotationSensorData();
		auto data_2 = std::make_shared<RotationSensorDeviceData>();
		auto serial_stream_1 = data_1->serialize(hardware_type_e::COPROCESSOR);
		data_2->deserialize(serial_stream_1, hardware_type_e::V5_BRAIN);

		auto serial_stream_2 = data_1->serialize(hardware_type_e::V5_BRAIN);
		data_2->deserialize(serial_stream_2, hardware_type_e::COPROCESSOR);

		EXPECT_EQ(*data_1, *data_2);
	}
}