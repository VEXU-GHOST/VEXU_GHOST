#include "ghost_v5_interfaces/devices/rotation_sensor_device_interface.hpp"

#include <stdlib.h>
#include "gtest/gtest.h"

using ghost_v5_interfaces::hardware_type_e;
using ghost_v5_interfaces::RotationSensorDeviceData;

TEST(TestRotationSensorDeviceInterface, testSerialization){
	RotationSensorDeviceData data_1;
	int NUM_TESTS = 50;
	for(int i = 0; i < NUM_TESTS; i++){
		data_1.curr_position = (float) rand();
		data_1.curr_velocity_rpm = (float) rand();

		RotationSensorDeviceData data_2;
		auto serial_stream_1 = data_1.serialize(hardware_type_e::COPROCESSOR);
		data_2.deserialize(serial_stream_1, hardware_type_e::V5_BRAIN);

		auto serial_stream_2 = data_1.serialize(hardware_type_e::V5_BRAIN);
		data_2.deserialize(serial_stream_2, hardware_type_e::COPROCESSOR);

		EXPECT_EQ(data_1, data_2);
	}
}