#include "ghost_v5_core/devices/rotation_sensor/rotation_sensor_device_interface.hpp"

#include <stdlib.h>
#include "gtest/gtest.h"

using ghost_v5_core::RotationSensorDeviceData;

TEST(TestRotationSensorDeviceInterface, testSerialization){
	RotationSensorDeviceData data_1;
	int NUM_TESTS = 50;
	for(int i = 0; i < NUM_TESTS; i++){
		data_1.curr_position = (float) rand();
		data_1.curr_velocity_rpm = (float) rand();

		RotationSensorDeviceData data_2;
		auto serial_stream_1 = data_1.serialize(true);
		data_2.deserialize(serial_stream_1, true);

		auto serial_stream_2 = data_1.serialize(false);
		data_2.deserialize(serial_stream_2, false);

		EXPECT_EQ(data_1, data_2);
	}
}