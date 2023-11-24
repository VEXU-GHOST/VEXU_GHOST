#include "ghost_v5_interfaces/devices/joystick_device_interface.hpp"

#include <stdlib.h>
#include "gtest/gtest.h"

using ghost_v5_interfaces::JoystickDeviceData;

TEST(TestJoystickDeviceInterface, testSerialization){
	JoystickDeviceData data_1;
	int NUM_TESTS = 50;
	for(int i = 0; i < NUM_TESTS; i++){
		data_1.left_x = (float) rand();
		data_1.left_y = (float) rand();
		data_1.right_x = (float) rand();
		data_1.right_y = (float) rand();
		data_1.btn_a = (bool) rand();
		data_1.btn_b = (bool) rand();
		data_1.btn_x = (bool) rand();
		data_1.btn_y = (bool) rand();
		data_1.btn_r1 = (bool) rand();
		data_1.btn_r2 = (bool) rand();
		data_1.btn_l1 = (bool) rand();
		data_1.btn_l2 = (bool) rand();
		data_1.btn_u = (bool) rand();
		data_1.btn_l = (bool) rand();
		data_1.btn_r = (bool) rand();
		data_1.btn_d = (bool) rand();
		data_1.is_master = (bool) rand();

		JoystickDeviceData data_2;
		auto serial_stream_1 = data_1.serialize(true);
		data_2.deserialize(serial_stream_1, true);

		auto serial_stream_2 = data_1.serialize(false);
		data_2.deserialize(serial_stream_2, false);

		EXPECT_EQ(data_1, data_2);
	}
}