#include "ghost_v5_interfaces/devices/motor_device_interface.hpp"

#include <stdlib.h>
#include "gtest/gtest.h"

using ghost_v5_interfaces::MotorDeviceData;

TEST(TestMotorDeviceInterface, testSerialization){
	MotorDeviceData data_1;
	int NUM_TESTS = 50;
	for(int i = 0; i < NUM_TESTS; i++){
		data_1.desired_position = (float) rand();
		data_1.desired_velocity = (float) rand();
		data_1.desired_torque = (float) rand();
		data_1.desired_voltage = (float) rand();
		data_1.current_limit = (float) rand();

		data_1.position_control = (bool) rand();
		data_1.velocity_control = (bool) rand();
		data_1.torque_control = (bool) rand();
		data_1.voltage_control = (bool) rand();

		data_1.curr_position = (float) rand();
		data_1.curr_velocity_rpm = (float) rand();
		data_1.curr_torque_nm = (float) rand();
		data_1.curr_voltage_mv = (float) rand();
		data_1.curr_current_ma = (float) rand();
		data_1.curr_power_w = (float) rand();
		data_1.curr_temp_c = (float) rand();

		MotorDeviceData data_2;
		auto serial_stream_1 = data_1.serialize(true);
		data_2.deserialize(serial_stream_1, true);

		auto serial_stream_2 = data_1.serialize(false);
		data_2.deserialize(serial_stream_2, false);

		EXPECT_EQ(data_1, data_2);
	}
}