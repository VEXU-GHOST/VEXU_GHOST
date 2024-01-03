#pragma once

#include <memory>
#include "ghost_v5_interfaces/devices/joystick_device_interface.hpp"
#include "ghost_v5_interfaces/devices/motor_device_interface.hpp"
#include "ghost_v5_interfaces/devices/rotation_sensor_device_interface.hpp"


namespace ghost_v5_interfaces {

namespace test_util {

bool getRandomBool(){
	return (bool) (rand() % 2);
}

float getRandomFloat(){
	return (float) rand() + 0.0001 * rand();
}

int getRandomInt(){
	return (int) getRandomFloat();
}

devices::MotorDeviceData::SerialConfig getRandomMotorSerialConfig(){
	devices::MotorDeviceData::SerialConfig config;
	config.send_position_command = getRandomBool();
	config.send_velocity_command = getRandomBool();
	config.send_voltage_command = getRandomBool();
	config.send_torque_command = getRandomBool();
	config.send_torque_data = getRandomBool();
	config.send_voltage_data = getRandomBool();
	config.send_current_data = getRandomBool();
	config.send_power_data = getRandomBool();
	config.send_temp_data = getRandomBool();
	return config;
}

devices::RotationSensorDeviceData::SerialConfig getRandomRotationSensorSerialConfig(){
	devices::RotationSensorDeviceData::SerialConfig config;
	config.send_angle_data = getRandomBool();
	config.send_position_data = getRandomBool();
	config.send_velocity_data = getRandomBool();
	return config;
}

std::shared_ptr<devices::JoystickDeviceData> getRandomJoystickData(){
	auto joy_ptr = std::make_shared<devices::JoystickDeviceData>("joy_" + std::to_string(rand() % 2));

	joy_ptr->left_x = getRandomFloat();
	joy_ptr->left_y = getRandomFloat();
	joy_ptr->right_x = getRandomFloat();
	joy_ptr->right_y = getRandomFloat();
	joy_ptr->btn_a = getRandomBool();
	joy_ptr->btn_b = getRandomBool();
	joy_ptr->btn_x = getRandomBool();
	joy_ptr->btn_y = getRandomBool();
	joy_ptr->btn_r1 = getRandomBool();
	joy_ptr->btn_r2 = getRandomBool();
	joy_ptr->btn_l1 = getRandomBool();
	joy_ptr->btn_l2 = getRandomBool();
	joy_ptr->btn_u = getRandomBool();
	joy_ptr->btn_l = getRandomBool();
	joy_ptr->btn_r = getRandomBool();
	joy_ptr->btn_d = getRandomBool();

	return joy_ptr;
}

std::shared_ptr<devices::MotorDeviceData> getRandomMotorData(bool actuator_cmd,
                                                             devices::MotorDeviceData::SerialConfig serial_config = devices::MotorDeviceData::SerialConfig()){
	auto motor_ptr = std::make_shared<devices::MotorDeviceData>("test", serial_config);

	// Actuator Values
	if(actuator_cmd){
		if(serial_config.send_position_command){
			motor_ptr->position_command = getRandomFloat();
			motor_ptr->position_control = getRandomBool();
		}
		if(serial_config.send_velocity_command){
			motor_ptr->velocity_command = getRandomFloat();
			motor_ptr->velocity_control = getRandomBool();
		}
		if(serial_config.send_voltage_command){
			motor_ptr->voltage_command = getRandomFloat();
			motor_ptr->voltage_control = getRandomBool();
		}
		if(serial_config.send_torque_command){
			motor_ptr->torque_command = getRandomFloat();
			motor_ptr->torque_control = getRandomBool();
		}
		motor_ptr->current_limit = getRandomFloat();
	}
	else{
		motor_ptr->curr_position = getRandomFloat();
		motor_ptr->curr_velocity_rpm = getRandomFloat();

		if(serial_config.send_torque_data){
			motor_ptr->curr_torque_nm = getRandomFloat();
		}
		if(serial_config.send_voltage_data){
			motor_ptr->curr_voltage_mv = getRandomFloat();
		}
		if(serial_config.send_current_data){
			motor_ptr->curr_current_ma = getRandomFloat();
		}
		if(serial_config.send_power_data){
			motor_ptr->curr_power_w = getRandomFloat();
		}
		if(serial_config.send_temp_data){
			motor_ptr->curr_temp_c = getRandomFloat();
		}
	}


	return motor_ptr;
}

std::shared_ptr<devices::RotationSensorDeviceData> getRandomRotationSensorData(devices::RotationSensorDeviceData::SerialConfig serial_config = devices::RotationSensorDeviceData::SerialConfig()) {
	auto rot_sensor_ptr = std::make_shared<devices::RotationSensorDeviceData>("test", serial_config);
	if(serial_config.send_angle_data){
		rot_sensor_ptr->angle = getRandomFloat();
	}
	if(serial_config.send_position_data){
		rot_sensor_ptr->position = getRandomFloat();
	}
	if(serial_config.send_velocity_data){
		rot_sensor_ptr->velocity = getRandomFloat();
	}

	return rot_sensor_ptr;
}

}

}