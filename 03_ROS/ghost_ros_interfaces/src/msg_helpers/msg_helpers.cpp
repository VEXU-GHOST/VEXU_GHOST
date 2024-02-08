#include <ghost_ros_interfaces/msg_helpers/msg_helpers.hpp>

using namespace ghost_msgs::msg;
using namespace ghost_v5_interfaces::devices;
using namespace ghost_v5_interfaces::util;
using namespace ghost_v5_interfaces;

namespace ghost_ros_interfaces {

namespace msg_helpers {

void toROSMsg(const DeviceData& device_data, V5DeviceHeader& header_msg){
	header_msg.name = device_data.name;
	try{
		header_msg.type = DEVICE_TYPE_TO_STRING_MAP.at(device_data.type);
	}
	catch(std::exception &e){
		std::cout << "[toROSMsg] Error: Failed to convert device type to string. Device type: " << device_data.type << std::endl;
	}
}

void fromROSMsg(DeviceData& device_data, const V5DeviceHeader& header_msg){
	device_data.name = header_msg.name;

	try{
		device_data.type = STRING_TO_DEVICE_TYPE_MAP.at(header_msg.type);
	}
	catch(std::exception &e){
		std::cout << "[toROSMsg] Error: Failed to convert string to device type. Device Type: " << header_msg.type << std::endl;
	}
}

/**
 * @brief Copies data from a JoystickDeviceData object to a V5JoystickState ROS Msg
 *
 * @param joy_data
 * @param joy_msg
 */
void toROSMsg(const JoystickDeviceData& joy_data, V5JoystickState& joy_msg){
	toROSMsg(joy_data, joy_msg.device_header); // Set device header
	joy_msg.joystick_left_x = joy_data.left_x;
	joy_msg.joystick_left_y = joy_data.left_y;
	joy_msg.joystick_right_x = joy_data.right_x;
	joy_msg.joystick_right_y = joy_data.right_y;
	joy_msg.joystick_btn_a = joy_data.btn_a;
	joy_msg.joystick_btn_b = joy_data.btn_b;
	joy_msg.joystick_btn_x = joy_data.btn_x;
	joy_msg.joystick_btn_y = joy_data.btn_y;
	joy_msg.joystick_btn_up = joy_data.btn_u;
	joy_msg.joystick_btn_down = joy_data.btn_d;
	joy_msg.joystick_btn_left = joy_data.btn_l;
	joy_msg.joystick_btn_right = joy_data.btn_r;
	joy_msg.joystick_btn_l1 = joy_data.btn_l1;
	joy_msg.joystick_btn_l2 = joy_data.btn_l2;
	joy_msg.joystick_btn_r1 = joy_data.btn_r1;
	joy_msg.joystick_btn_r2 = joy_data.btn_r2;
}

/**
 * @brief Copies data from a V5JoystickState ROS Msg to a JoystickDeviceData object
 *
 * @param joy_msg
 * @param joy_data
 */
void fromROSMsg(JoystickDeviceData& joy_data, const V5JoystickState& joy_msg){
	fromROSMsg(joy_data, joy_msg.device_header); // Set base attributes
	joy_data.left_x = joy_msg.joystick_left_x;
	joy_data.left_y = joy_msg.joystick_left_y;
	joy_data.right_x = joy_msg.joystick_right_x;
	joy_data.right_y = joy_msg.joystick_right_y;
	joy_data.btn_a = joy_msg.joystick_btn_a;
	joy_data.btn_b = joy_msg.joystick_btn_b;
	joy_data.btn_x = joy_msg.joystick_btn_x;
	joy_data.btn_y = joy_msg.joystick_btn_y;
	joy_data.btn_u = joy_msg.joystick_btn_up;
	joy_data.btn_d = joy_msg.joystick_btn_down;
	joy_data.btn_l = joy_msg.joystick_btn_left;
	joy_data.btn_r = joy_msg.joystick_btn_right;
	joy_data.btn_l1 = joy_msg.joystick_btn_l1;
	joy_data.btn_l2 = joy_msg.joystick_btn_l2;
	joy_data.btn_r1 = joy_msg.joystick_btn_r1;
	joy_data.btn_r2 = joy_msg.joystick_btn_r2;
}

void toROSMsg(const MotorDeviceData& motor_data, V5MotorState& motor_msg){
	toROSMsg(motor_data, motor_msg.device_header); // Set device header
	motor_msg.curr_position = motor_data.curr_position;
	motor_msg.curr_velocity = motor_data.curr_velocity_rpm;
	motor_msg.curr_torque_nm = motor_data.curr_torque_nm;
	motor_msg.curr_voltage_mv = motor_data.curr_voltage_mv;
	motor_msg.curr_current_ma = motor_data.curr_current_ma;
	motor_msg.curr_power_w = motor_data.curr_power_w;
	motor_msg.curr_temp_c = motor_data.curr_temp_c;
}

void fromROSMsg(MotorDeviceData& motor_data, const V5MotorState& motor_msg){
	fromROSMsg(motor_data, motor_msg.device_header); // Set base attributes
	motor_data.curr_position = motor_msg.curr_position;
	motor_data.curr_velocity_rpm = motor_msg.curr_velocity;
	motor_data.curr_torque_nm = motor_msg.curr_torque_nm;
	motor_data.curr_voltage_mv = motor_msg.curr_voltage_mv;
	motor_data.curr_current_ma = motor_msg.curr_current_ma;
	motor_data.curr_power_w = motor_msg.curr_power_w;
	motor_data.curr_temp_c = motor_msg.curr_temp_c;
}

void toROSMsg(const MotorDeviceData& motor_data, V5MotorCommand& motor_msg){
	toROSMsg(motor_data, motor_msg.device_header); // Set device header

	motor_msg.position_command = motor_data.position_command;
	motor_msg.velocity_command = motor_data.velocity_command;
	motor_msg.torque_command = motor_data.torque_command;
	motor_msg.voltage_command = motor_data.voltage_command;
	motor_msg.current_limit = motor_data.current_limit;
	motor_msg.position_control = motor_data.position_control;
	motor_msg.velocity_control = motor_data.velocity_control;
	motor_msg.torque_control = motor_data.torque_control;
	motor_msg.voltage_control = motor_data.voltage_control;
}

void fromROSMsg(MotorDeviceData& motor_data, const V5MotorCommand& motor_msg){
	fromROSMsg(motor_data, motor_msg.device_header); // Set base attributes
	motor_data.position_command = motor_msg.position_command;
	motor_data.velocity_command = motor_msg.velocity_command;
	motor_data.torque_command = motor_msg.torque_command;
	motor_data.voltage_command = motor_msg.voltage_command;
	motor_data.current_limit = motor_msg.current_limit;
	motor_data.position_control = motor_msg.position_control;
	motor_data.velocity_control = motor_msg.velocity_control;
	motor_data.torque_control = motor_msg.torque_control;
	motor_data.voltage_control = motor_msg.voltage_control;
}

void toROSMsg(const RotationSensorDeviceData& rotation_sensor_data, V5RotationSensorState& rotation_sensor_msg){
	toROSMsg(rotation_sensor_data, rotation_sensor_msg.device_header); // Set device header
	rotation_sensor_msg.angle = rotation_sensor_data.angle;
	rotation_sensor_msg.position = rotation_sensor_data.position;
	rotation_sensor_msg.velocity = rotation_sensor_data.velocity;
}

void fromROSMsg(RotationSensorDeviceData& rotation_sensor_data, const V5RotationSensorState& rotation_sensor_msg){
	fromROSMsg(rotation_sensor_data, rotation_sensor_msg.device_header); // Set base attributes
	rotation_sensor_data.angle = rotation_sensor_msg.angle;
	rotation_sensor_data.position = rotation_sensor_msg.position;
	rotation_sensor_data.velocity = rotation_sensor_msg.velocity;
}

void toROSMsg(const RobotHardwareInterface& hardware_interface, V5ActuatorCommand& actuator_cmd_msg){
	actuator_cmd_msg.msg_id = hardware_interface.getMsgID();

	// Devices
	for(const std::string & device_name : hardware_interface){
		auto device_data_ptr = hardware_interface.getDeviceData<DeviceData>(device_name);
		if(device_data_ptr->type == device_type_e::MOTOR){
			V5MotorCommand msg{};
			auto motor_data_ptr = device_data_ptr->as<MotorDeviceData>();
			toROSMsg(*motor_data_ptr, msg);
			actuator_cmd_msg.motor_commands.push_back(msg);
		}
		else if(device_data_ptr->type == device_type_e::ROTATION_SENSOR){
			continue;
		}
		else if(device_data_ptr->type == device_type_e::JOYSTICK){
			continue;
		}
		else{
			std::string dev_type_str;
			if(DEVICE_TYPE_TO_STRING_MAP.count(device_data_ptr->type) == 1){
				dev_type_str = DEVICE_TYPE_TO_STRING_MAP.at(device_data_ptr->type);
			}
			else{
				dev_type_str = std::to_string(device_data_ptr->type);
			}
			throw std::runtime_error("[toROSMsg] Error: Attempted to convert unsupported device type to ROS Msg. Device Type: " + dev_type_str);
		}
	}

	// Digital IO
	actuator_cmd_msg.digital_io = hardware_interface.getDigitalIO();
}

void fromROSMsg(RobotHardwareInterface& hardware_interface, const V5ActuatorCommand& actuator_cmd_msg){
	hardware_interface.setMsgID(actuator_cmd_msg.msg_id);

	// Motors
	for(const auto &motor_msg : actuator_cmd_msg.motor_commands){
		auto motor_data_ptr = hardware_interface.getDeviceData<MotorDeviceData>(motor_msg.device_header.name);
		fromROSMsg(*motor_data_ptr, motor_msg);
		hardware_interface.setDeviceData(motor_data_ptr);
	}

	// Digital IO
	hardware_interface.setDigitalIO(actuator_cmd_msg.digital_io);
}

void toROSMsg(const RobotHardwareInterface& hardware_interface, V5SensorUpdate& sensor_update_msg){
	sensor_update_msg.msg_id = hardware_interface.getMsgID();

	// Competition Status
	sensor_update_msg.competition_status.is_disabled = hardware_interface.isDisabled();
	sensor_update_msg.competition_status.is_autonomous = hardware_interface.isAutonomous();
	sensor_update_msg.competition_status.is_connected = hardware_interface.isConnected();

	// Devices
	for(const std::string & device_name : hardware_interface){
		auto device_data_ptr = hardware_interface.getDeviceData<DeviceData>(device_name);
		if(device_data_ptr->type == device_type_e::MOTOR){
			V5MotorState msg{};
			auto motor_data_ptr = device_data_ptr->as<MotorDeviceData>();
			toROSMsg(*motor_data_ptr, msg);
			sensor_update_msg.motors.push_back(msg);
		}
		else if(device_data_ptr->type == device_type_e::ROTATION_SENSOR){
			V5RotationSensorState msg{};
			auto rotation_data_ptr = device_data_ptr->as<RotationSensorDeviceData>();
			toROSMsg(*rotation_data_ptr, msg);
			sensor_update_msg.rotation_sensors.push_back(msg);
		}
		else if(device_data_ptr->type == device_type_e::JOYSTICK){
			V5JoystickState msg{};
			auto joy_data_ptr = device_data_ptr->as<JoystickDeviceData>();
			toROSMsg(*joy_data_ptr, msg);
			sensor_update_msg.joysticks.push_back(msg);
		}
		else{
			std::string dev_type_str;
			if(DEVICE_TYPE_TO_STRING_MAP.count(device_data_ptr->type) == 1){
				dev_type_str = DEVICE_TYPE_TO_STRING_MAP.at(device_data_ptr->type);
			}
			else{
				dev_type_str = std::to_string(device_data_ptr->type);
			}
			throw std::runtime_error("[toROSMsg] Error: Attempted to convert unsupported device type to ROS Msg. Device Type: " + dev_type_str);
		}
	}

	// Digital IO
	sensor_update_msg.digital_io = hardware_interface.getDigitalIO();
}

void fromROSMsg(RobotHardwareInterface& hardware_interface, const V5SensorUpdate& sensor_update_msg){
	hardware_interface.setMsgID(sensor_update_msg.msg_id);

	// Competition Status
	hardware_interface.setDisabledStatus(sensor_update_msg.competition_status.is_disabled);
	hardware_interface.setAutonomousStatus(sensor_update_msg.competition_status.is_autonomous);
	hardware_interface.setConnectedStatus(sensor_update_msg.competition_status.is_connected);

	// Joysticks
	for(const auto& joy_msg : sensor_update_msg.joysticks){
		auto joy_data_ptr = hardware_interface.getDeviceData<JoystickDeviceData>(joy_msg.device_header.name);
		fromROSMsg(*joy_data_ptr, joy_msg);
		hardware_interface.setDeviceData(joy_data_ptr);
	}

	// Motors
	for(const auto &motor_msg : sensor_update_msg.motors){
		auto motor_data_ptr = hardware_interface.getDeviceData<MotorDeviceData>(motor_msg.device_header.name);
		fromROSMsg(*motor_data_ptr, motor_msg);
		hardware_interface.setDeviceData(motor_data_ptr);
	}

	// Rotation Sensors
	for(const auto &rotation_sensor_msg : sensor_update_msg.rotation_sensors){
		auto rotation_sensor_data_ptr = hardware_interface.getDeviceData<RotationSensorDeviceData>(rotation_sensor_msg.device_header.name);
		fromROSMsg(*rotation_sensor_data_ptr, rotation_sensor_msg);
		hardware_interface.setDeviceData(rotation_sensor_data_ptr);
	}

	// Digital IO
	hardware_interface.setDigitalIO(sensor_update_msg.digital_io);
}

void fromROSMsg(ghost_planners::RobotTrajectory& robot_trajectory, const ghost_msgs::msg::RobotTrajectory& robot_trajectory_msg){
	robot_trajectory.motor_names = robot_trajectory_msg.motor_names;
	for(const auto& motor_trajectory_msg : robot_trajectory_msg.trajectories){
		auto motor_trajectory_ptr = std::shared_ptr<ghost_planners::RobotTrajectory::MotorTrajectory>();
		fromROSMsg(*motor_trajectory_ptr, motor_trajectory_msg);
		robot_trajectory.add_trajectory(motor_trajectory_ptr);
	}
}

void fromROSMsg(ghost_planners::RobotTrajectory::MotorTrajectory& motor_trajectory, const ghost_msgs::msg::MotorTrajectory& motor_trajectory_msg){
	motor_trajectory.time_vector = std::vector<double>(std::begin(motor_trajectory_msg.time), std::end(motor_trajectory_msg.time));
	motor_trajectory.velocity_vector = std::vector<double>(std::begin(motor_trajectory_msg.velocity), std::end(motor_trajectory_msg.velocity));
	motor_trajectory.voltage_vector = std::vector<double>(std::begin(motor_trajectory_msg.voltage), std::end(motor_trajectory_msg.voltage));
	motor_trajectory.position_vector= std::vector<double>(std::begin(motor_trajectory_msg.position), std::end(motor_trajectory_msg.position));
	motor_trajectory.torque_vector = std::vector<double>(std::begin(motor_trajectory_msg.torque), std::end(motor_trajectory_msg.torque));
}

} // namespace msg_helpers

} // namespace ghost_ros_interfaces