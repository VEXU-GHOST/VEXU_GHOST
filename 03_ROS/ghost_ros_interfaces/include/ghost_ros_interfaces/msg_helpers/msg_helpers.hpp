#include <ghost_v5_interfaces/devices/joystick_device_interface.hpp>
#include <ghost_v5_interfaces/devices/motor_device_interface.hpp>
#include <ghost_v5_interfaces/devices/rotation_sensor_device_interface.hpp>
#include <ghost_v5_interfaces/robot_hardware_interface.hpp>
#include <ghost_v5_interfaces/util/device_config_factory_utils.hpp>

#include <ghost_msgs/msg/v5_actuator_command.hpp>
#include <ghost_msgs/msg/v5_device_header.hpp>
#include <ghost_msgs/msg/v5_joystick_state.hpp>
#include <ghost_msgs/msg/v5_motor_command.hpp>
#include <ghost_msgs/msg/v5_motor_state.hpp>
#include <ghost_msgs/msg/v5_rotation_sensor_state.hpp>
#include <ghost_msgs/msg/v5_sensor_update.hpp>

namespace ghost_ros_interfaces {

void toROSMsg(const ghost_v5_interfaces::devices::DeviceData& device_data, ghost_msgs::msg::V5DeviceHeader& header_msg){
	header_msg.name = device_data.name;
	header_msg.msg_id = device_data.msg_id;
	try{
		header_msg.type = ghost_v5_interfaces::util::DEVICE_TYPE_TO_STRING_MAP.at(device_data.type);
	}
	catch(std::exception &e){
		std::cout << "[toROSMsg] Error: Failed to convert device type to string. Device type: " << device_data.type << std::endl;
	}
}

void fromROSMsg(const ghost_msgs::msg::V5DeviceHeader& header_msg, ghost_v5_interfaces::devices::DeviceData& device_data){
	device_data.name = header_msg.name;
	device_data.msg_id = header_msg.msg_id;

	try{
		device_data.type = ghost_v5_interfaces::util::STRING_TO_DEVICE_TYPE_MAP.at(header_msg.type);
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
void toROSMsg(const ghost_v5_interfaces::devices::JoystickDeviceData& joy_data, ghost_msgs::msg::V5JoystickState& joy_msg){
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
	joy_msg.is_secondary_joystick = joy_data.is_secondary_joystick;
}

/**
 * @brief Copies data from a V5JoystickState ROS Msg to a JoystickDeviceData object
 *
 * @param joy_msg
 * @param joy_data
 */
void fromROSMsg(const ghost_msgs::msg::V5JoystickState& joy_msg, ghost_v5_interfaces::devices::JoystickDeviceData& joy_data){
	fromROSMsg(joy_msg.device_header, joy_data); // Set base attributes
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
	joy_data.is_secondary_joystick = joy_msg.is_secondary_joystick;
}

void toROSMsg(const ghost_v5_interfaces::devices::MotorDeviceData& motor_data, ghost_msgs::msg::V5MotorState& motor_msg){
	toROSMsg(motor_data, motor_msg.device_header); // Set device header
	motor_msg.curr_position = motor_data.curr_position;
	motor_msg.curr_velocity = motor_data.curr_velocity_rpm;
	motor_msg.curr_torque_nm = motor_data.curr_torque_nm;
	motor_msg.curr_voltage_mv = motor_data.curr_voltage_mv;
	motor_msg.curr_current_ma = motor_data.curr_current_ma;
	motor_msg.curr_power_w = motor_data.curr_power_w;
	motor_msg.curr_temp_c = motor_data.curr_temp_c;
}

void fromROSMsg(const ghost_msgs::msg::V5MotorState& motor_msg, ghost_v5_interfaces::devices::MotorDeviceData& motor_data){
	fromROSMsg(motor_msg.device_header, motor_data); // Set base attributes
	motor_data.curr_position = motor_msg.curr_position;
	motor_data.curr_velocity_rpm = motor_msg.curr_velocity;
	motor_data.curr_torque_nm = motor_msg.curr_torque_nm;
	motor_data.curr_voltage_mv = motor_msg.curr_voltage_mv;
	motor_data.curr_current_ma = motor_msg.curr_current_ma;
	motor_data.curr_power_w = motor_msg.curr_power_w;
	motor_data.curr_temp_c = motor_msg.curr_temp_c;
}

void toROSMsg(const ghost_v5_interfaces::devices::MotorDeviceData& motor_data, ghost_msgs::msg::V5MotorCommand& motor_msg){
	toROSMsg(motor_data, motor_msg.device_header); // Set device header

	motor_msg.desired_position = motor_data.desired_position;
	motor_msg.desired_velocity = motor_data.desired_velocity;
	motor_msg.desired_torque = motor_data.desired_torque;
	motor_msg.desired_voltage = motor_data.desired_voltage;
	motor_msg.current_limit = motor_data.current_limit;
	motor_msg.position_control = motor_data.position_control;
	motor_msg.velocity_control = motor_data.velocity_control;
	motor_msg.torque_control = motor_data.torque_control;
	motor_msg.voltage_control = motor_data.voltage_control;
}

void fromROSMsg(const ghost_msgs::msg::V5MotorCommand& motor_msg, ghost_v5_interfaces::devices::MotorDeviceData& motor_data){
	fromROSMsg(motor_msg.device_header, motor_data); // Set base attributes
	motor_data.desired_position = motor_msg.desired_position;
	motor_data.desired_velocity = motor_msg.desired_velocity;
	motor_data.desired_torque = motor_msg.desired_torque;
	motor_data.desired_voltage = motor_msg.desired_voltage;
	motor_data.current_limit = motor_msg.current_limit;
	motor_data.position_control = motor_msg.position_control;
	motor_data.velocity_control = motor_msg.velocity_control;
	motor_data.torque_control = motor_msg.torque_control;
	motor_data.voltage_control = motor_msg.voltage_control;
}

void toROSMsg(const ghost_v5_interfaces::devices::RotationSensorDeviceData& rotation_sensor_data, ghost_msgs::msg::V5RotationSensorState& rotation_sensor_msg){
	toROSMsg(rotation_sensor_data, rotation_sensor_msg.device_header); // Set device header
	rotation_sensor_msg.angle = rotation_sensor_data.angle;
	rotation_sensor_msg.position = rotation_sensor_data.position;
	rotation_sensor_msg.velocity = rotation_sensor_data.velocity;
}

void fromROSMsg(const ghost_msgs::msg::V5RotationSensorState& rotation_sensor_msg, ghost_v5_interfaces::devices::RotationSensorDeviceData& rotation_sensor_data){
	fromROSMsg(rotation_sensor_msg.device_header, rotation_sensor_data); // Set base attributes
	rotation_sensor_data.angle = rotation_sensor_msg.angle;
	rotation_sensor_data.position = rotation_sensor_msg.position;
	rotation_sensor_data.velocity = rotation_sensor_msg.velocity;
}

void toROSMsg(const ghost_v5_interfaces::RobotHardwareInterface& hardware_interface, ghost_msgs::msg::V5ActuatorCommand& actuator_cmd_msg){
}

void fromROSMsg(const ghost_msgs::msg::V5ActuatorCommand& actuator_cmd_msg, ghost_v5_interfaces::RobotHardwareInterface& hardware_interface){
}

void toROSMsg(const ghost_v5_interfaces::RobotHardwareInterface& hardware_interface, ghost_msgs::msg::V5SensorUpdate& sensor_update_msg){
}

void fromROSMsg(const ghost_msgs::msg::V5SensorUpdate& sensor_update_msg, ghost_v5_interfaces::RobotHardwareInterface& hardware_interface){
}

} // namespace ghost_ros_interfaces