#include <ghost_v5_interfaces/devices/joystick_device_interface.hpp>
#include <ghost_v5_interfaces/devices/motor_device_interface.hpp>
#include <ghost_v5_interfaces/devices/rotation_sensor_device_interface.hpp>
#include <ghost_v5_interfaces/robot_hardware_interface.hpp>

#include <ghost_msgs/msg/v5_actuator_command.hpp>
#include <ghost_msgs/msg/v5_device_header.hpp>
#include <ghost_msgs/msg/v5_joystick_state.hpp>
#include <ghost_msgs/msg/v5_motor_command.hpp>
#include <ghost_msgs/msg/v5_motor_state.hpp>
#include <ghost_msgs/msg/v5_rotation_sensor_state.hpp>
#include <ghost_msgs/msg/v5_sensor_update.hpp>

namespace ghost_ros_interfaces {

/**
 * @brief Loads JoystickDeviceData into a V5JoystickState ROS Msg
 *
 * @param joy_data
 * @param joy_msg
 */
void toROSMsg(const ghost_v5_interfaces::devices::JoystickDeviceData& joy_data, ghost_msgs::msg::V5JoystickState& joy_msg){
	// Joystick State
	float left_x = 0.0;
	float left_y = 0.0;
	float right_x = 0.0;
	float right_y = 0.0;
	bool btn_a = false;
	bool btn_b = false;
	bool btn_x = false;
	bool btn_y = false;
	bool btn_r1 = false;
	bool btn_r2 = false;
	bool btn_l1 = false;
	bool btn_l2 = false;
	bool btn_u = false;
	bool btn_l = false;
	bool btn_r = false;
	bool btn_d = false;
	bool is_secondary_joystick = true;

    # Joystick Channels
	joystick_left_x
	joystick_left_y
	joystick_right_x
	        joystick_right_y

# Joystick Buttons
	joystick_btn_a
	joystick_btn_b
	joystick_btn_x
	joystick_btn_y
	joystick_btn_up
	joystick_btn_down
	joystick_btn_left
	joystick_btn_right
	joystick_btn_l1
	joystick_btn_l2
	joystick_btn_r1
	        joystick_btn_r2

	bool is_secondary_joystick
}

void fromROSMsg(const ghost_msgs::msg::V5JoystickState& joy_msg, ghost_v5_interfaces::devices::JoystickDeviceData& joy_data){
}

void toROSMsg(const ghost_v5_interfaces::devices::MotorDeviceData& motor_data, ghost_msgs::msg::V5MotorState& motor_msg){
}

void fromROSMsg(const ghost_msgs::msg::V5MotorState& motor_msg, ghost_v5_interfaces::devices::MotorDeviceData& motor_data){
}

void toROSMsg(const ghost_v5_interfaces::devices::MotorDeviceData& motor_data, ghost_msgs::msg::V5MotorCommand& motor_msg){
}

void fromROSMsg(const ghost_msgs::msg::V5MotorCommand& motor_msg, ghost_v5_interfaces::devices::MotorDeviceData& motor_data){
}

void toROSMsg(const ghost_v5_interfaces::devices::RotationSensorDeviceData& joy_data, ghost_msgs::msg::V5RotationSensorState& rotation_sensor_msg){
}

void fromROSMsg(const ghost_msgs::msg::V5RotationSensorState& rotation_sensor_msg, ghost_v5_interfaces::devices::RotationSensorDeviceData& joy_data){
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