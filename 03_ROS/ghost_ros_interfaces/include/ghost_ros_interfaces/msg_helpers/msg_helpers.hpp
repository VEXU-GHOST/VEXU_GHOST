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

using ghost_v5_interfaces::JoystickDeviceData;
using ghost_v5_interfaces::MotorDeviceData;
using ghost_v5_interfaces::RotationSensorDeviceData;

namespace ghost_ros_interfaces {

/**
 * @brief Loads JoystickDeviceData into a V5JoystickState ROS Msg
 *
 * @param joy_data
 * @param joy_msg
 */
void toROSMsg(const JoystickDeviceData& joy_data, ghost_msgs::msg::V5JoystickState& joy_msg){
}

void fromROSMsg(const ghost_msgs::msg::V5JoystickState& joy_msg, JoystickDeviceData& joy_data){
}

void toROSMsg(const MotorDeviceData& motor_data, ghost_msgs::msg::V5MotorState& motor_msg){
}

void fromROSMsg(const ghost_msgs::msg::V5MotorState& motor_msg, MotorDeviceData& motor_data){
}

void toROSMsg(const MotorDeviceData& motor_data, ghost_msgs::msg::V5MotorCommand& motor_msg){
}

void fromROSMsg(const ghost_msgs::msg::V5MotorCommand& motor_msg, MotorDeviceData& motor_data){
}

void toROSMsg(const RotationSensorDeviceData& joy_data, ghost_msgs::msg::V5RotationSensorState& rotation_sensor_msg){
}

void fromROSMsg(const ghost_msgs::msg::V5RotationSensorState& rotation_sensor_msg, RotationSensorDeviceData& joy_data){
}

void toROSMsg(const RobotHardwareInterface& hardware_interface, ghost_msgs::msg::V5ActuatorCommand& actuator_cmd_msg){
}

void fromROSMsg(const ghost_msgs::msg::V5ActuatorCommand& actuator_cmd_msg, RobotHardwareInterface& hardware_interface){
}

void toROSMsg(const RobotHardwareInterface& hardware_interface, ghost_msgs::msg::V5SensorUpdate& sensor_update_msg){
}

void fromROSMsg(const ghost_msgs::msg::V5SensorUpdate& sensor_update_msg, RobotHardwareInterface& hardware_interface){
}

} // namespace ghost_ros_interfaces