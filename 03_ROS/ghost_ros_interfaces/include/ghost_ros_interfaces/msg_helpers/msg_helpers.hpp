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

using ghost_msgs::msg::V5JoystickState;
using ghost_msgs::msg::V5MotorCommand;
using ghost_msgs::msg::V5MotorState;
using ghost_msgs::msg::V5RotationSensorState;

namespace ghost_ros_interfaces {

void toROSMsg(std::shared_ptr<JoystickDeviceData> joy_data, std::shared_ptr<V5JoystickState> joy_msg){
}

void toROSMsg(std::shared_ptr<MotorDeviceData> motor_data, std::shared_ptr<V5MotorState> motor_msg){
}

void toROSMsg(std::shared_ptr<MotorDeviceData> motor_data, std::shared_ptr<V5MotorCommand> motor_msg){
}

void toROSMsg(std::shared_ptr<RotationSensorDeviceData> joy_data, std::shared_ptr<V5RotationSensorState> rotation_sensor_msg){
}

void fromROSMsg(std::shared_ptr<V5JoystickState> joy_msg, std::shared_ptr<JoystickDeviceData> joy_data){
}

void fromROSMsg(std::shared_ptr<V5MotorState> motor_msg, std::shared_ptr<MotorDeviceData> motor_data){
}

void fromROSMsg(std::shared_ptr<V5MotorCommand> motor_msg, std::shared_ptr<MotorDeviceData> motor_data){
}

void fromROSMsg(std::shared_ptr<V5RotationSensorState> rotation_sensor_msg, std::shared_ptr<RotationSensorDeviceData> joy_data){
}

} // namespace ghost_ros_interfaces