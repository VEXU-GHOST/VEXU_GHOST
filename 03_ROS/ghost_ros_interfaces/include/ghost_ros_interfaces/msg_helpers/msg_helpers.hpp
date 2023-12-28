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

#include <rclcpp/rclcpp.hpp>

namespace ghost_ros_interfaces {

namespace msg_helpers {

/**
 * @brief Copies base attributes from the base class DeviceData to a V5DeviceHeader msg (which is contained in all Device Msgs)
 *
 * @param device_data
 * @param header_msg
 */
void toROSMsg(const ghost_v5_interfaces::devices::DeviceData& device_data, ghost_msgs::msg::V5DeviceHeader& header_msg);

/**
 * @brief Copies V5DeviceHeader msg data into DeviceData base attributes
 *
 * @param header_msg
 * @param device_data
 */
void fromROSMsg(const ghost_msgs::msg::V5DeviceHeader& header_msg, ghost_v5_interfaces::devices::DeviceData& device_data);

/**
 * @brief Copies data from a JoystickDeviceData object to a V5JoystickState ROS Msg
 *
 * @param joy_data
 * @param joy_msg
 */
void toROSMsg(const ghost_v5_interfaces::devices::JoystickDeviceData& joy_data, ghost_msgs::msg::V5JoystickState& joy_msg);

/**
 * @brief Copies data from a V5JoystickState ROS Msg to a JoystickDeviceData object
 *
 * @param joy_msg
 * @param joy_data
 */
void fromROSMsg(const ghost_msgs::msg::V5JoystickState& joy_msg, ghost_v5_interfaces::devices::JoystickDeviceData& joy_data);

/**
 * @brief Copies motor state data from a MotorDeviceData object into a V5MotorState msg.
 *
 * @param motor_data
 * @param motor_msg
 */
void toROSMsg(const ghost_v5_interfaces::devices::MotorDeviceData& motor_data, ghost_msgs::msg::V5MotorState& motor_msg);

/**
 * @brief Copies motor state data from a V5MotorState msg into a MotorDeviceData object.
 *
 * @param motor_msg
 * @param motor_data
 */
void fromROSMsg(const ghost_msgs::msg::V5MotorState& motor_msg, ghost_v5_interfaces::devices::MotorDeviceData& motor_data);

/**
 * @brief Copies motor command data from a MotorDeviceData object into a V5MotorCommand msg.
 *
 * @param motor_data
 * @param motor_msg
 */
void toROSMsg(const ghost_v5_interfaces::devices::MotorDeviceData& motor_data, ghost_msgs::msg::V5MotorCommand& motor_msg);

/**
 * @brief Copies motor command data from a V5MotorCommand msg into a MotorDeviceData object.
 *
 * @param motor_data
 * @param motor_msg
 */
void fromROSMsg(const ghost_msgs::msg::V5MotorCommand& motor_msg, ghost_v5_interfaces::devices::MotorDeviceData& motor_data);

/**
 * @brief Copies rotation sensor state data from a RotationSensorDeviceData object into a V5RotationSensorState msg.
 *
 * @param rotation_sensor_data
 * @param rotation_sensor_msg
 */
void toROSMsg(const ghost_v5_interfaces::devices::RotationSensorDeviceData& rotation_sensor_data, ghost_msgs::msg::V5RotationSensorState& rotation_sensor_msg);

/**
 * @brief Copies rotation sensor state data from a V5RotationSensorState msg into a RotationSensorDeviceData object.
 *
 * @param rotation_sensor_msg
 * @param rotation_sensor_data
 */
void fromROSMsg(const ghost_msgs::msg::V5RotationSensorState& rotation_sensor_msg, ghost_v5_interfaces::devices::RotationSensorDeviceData& rotation_sensor_data);

/**
 * @brief Copies actuator command data from all devices in a RobotHardwareInterface object into a V5ActuatorCommand msg.
 *
 * @param hardware_interface
 * @param actuator_cmd_msg
 */
void toROSMsg(const ghost_v5_interfaces::RobotHardwareInterface& hardware_interface, ghost_msgs::msg::V5ActuatorCommand& actuator_cmd_msg);

/**
 * @brief Copies actuator command data from a V5ActuatorCommand msg into all listed devices within a RobotHardwareInterface object
 *
 * @param actuator_cmd_msg
 * @param hardware_interface
 */
void fromROSMsg(const ghost_msgs::msg::V5ActuatorCommand& actuator_cmd_msg, ghost_v5_interfaces::RobotHardwareInterface& hardware_interface);

/**
 * @brief Copies sensor update data from all devices in a RobotHardwareInterface object into a V5SensorUpdate msg.
 *
 * Note: ROS2 Time requires a node, and thus we can't extract or set the Header msg time from a utility function.
 * If you want the msg timing to be correct, you need to do it yourself from your ROS Node.
 *
 * @param hardware_interface
 * @param sensor_update_msg
 */
void toROSMsg(const ghost_v5_interfaces::RobotHardwareInterface& hardware_interface, ghost_msgs::msg::V5SensorUpdate& sensor_update_msg);

/**
 * @brief Copies sensor update data from a V5SensorUpdate msg into all listed devices within a RobotHardwareInterface object
 *
 * Note: ROS2 Time requires a node, and thus we can't extract or set the Header msg time from a utility function.
 * If you want the msg timing to be correct, you need to do it yourself from your ROS Node.
 *
 * @param sensor_update_msg
 * @param hardware_interface
 */
void fromROSMsg(const ghost_msgs::msg::V5SensorUpdate& sensor_update_msg, ghost_v5_interfaces::RobotHardwareInterface& hardware_interface);

} // namespace msg_helpers

} // namespace ghost_ros_interfaces