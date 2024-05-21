#include <ghost_planners/robot_trajectory.hpp>
#include <ghost_v5_interfaces/devices/inertial_sensor_device_interface.hpp>
#include <ghost_v5_interfaces/devices/joystick_device_interface.hpp>
#include <ghost_v5_interfaces/devices/motor_device_interface.hpp>
#include <ghost_v5_interfaces/devices/rotation_sensor_device_interface.hpp>
#include <ghost_v5_interfaces/robot_hardware_interface.hpp>
#include <ghost_v5_interfaces/util/device_config_factory_utils.hpp>

#include <ghost_msgs/msg/labeled_vector.hpp>
#include <ghost_msgs/msg/labeled_vector_map.hpp>
#include <ghost_msgs/msg/robot_trajectory.hpp>
#include <ghost_msgs/msg/v5_actuator_command.hpp>
#include <ghost_msgs/msg/v5_device_header.hpp>
#include <ghost_msgs/msg/v5_inertial_sensor_state.hpp>
#include <ghost_msgs/msg/v5_joystick_state.hpp>
#include <ghost_msgs/msg/v5_motor_command.hpp>
#include <ghost_msgs/msg/v5_motor_state.hpp>
#include <ghost_msgs/msg/v5_rotation_sensor_state.hpp>
#include <ghost_msgs/msg/v5_sensor_update.hpp>

#include <rclcpp/rclcpp.hpp>

namespace ghost_ros_interfaces {

namespace msg_helpers {

// Device Header

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
 * @param device_data
 * @param header_msg
 */
void fromROSMsg(ghost_v5_interfaces::devices::DeviceData& device_data, const ghost_msgs::msg::V5DeviceHeader& header_msg);

// Joystick

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
 * @param joy_data
 * @param joy_msg
 */
void fromROSMsg(ghost_v5_interfaces::devices::JoystickDeviceData& joy_data, const ghost_msgs::msg::V5JoystickState& joy_msg);

// Motor State

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
 * @param motor_data
 * @param motor_msg
 */
void fromROSMsg(ghost_v5_interfaces::devices::MotorDeviceData& motor_data, const ghost_msgs::msg::V5MotorState& motor_msg);

// Motor Command

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
void fromROSMsg(ghost_v5_interfaces::devices::MotorDeviceData& motor_data, const ghost_msgs::msg::V5MotorCommand& motor_msg);

// Rotation Sensor

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
 * @param rotation_sensor_data
 * @param rotation_sensor_msg
 */
void fromROSMsg(ghost_v5_interfaces::devices::RotationSensorDeviceData& rotation_sensor_data, const ghost_msgs::msg::V5RotationSensorState& rotation_sensor_msg);

// Inertial Sensor

/**
 * @brief Copies inertial sensor state data from a InertialSensorDeviceData object into a V5InertialSensorState msg.
 *
 * @param inertial_sensor_data
 * @param inertial_sensor_msg
 */
void toROSMsg(const ghost_v5_interfaces::devices::InertialSensorDeviceData& inertial_sensor_data, ghost_msgs::msg::V5InertialSensorState& inertial_sensor_msg);

/**
 * @brief Copies inertial sensor state data from a V5InertialSensorState msg into a InertialSensorDeviceData object.
 *
 * @param inertial_sensor_data
 * @param inertial_sensor_msg
 */
void fromROSMsg(ghost_v5_interfaces::devices::InertialSensorDeviceData& inertial_sensor_data, const ghost_msgs::msg::V5InertialSensorState& inertial_sensor_msg);

// Aggregate Actuator Command

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
 * @param hardware_interface
 * @param actuator_cmd_msg
 */
void fromROSMsg(ghost_v5_interfaces::RobotHardwareInterface& hardware_interface, const ghost_msgs::msg::V5ActuatorCommand& actuator_cmd_msg);

// Aggregate Sensor Update

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
 * @param hardware_interface
 * @param sensor_update_msg
 */
void fromROSMsg(ghost_v5_interfaces::RobotHardwareInterface& hardware_interface, const ghost_msgs::msg::V5SensorUpdate& sensor_update_msg);

// /**
//  * @brief Copies trajectory data from a RobotTrajectory msg into all listed devices within a RobotTrajectory object
//  *
//  * Note: ROS2 Time requires a node, and thus we can't extract or set the Header msg time from a utility function.
//  * If you want the msg timing to be correct, you need to do it yourself from your ROS Node.
//  *
//  * @param robot_trajectory
//  * @param robot_trajectory_msg
//  */
void fromROSMsg(ghost_planners::RobotTrajectory& robot_trajectory, const ghost_msgs::msg::RobotTrajectory& robot_trajectory_msg);
void fromROSMsg(ghost_planners::RobotTrajectory::MotorTrajectory& motor_trajectory, const ghost_msgs::msg::MotorTrajectory& motor_trajectory_msg);
void toROSMsg(const ghost_planners::RobotTrajectory& robot_trajectory, ghost_msgs::msg::RobotTrajectory& robot_trajectory_msg);
void toROSMsg(const ghost_planners::RobotTrajectory::MotorTrajectory& motor_trajectory, ghost_msgs::msg::MotorTrajectory& motor_trajectory_msg);


void fromROSMsg(std::unordered_map<std::string, std::vector<double> > &labeled_vector_map, const ghost_msgs::msg::LabeledVectorMap& msg);
void toROSMsg(const std::unordered_map<std::string, std::vector<double> > &labeled_vector_map, ghost_msgs::msg::LabeledVectorMap& msg);

} // namespace msg_helpers

} // namespace ghost_ros_interfaces