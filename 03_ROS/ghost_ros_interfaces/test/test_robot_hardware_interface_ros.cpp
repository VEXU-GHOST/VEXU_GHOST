#include <gtest/gtest.h>
#include "ghost_v5_interfaces/devices/device_config_map.hpp"
#include "ghost_v5_interfaces/devices/joystick_device_interface.hpp"
#include "ghost_v5_interfaces/devices/motor_device_interface.hpp"
#include "ghost_v5_interfaces/devices/rotation_sensor_device_interface.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"
#include "ghost_v5_interfaces/test/device_test_utils.hpp"

#include "ghost_ros_interfaces/msg_helpers/msg_helpers.hpp"

#include "yaml-cpp/yaml.h"


using namespace ghost_ros_interfaces;
using namespace ghost_v5_interfaces::test_utils;
using namespace ghost_v5_interfaces;

class RobotHardwareInterfaceROSTestFixture : public ::testing::Test {
public:
	void SetUp() override {
		std::string config_path = std::string(getenv("HOME")) + "/VEXU_GHOST/01_Libraries/ghost_v5_interfaces/test/config/example_robot.yaml";
		config_yaml_ = YAML::LoadFile(config_path);

		device_config_map_ptr_ = loadRobotConfigFromYAML(config_yaml_, false);
	}

	std::shared_ptr<DeviceConfigMap> device_config_map_ptr_;
	YAML::Node config_yaml_;
};

TEST_F(RobotHardwareInterfaceROSTestFixture, testSensorUpdate){
	auto robot_hardware_interface = std::make_shared<RobotHardwareInterface>(device_config_map_ptr_);
	robot_hardware_interface->setDisabledStatus(getRandomBool());
	robot_hardware_interface->setAutonomousStatus(getRandomBool());
	robot_hardware_interface->setConnectedStatus(getRandomBool());
	robot_hardware_interface->setPrimaryJoystickData(getRandomJoystickData(true));

	auto motor_data_ptr = getRandomMotorData(false);
	motor_data_ptr->name = "default_motor";
	robot_hardware_interface->setDeviceData(motor_data_ptr);

	auto rotation_sensor_data_ptr = getRandomRotationSensorData();
	rotation_sensor_data_ptr->name = "rotation_sensor_1";
	robot_hardware_interface->setDeviceData(rotation_sensor_data_ptr);

	// Convert to msg
	auto msg = getROSMsgFromRobotHardwareInterface(robot_hardware_interface);

	// Convert back to hardware interface
	auto robot_hardware_interface_copy = std::make_shared<RobotHardwareInterface>(device_config_map_ptr_);
	updateRobotHardwareInterfaceFromROSMsg(robot_hardware_interface_copy);

	EXPECT_EQ(*robot_hardware_interface, *robot_hardware_interface_copy);
}

TEST_F(RobotHardwareInterfaceROSTestFixture, testActuatorUpdate){
	auto robot_hardware_interface = std::make_shared<RobotHardwareInterface>(device_config_map_ptr_);
	auto motor_data_ptr = getRandomMotorData(true);
	motor_data_ptr->name = "default_motor";
	robot_hardware_interface->setDeviceData(motor_data_ptr);

	auto rotation_sensor_data_ptr = getRandomRotationSensorData();
	rotation_sensor_data_ptr->name = "rotation_sensor_1";
	robot_hardware_interface->setDeviceData(rotation_sensor_data_ptr);

	// Convert to msg
	auto msg = getROSMsgFromRobotHardwareInterface(robot_hardware_interface);

	// Convert back to hardware interface
	auto robot_hardware_interface_copy = std::make_shared<RobotHardwareInterface>(device_config_map_ptr_);
	updateRobotHardwareInterfaceFromROSMsg(robot_hardware_interface_copy);

	EXPECT_EQ(*robot_hardware_interface, *robot_hardware_interface_copy);
}