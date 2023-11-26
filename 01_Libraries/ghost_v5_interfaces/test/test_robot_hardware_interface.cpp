#include <gtest/gtest.h>
#include "ghost_v5_interfaces/devices/device_config_map.hpp"
#include "ghost_v5_interfaces/devices/joystick_device_interface.hpp"
#include "ghost_v5_interfaces/devices/motor_device_interface.hpp"
#include "ghost_v5_interfaces/devices/rotation_sensor_device_interface.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"
#include "ghost_v5_interfaces/test/device_test_utils.hpp"
#include "ghost_v5_interfaces/util/device_config_factory_utils.hpp"
#include "yaml-cpp/yaml.h"

#include <algorithm>

using ghost_v5_interfaces::util::loadRobotConfigFromYAML;
using namespace ghost_v5_interfaces::test_utils;
using namespace ghost_v5_interfaces;

class RobotHardwareInterfaceTestFixture : public ::testing::Test {
public:
	void SetUp() override {
		std::string config_path = std::string(getenv("HOME")) + "/VEXU_GHOST/01_Libraries/ghost_v5_interfaces/test/config/example_robot_2.yaml";
		config_yaml_ = YAML::LoadFile(config_path);

		device_config_map_ptr_ = loadRobotConfigFromYAML(config_yaml_, false);
	}

	std::shared_ptr<DeviceConfigMap> device_config_map_ptr_;
	YAML::Node config_yaml_;
};

TEST_F(RobotHardwareInterfaceTestFixture, testConstructionNoThrow){
	EXPECT_NO_THROW(RobotHardwareInterface hw_interface(device_config_map_ptr_, hardware_type_e::COPROCESSOR));
}

TEST_F(RobotHardwareInterfaceTestFixture, testEqualityOperator){
	RobotHardwareInterface hw_interface_1(device_config_map_ptr_, hardware_type_e::COPROCESSOR);
	RobotHardwareInterface hw_interface_2(device_config_map_ptr_, hardware_type_e::COPROCESSOR);
	EXPECT_EQ(hw_interface_1, hw_interface_2);
}

TEST_F(RobotHardwareInterfaceTestFixture, testThrowsOnNonExistentDevice){
	RobotHardwareInterface hw_interface(device_config_map_ptr_, hardware_type_e::COPROCESSOR);
	auto motor_data_ptr = std::make_shared<MotorDeviceData>();
	EXPECT_THROW(auto data = hw_interface.getDeviceData("non_existent_motor"), std::runtime_error);
	EXPECT_THROW(hw_interface.setDeviceData("non_existent_motor", motor_data_ptr), std::runtime_error);
	EXPECT_THROW(auto config = hw_interface.getDeviceConfig("non_existent_motor"), std::runtime_error);
}

TEST_F(RobotHardwareInterfaceTestFixture, testSetAndRetrieveMotorDeviceData){
	RobotHardwareInterface hw_interface(device_config_map_ptr_, hardware_type_e::COPROCESSOR);
	auto motor_data_ptr = std::make_shared<MotorDeviceData>();

	motor_data_ptr->desired_position = (float) rand();
	motor_data_ptr->desired_velocity = (float) rand();
	motor_data_ptr->desired_torque = (float) rand();
	motor_data_ptr->desired_voltage = (float) rand();
	motor_data_ptr->current_limit = (float) rand();
	motor_data_ptr->position_control = (bool) rand();
	motor_data_ptr->velocity_control = (bool) rand();
	motor_data_ptr->torque_control = (bool) rand();
	motor_data_ptr->voltage_control = (bool) rand();
	motor_data_ptr->curr_position = (float) rand();
	motor_data_ptr->curr_velocity_rpm = (float) rand();
	motor_data_ptr->curr_torque_nm = (float) rand();
	motor_data_ptr->curr_voltage_mv = (float) rand();
	motor_data_ptr->curr_current_ma = (float) rand();
	motor_data_ptr->curr_power_w = (float) rand();
	motor_data_ptr->curr_temp_c = (float) rand();
	hw_interface.setDeviceData("left_drive_motor", motor_data_ptr);

	auto motor_data_retrieved_ptr = hw_interface.getDeviceData("left_drive_motor")->as<MotorDeviceData>();

	EXPECT_EQ(*motor_data_ptr, *motor_data_retrieved_ptr);
}

TEST_F(RobotHardwareInterfaceTestFixture, testSetAndRetrieveJoystickDeviceData){
	RobotHardwareInterface hw_interface(device_config_map_ptr_, hardware_type_e::COPROCESSOR);
	auto j1 = getRandomJoystickData();
	j1->name = "primary_joystick";
	hw_interface.setPrimaryJoystickData(j1);
	auto j2 = hw_interface.getPrimaryJoystickData();

	EXPECT_EQ(*j1, *j2);
}

TEST_F(RobotHardwareInterfaceTestFixture, testIteratorIsOrderedByPort){
	RobotHardwareInterface hw_interface(device_config_map_ptr_, hardware_type_e::COPROCESSOR);

	std::vector<int> ports;
	for(const auto & [key, val] : hw_interface){
		ports.push_back(val.config_ptr->port);
	}

	auto ports_sorted = ports;
	std::sort(ports_sorted.begin(), ports_sorted.end());
	EXPECT_EQ(ports, ports_sorted);
}

TEST_F(RobotHardwareInterfaceTestFixture, testSerializationPipelineCoprocessorToV5){
	device_config_map_ptr_->use_secondary_joystick = false;
	RobotHardwareInterface hw_interface(device_config_map_ptr_, hardware_type_e::COPROCESSOR);

	// Update all motors in the default robot config
	auto motor_data_1 = getRandomMotorData(true);
	hw_interface.setDeviceData("left_drive_motor", motor_data_1);
	auto motor_data_2 = getRandomMotorData(true);
	hw_interface.setDeviceData("test_motor", motor_data_2);
	auto motor_data_3 = getRandomMotorData(true);
	hw_interface.setDeviceData("default_motor", motor_data_3);
	auto rotation_sensor_1 = getRandomRotationSensorData();
	hw_interface.setDeviceData("rotation_sensor_1", rotation_sensor_1);
	auto rotation_sensor_2 = getRandomRotationSensorData();
	hw_interface.setDeviceData("rotation_sensor_2", rotation_sensor_2);

	auto joy = getRandomJoystickData();
	joy->name = "primary_joystick";
	hw_interface.setPrimaryJoystickData(joy);

	RobotHardwareInterface hw_interface_copy(device_config_map_ptr_, hardware_type_e::V5_BRAIN);
	std::vector<unsigned char> serial_data = hw_interface.serialize();
	hw_interface_copy.deserialize(serial_data);

	for(const auto& [key, val] : hw_interface){
		auto copied_data_ptr = hw_interface_copy.getDeviceData(key);
		if(copied_data_ptr->type == device_type_e::MOTOR){
			auto expected_motor_data = val.data_ptr->as<MotorDeviceData>();
			auto received_motor_data = copied_data_ptr->as<MotorDeviceData>();
			EXPECT_EQ(*val.data_ptr, *copied_data_ptr);
		}
	}
}


TEST_F(RobotHardwareInterfaceTestFixture, testSerializationPipelineV5ToCoprocessor){
	device_config_map_ptr_->use_secondary_joystick = false;
	RobotHardwareInterface hw_interface(device_config_map_ptr_, hardware_type_e::V5_BRAIN);

	// Update all motors in the default robot config
	auto motor_data_1 = getRandomMotorData(false);
	hw_interface.setDeviceData("left_drive_motor", motor_data_1);
	auto motor_data_2 = getRandomMotorData(false);
	hw_interface.setDeviceData("test_motor", motor_data_2);
	auto motor_data_3 = getRandomMotorData(false);
	hw_interface.setDeviceData("default_motor", motor_data_3);
	auto rotation_sensor_1 = getRandomRotationSensorData();
	hw_interface.setDeviceData("rotation_sensor_1", rotation_sensor_1);
	auto rotation_sensor_2 = getRandomRotationSensorData();
	hw_interface.setDeviceData("rotation_sensor_2", rotation_sensor_2);

	hw_interface.setDisabledStatus(getRandomBool());
	hw_interface.setAutonomousStatus(getRandomBool());
	hw_interface.setConnectedStatus(getRandomBool());

	auto joy = getRandomJoystickData();
	joy->name = "primary_joystick";
	hw_interface.setPrimaryJoystickData(joy);

	RobotHardwareInterface hw_interface_copy(device_config_map_ptr_, hardware_type_e::COPROCESSOR);
	std::vector<unsigned char> serial_data = hw_interface.serialize();
	hw_interface_copy.deserialize(serial_data);

	EXPECT_EQ(*hw_interface.getPrimaryJoystickData(), *hw_interface_copy.getPrimaryJoystickData());

	EXPECT_EQ(hw_interface.isDisabled(), hw_interface_copy.isDisabled());
	EXPECT_EQ(hw_interface.isAutonomous(), hw_interface_copy.isAutonomous());
	EXPECT_EQ(hw_interface.isConnected(), hw_interface_copy.isConnected());

	for(const auto& [key, val] : hw_interface){
		auto copied_data_ptr = hw_interface_copy.getDeviceData(key);
		if(copied_data_ptr->type == device_type_e::MOTOR){
			auto expected_motor_data = val.data_ptr->as<MotorDeviceData>();
			auto received_motor_data = copied_data_ptr->as<MotorDeviceData>();
			EXPECT_EQ(*val.data_ptr, *copied_data_ptr);
		}
	}
}

TEST_F(RobotHardwareInterfaceTestFixture, testSerializationPipelineV5ToCoprocessorDualJoystick){
	device_config_map_ptr_->use_secondary_joystick = true;
	RobotHardwareInterface hw_interface(device_config_map_ptr_, hardware_type_e::V5_BRAIN);

	// Update all motors in the default robot config
	auto motor_data_1 = getRandomMotorData(false);
	hw_interface.setDeviceData("left_drive_motor", motor_data_1);
	auto motor_data_2 = getRandomMotorData(false);
	hw_interface.setDeviceData("test_motor", motor_data_2);
	auto motor_data_3 = getRandomMotorData(false);
	hw_interface.setDeviceData("default_motor", motor_data_3);
	auto rotation_sensor_1 = getRandomRotationSensorData();
	hw_interface.setDeviceData("rotation_sensor_1", rotation_sensor_1);
	auto rotation_sensor_2 = getRandomRotationSensorData();
	hw_interface.setDeviceData("rotation_sensor_2", rotation_sensor_2);

	hw_interface.setDisabledStatus(getRandomBool());
	hw_interface.setAutonomousStatus(getRandomBool());
	hw_interface.setConnectedStatus(getRandomBool());

	auto joy = getRandomJoystickData();
	joy->name = "primary_joystick";
	hw_interface.setPrimaryJoystickData(joy);

	auto joy_2 = getRandomJoystickData();
	joy_2->name = "secondary_joystick";
	hw_interface.setSecondaryJoystickData(joy);

	RobotHardwareInterface hw_interface_copy(device_config_map_ptr_, hardware_type_e::COPROCESSOR);
	std::vector<unsigned char> serial_data = hw_interface.serialize();
	hw_interface_copy.deserialize(serial_data);

	EXPECT_EQ(*hw_interface.getPrimaryJoystickData(), *hw_interface_copy.getPrimaryJoystickData());
	EXPECT_EQ(*hw_interface.getSecondaryJoystickData(), *hw_interface_copy.getSecondaryJoystickData());

	EXPECT_EQ(hw_interface.isDisabled(), hw_interface_copy.isDisabled());
	EXPECT_EQ(hw_interface.isAutonomous(), hw_interface_copy.isAutonomous());
	EXPECT_EQ(hw_interface.isConnected(), hw_interface_copy.isConnected());

	for(const auto& [key, val] : hw_interface){
		auto copied_data_ptr = hw_interface_copy.getDeviceData(key);
		if(copied_data_ptr->type == device_type_e::MOTOR){
			auto expected_motor_data = val.data_ptr->as<MotorDeviceData>();
			auto received_motor_data = copied_data_ptr->as<MotorDeviceData>();
			EXPECT_EQ(*val.data_ptr, *copied_data_ptr);
		}
	}
}