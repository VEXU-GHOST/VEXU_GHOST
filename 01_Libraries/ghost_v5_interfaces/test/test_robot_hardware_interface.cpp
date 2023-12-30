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
using namespace ghost_v5_interfaces::devices;
using namespace ghost_v5_interfaces::test_util;
using namespace ghost_v5_interfaces;

class RobotHardwareInterfaceTestFixture : public ::testing::Test {
public:
	void SetUp() override {
		std::string config_path = std::string(getenv("HOME")) + "/VEXU_GHOST/01_Libraries/ghost_v5_interfaces/test/config/example_robot_2.yaml";
		config_yaml_ = YAML::LoadFile(config_path);

		device_config_map_ptr_single_joy_ = loadRobotConfigFromYAML(config_yaml_, false);

		config_yaml_["port_configuration"]["use_partner_joystick"] = true;
		device_config_map_ptr_dual_joy_ = loadRobotConfigFromYAML(config_yaml_, false);
	}

	std::shared_ptr<DeviceConfigMap> device_config_map_ptr_single_joy_;
	std::shared_ptr<DeviceConfigMap> device_config_map_ptr_dual_joy_;
	YAML::Node config_yaml_;
};

TEST_F(RobotHardwareInterfaceTestFixture, testConstructionNoThrow){
	EXPECT_NO_THROW(RobotHardwareInterface hw_interface(device_config_map_ptr_single_joy_, hardware_type_e::COPROCESSOR));
	EXPECT_NO_THROW(RobotHardwareInterface hw_interface(device_config_map_ptr_dual_joy_, hardware_type_e::COPROCESSOR));
}

TEST_F(RobotHardwareInterfaceTestFixture, testEqualityOperator){
	RobotHardwareInterface hw_interface_1(device_config_map_ptr_dual_joy_, hardware_type_e::COPROCESSOR);
	RobotHardwareInterface hw_interface_2(device_config_map_ptr_dual_joy_, hardware_type_e::COPROCESSOR);
	EXPECT_EQ(hw_interface_1, hw_interface_2);
}

TEST_F(RobotHardwareInterfaceTestFixture, testThrowsOnNonExistentDevice){
	RobotHardwareInterface hw_interface(device_config_map_ptr_dual_joy_, hardware_type_e::COPROCESSOR);
	auto motor_data_ptr = std::make_shared<MotorDeviceData>();
	EXPECT_THROW(auto data = hw_interface.getDeviceData<MotorDeviceData>("non_existent_motor"), std::runtime_error);
	EXPECT_THROW(hw_interface.setDeviceData("non_existent_motor", motor_data_ptr), std::runtime_error);
	EXPECT_THROW(auto config = hw_interface.getDeviceConfig<MotorDeviceConfig>("non_existent_motor"), std::runtime_error);
}

TEST_F(RobotHardwareInterfaceTestFixture, testSetAndRetrieveCompetitionStatus){
	RobotHardwareInterface hw_interface(device_config_map_ptr_dual_joy_, hardware_type_e::COPROCESSOR);

	// Default values
	EXPECT_EQ(hw_interface.isDisabled(), true);
	EXPECT_EQ(hw_interface.isAutonomous(), false);
	EXPECT_EQ(hw_interface.isConnected(), false);

	hw_interface.setDisabledStatus(false);
	hw_interface.setAutonomousStatus(true);
	hw_interface.setConnectedStatus(true);

	EXPECT_EQ(hw_interface.isDisabled(), false);
	EXPECT_EQ(hw_interface.isAutonomous(), true);
	EXPECT_EQ(hw_interface.isConnected(), true);
}

TEST_F(RobotHardwareInterfaceTestFixture, testSetAndRetrieveDigitalIO){
	RobotHardwareInterface hw_interface(device_config_map_ptr_dual_joy_, hardware_type_e::COPROCESSOR);

	// Default values
	EXPECT_EQ(hw_interface.getDigitalIO(), std::vector<bool>(8, false));

	auto test_io = std::vector<bool>{
		getRandomBool(), getRandomBool(), getRandomBool(), getRandomBool(),
		getRandomBool(), getRandomBool(), getRandomBool(), getRandomBool()
	};

	hw_interface.setDigitalIO(test_io);

	EXPECT_EQ(hw_interface.getDigitalIO(), test_io);
}

TEST_F(RobotHardwareInterfaceTestFixture, testSetAndRetrieveMotorDeviceData){
	RobotHardwareInterface hw_interface(device_config_map_ptr_dual_joy_, hardware_type_e::COPROCESSOR);
	auto motor_data_ptr = std::make_shared<MotorDeviceData>();

	motor_data_ptr->position_command = (float) rand();
	motor_data_ptr->velocity_command = (float) rand();
	motor_data_ptr->torque_command = (float) rand();
	motor_data_ptr->voltage_command = (float) rand();
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

	auto motor_data_retrieved_ptr = hw_interface.getDeviceData<MotorDeviceData>("left_drive_motor");

	EXPECT_EQ(*motor_data_ptr, *motor_data_retrieved_ptr);
}

TEST_F(RobotHardwareInterfaceTestFixture, testSetAndRetrieveJoystickDeviceDataSingle){
	RobotHardwareInterface hw_interface(device_config_map_ptr_single_joy_, hardware_type_e::COPROCESSOR);
	auto j1 = getRandomJoystickData();
	j1->name = MAIN_JOYSTICK_NAME;
	hw_interface.setDeviceData(j1);
	auto j2 = hw_interface.getDeviceData<JoystickDeviceData>(j1->name);

	EXPECT_EQ(*j1, *j2);
}

TEST_F(RobotHardwareInterfaceTestFixture, testSetAndRetrieveJoystickDeviceDataDual){
	RobotHardwareInterface hw_interface(device_config_map_ptr_dual_joy_, hardware_type_e::COPROCESSOR);
	auto j1 = getRandomJoystickData();
	j1->name = PARTNER_JOYSTICK_NAME;
	hw_interface.setDeviceData(j1);
	auto j2 = hw_interface.getDeviceData<JoystickDeviceData>(j1->name);

	EXPECT_EQ(*j1, *j2);
}

TEST_F(RobotHardwareInterfaceTestFixture, testIteratorIsOrderedByPort){
	RobotHardwareInterface hw_interface(device_config_map_ptr_dual_joy_, hardware_type_e::COPROCESSOR);

	std::vector<int> ports;
	for(const auto & key : hw_interface){
		auto config_ptr = hw_interface.getDeviceConfig<DeviceConfig>(key);
		ports.push_back(config_ptr->port);
	}

	auto ports_sorted = ports;
	std::sort(ports_sorted.begin(), ports_sorted.end());
	EXPECT_EQ(ports, ports_sorted);
}

TEST_F(RobotHardwareInterfaceTestFixture, testSerializationPipelineCoprocessorToV5){
	RobotHardwareInterface hw_interface(device_config_map_ptr_single_joy_, hardware_type_e::COPROCESSOR);

	// Update all motors in the default robot config
	auto motor_data_1 = getRandomMotorData(true);
	hw_interface.setDeviceData("left_drive_motor", motor_data_1);
	auto motor_data_2 = getRandomMotorData(true);
	hw_interface.setDeviceData("test_motor", motor_data_2);
	auto motor_data_3 = getRandomMotorData(true);
	hw_interface.setDeviceData("default_motor", motor_data_3);

	RobotHardwareInterface hw_interface_copy(device_config_map_ptr_single_joy_, hardware_type_e::V5_BRAIN);
	std::vector<unsigned char> serial_data = hw_interface.serialize();
	hw_interface_copy.deserialize(serial_data);

	EXPECT_TRUE(hw_interface.isDataEqual(hw_interface_copy));
}


TEST_F(RobotHardwareInterfaceTestFixture, testSerializationPipelineV5ToCoprocessor){
	RobotHardwareInterface hw_interface(device_config_map_ptr_single_joy_, hardware_type_e::V5_BRAIN);

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
	joy->name = MAIN_JOYSTICK_NAME;
	hw_interface.setDeviceData(joy);

	RobotHardwareInterface hw_interface_copy(device_config_map_ptr_single_joy_, hardware_type_e::COPROCESSOR);
	std::vector<unsigned char> serial_data = hw_interface.serialize();
	hw_interface_copy.deserialize(serial_data);

	EXPECT_TRUE(hw_interface.isDataEqual(hw_interface_copy));
}

TEST_F(RobotHardwareInterfaceTestFixture, testSerializationPipelineV5ToCoprocessorDualJoystick){
	RobotHardwareInterface hw_interface(device_config_map_ptr_dual_joy_, hardware_type_e::V5_BRAIN);

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
	joy->name = MAIN_JOYSTICK_NAME;
	hw_interface.setDeviceData(joy);

	auto joy_2 = getRandomJoystickData();
	joy_2->name = PARTNER_JOYSTICK_NAME;
	hw_interface.setDeviceData(joy_2);

	RobotHardwareInterface hw_interface_copy(device_config_map_ptr_dual_joy_, hardware_type_e::COPROCESSOR);
	std::vector<unsigned char> serial_data = hw_interface.serialize();
	hw_interface_copy.deserialize(serial_data);

	EXPECT_TRUE(hw_interface.isDataEqual(hw_interface_copy));
}

TEST_F(RobotHardwareInterfaceTestFixture, testMotorStateGetters){
	RobotHardwareInterface hw_interface(device_config_map_ptr_dual_joy_, hardware_type_e::V5_BRAIN);

	// Default
	EXPECT_EQ(hw_interface.getMotorPosition("left_drive_motor"), 0);
	EXPECT_EQ(hw_interface.getMotorVelocityRPM("left_drive_motor"), 0);

	auto motor_data_ptr = hw_interface.getDeviceData<MotorDeviceData>("left_drive_motor");
	motor_data_ptr->curr_position = getRandomFloat();
	motor_data_ptr->curr_velocity_rpm = getRandomFloat();
	hw_interface.setDeviceData(motor_data_ptr);

	EXPECT_EQ(hw_interface.getMotorPosition("left_drive_motor"), motor_data_ptr->curr_position);
	EXPECT_EQ(hw_interface.getMotorVelocityRPM("left_drive_motor"), motor_data_ptr->curr_velocity_rpm);
}

TEST_F(RobotHardwareInterfaceTestFixture, testRotationSensorStateGetters){
	RobotHardwareInterface hw_interface(device_config_map_ptr_dual_joy_, hardware_type_e::V5_BRAIN);

	// Default
	EXPECT_EQ(hw_interface.getRotationSensorAngleDegrees("rotation_sensor_1"), 0);
	EXPECT_EQ(hw_interface.getRotationSensorPositionDegrees("rotation_sensor_1"), 0);
	EXPECT_EQ(hw_interface.getRotationSensorVelocityRPM("rotation_sensor_1"), 0);

	auto sensor_data_ptr = hw_interface.getDeviceData<RotationSensorDeviceData>("rotation_sensor_1");
	sensor_data_ptr->angle = getRandomFloat();
	sensor_data_ptr->position = getRandomFloat();
	sensor_data_ptr->velocity = getRandomFloat();
	hw_interface.setDeviceData(sensor_data_ptr);

	EXPECT_EQ(hw_interface.getRotationSensorAngleDegrees("rotation_sensor_1"), sensor_data_ptr->angle);
	EXPECT_EQ(hw_interface.getRotationSensorPositionDegrees("rotation_sensor_1"), sensor_data_ptr->position);
	EXPECT_EQ(hw_interface.getRotationSensorVelocityRPM("rotation_sensor_1"), sensor_data_ptr->velocity);
}

TEST_F(RobotHardwareInterfaceTestFixture, testMotorStateSetters){
	RobotHardwareInterface hw_interface(device_config_map_ptr_dual_joy_, hardware_type_e::V5_BRAIN);

	// Default
	auto motor_data_ptr = hw_interface.getDeviceData<MotorDeviceData>("left_drive_motor");
	EXPECT_EQ(motor_data_ptr->position_command, 0);
	EXPECT_EQ(motor_data_ptr->velocity_command, 0);
	EXPECT_EQ(motor_data_ptr->voltage_command, 0);
	EXPECT_EQ(motor_data_ptr->torque_command, 0);

	auto pos_cmd = getRandomFloat();
	auto vel_cmd = getRandomFloat();
	auto vlt_cmd = getRandomFloat();
	auto trq_cmd = getRandomFloat();

	hw_interface.setMotorPositionCommand("left_drive_motor", pos_cmd);
	hw_interface.setMotorVelocityCommandRPM("left_drive_motor", vel_cmd);
	hw_interface.setMotorVoltageCommandPercent("left_drive_motor", vlt_cmd);
	hw_interface.setMotorTorqueCommandPercent("left_drive_motor", trq_cmd);

	// Original ptr did not change, it was copied
	EXPECT_EQ(motor_data_ptr->position_command, 0);
	EXPECT_EQ(motor_data_ptr->velocity_command, 0);
	EXPECT_EQ(motor_data_ptr->voltage_command, 0);
	EXPECT_EQ(motor_data_ptr->torque_command, 0);

	// New data has been updated
	auto motor_data_ptr_updated = hw_interface.getDeviceData<MotorDeviceData>("left_drive_motor");

	EXPECT_EQ(motor_data_ptr_updated->position_command, pos_cmd);
	EXPECT_EQ(motor_data_ptr_updated->velocity_command, vel_cmd);
	EXPECT_EQ(motor_data_ptr_updated->voltage_command, vlt_cmd);
	EXPECT_EQ(motor_data_ptr_updated->torque_command, trq_cmd);
}