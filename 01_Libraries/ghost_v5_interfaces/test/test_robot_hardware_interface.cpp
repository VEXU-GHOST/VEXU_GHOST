#include <gtest/gtest.h>
#include "ghost_util/test_util.hpp"
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
using namespace ghost_util;
using namespace ghost_v5_interfaces::devices;
using namespace ghost_v5_interfaces::test_util;
using namespace ghost_v5_interfaces;

class RobotHardwareInterfaceTestFixture : public ::testing::Test {
public:
	void SetUp() override {
		std::string config_path = std::string(getenv("VEXU_HOME")) + "/01_Libraries/ghost_v5_interfaces/test/config/example_robot_2.yaml";
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
	auto motor_data_ptr = std::make_shared<MotorDeviceData>("non_existent_motor");
	EXPECT_THROW(auto data = hw_interface.getDeviceData<MotorDeviceData>("non_existent_motor"), std::runtime_error);
	EXPECT_THROW(hw_interface.setDeviceData(motor_data_ptr), std::runtime_error);
	EXPECT_THROW(auto config = hw_interface.getDeviceConfig<MotorDeviceConfig>("non_existent_motor"), std::runtime_error);
}

TEST_F(RobotHardwareInterfaceTestFixture, testSerialMsgLengths){
	RobotHardwareInterface hw_interface(device_config_map_ptr_dual_joy_, hardware_type_e::COPROCESSOR);
	EXPECT_TRUE(hw_interface.getSensorUpdateMsgLength() != 0);
	EXPECT_TRUE(hw_interface.getActuatorCommandMsgLength() != 0);
}

TEST_F(RobotHardwareInterfaceTestFixture, testSetAndRetrieveMsgID){
	RobotHardwareInterface hw_interface(device_config_map_ptr_dual_joy_, hardware_type_e::COPROCESSOR);

	// Default values
	EXPECT_EQ(hw_interface.getMsgID(), 0);

	auto rand_int = getRandomInt();
	hw_interface.setMsgID(rand_int);

	EXPECT_EQ(hw_interface.getMsgID(), rand_int);
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

TEST_F(RobotHardwareInterfaceTestFixture, testGetDevicePair){
	RobotHardwareInterface hw_interface(device_config_map_ptr_dual_joy_, hardware_type_e::COPROCESSOR);

	// Get Config
	auto motor_config_ptr = hw_interface.getDeviceConfig<MotorDeviceConfig>("left_drive_motor");

	// Update Data
	auto motor_data_ptr = getRandomMotorData(true);
	motor_data_ptr->name = "left_drive_motor";
	hw_interface.setDeviceData(motor_data_ptr);

	// Make Device Pair
	DevicePair expected_pair{};
	expected_pair.config_ptr = motor_config_ptr;
	expected_pair.data_ptr = motor_data_ptr;

	// Get Pair and Check for Equality
	EXPECT_EQ(expected_pair, hw_interface.getDevicePair("left_drive_motor"));
}

TEST_F(RobotHardwareInterfaceTestFixture, testSetAndRetrieveMotorDeviceData){
	RobotHardwareInterface hw_interface(device_config_map_ptr_dual_joy_, hardware_type_e::COPROCESSOR);
	auto motor_data_ptr = std::make_shared<MotorDeviceData>("left_drive_motor");

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
	hw_interface.setDeviceData(motor_data_ptr);

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

TEST_F(RobotHardwareInterfaceTestFixture, testGetMainJoystickData){
	RobotHardwareInterface hw_interface(device_config_map_ptr_single_joy_, hardware_type_e::COPROCESSOR);
	auto j1 = getRandomJoystickData();
	j1->name = MAIN_JOYSTICK_NAME;
	hw_interface.setDeviceData(j1);
	auto j2 = hw_interface.getMainJoystickData();

	EXPECT_EQ(*j1, *j2);
}

TEST_F(RobotHardwareInterfaceTestFixture, testGetPartnerJoystickData){
	RobotHardwareInterface hw_interface(device_config_map_ptr_dual_joy_, hardware_type_e::COPROCESSOR);
	auto j1 = getRandomJoystickData();
	j1->name = PARTNER_JOYSTICK_NAME;
	hw_interface.setDeviceData(j1);
	auto j2 = hw_interface.getPartnerJoystickData();

	EXPECT_EQ(*j1, *j2);
}

TEST_F(RobotHardwareInterfaceTestFixture, testSingleJoystickThrowsWhenRequestingDualJoystick){
	RobotHardwareInterface hw_interface(device_config_map_ptr_single_joy_, hardware_type_e::COPROCESSOR);

	EXPECT_THROW(hw_interface.getDeviceData<JoystickDeviceData>(PARTNER_JOYSTICK_NAME), std::runtime_error);
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
	motor_data_1->name = "left_drive_motor";
	hw_interface.setDeviceData(motor_data_1);
	auto motor_data_2 = getRandomMotorData(true);
	motor_data_2->name = "test_motor";
	hw_interface.setDeviceData(motor_data_2);
	auto motor_data_3 = getRandomMotorData(true);
	motor_data_3->name = "default_motor";
	hw_interface.setDeviceData(motor_data_3);

	RobotHardwareInterface hw_interface_copy(device_config_map_ptr_single_joy_, hardware_type_e::V5_BRAIN);
	std::vector<unsigned char> serial_data = hw_interface.serialize();
	hw_interface_copy.deserialize(serial_data);

	EXPECT_TRUE(hw_interface.isDataEqual(hw_interface_copy));
}


TEST_F(RobotHardwareInterfaceTestFixture, testSerializationPipelineV5ToCoprocessor){
	RobotHardwareInterface hw_interface(device_config_map_ptr_single_joy_, hardware_type_e::V5_BRAIN);

	// Update Motors
	auto motor_data_1 = getRandomMotorData(false);
	motor_data_1->name = "left_drive_motor";
	hw_interface.setDeviceData(motor_data_1);
	auto motor_data_2 = getRandomMotorData(false);
	motor_data_2->name = "test_motor";
	hw_interface.setDeviceData(motor_data_2);
	auto motor_data_3 = getRandomMotorData(false);
	motor_data_3->name = "default_motor";
	hw_interface.setDeviceData(motor_data_3);

	// Update Rotation Sensors
	auto rotation_sensor_1 = getRandomRotationSensorData();
	rotation_sensor_1->name = "rotation_sensor_1";
	hw_interface.setDeviceData(rotation_sensor_1);
	auto rotation_sensor_2 = getRandomRotationSensorData();
	rotation_sensor_2->name = "rotation_sensor_2";
	hw_interface.setDeviceData(rotation_sensor_2);

	// Update Inertial Sensor
	auto inertial_sensor_1 = getRandomInertialSensorData();
	inertial_sensor_1->name = "inertial_sensor_1";
	hw_interface.setDeviceData(inertial_sensor_1);

	// Update Competition State
	hw_interface.setDisabledStatus(getRandomBool());
	hw_interface.setAutonomousStatus(getRandomBool());
	hw_interface.setConnectedStatus(getRandomBool());

	// Update Joystick
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

	// Update Motors
	auto motor_data_1 = getRandomMotorData(false);
	motor_data_1->name = "left_drive_motor";
	hw_interface.setDeviceData(motor_data_1);
	auto motor_data_2 = getRandomMotorData(false);
	motor_data_2->name = "test_motor";
	hw_interface.setDeviceData(motor_data_2);
	auto motor_data_3 = getRandomMotorData(false);
	motor_data_3->name = "default_motor";
	hw_interface.setDeviceData(motor_data_3);

	// Update Rotation Sensors
	auto rotation_sensor_1 = getRandomRotationSensorData();
	rotation_sensor_1->name = "rotation_sensor_1";
	hw_interface.setDeviceData(rotation_sensor_1);
	auto rotation_sensor_2 = getRandomRotationSensorData();
	rotation_sensor_2->name = "rotation_sensor_2";
	hw_interface.setDeviceData(rotation_sensor_2);

	// Update Inertial Sensor
	auto inertial_sensor_1 = getRandomInertialSensorData();
	inertial_sensor_1->name = "inertial_sensor_1";
	hw_interface.setDeviceData(inertial_sensor_1);

	// Update Competition State
	hw_interface.setDisabledStatus(getRandomBool());
	hw_interface.setAutonomousStatus(getRandomBool());
	hw_interface.setConnectedStatus(getRandomBool());

	// Update Joysticks
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
	RobotHardwareInterface hw_interface(device_config_map_ptr_dual_joy_, hardware_type_e::COPROCESSOR);

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
	RobotHardwareInterface hw_interface(device_config_map_ptr_dual_joy_, hardware_type_e::COPROCESSOR);

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

TEST_F(RobotHardwareInterfaceTestFixture, testInertialSensorStateGetters){
	RobotHardwareInterface hw_interface(device_config_map_ptr_dual_joy_, hardware_type_e::COPROCESSOR);

	// Default
	EXPECT_EQ(hw_interface.getInertialSensorXAccel("inertial_sensor_1"), 0);
	EXPECT_EQ(hw_interface.getInertialSensorYAccel("inertial_sensor_1"), 0);
	EXPECT_EQ(hw_interface.getInertialSensorZAccel("inertial_sensor_1"), 0);
	EXPECT_EQ(hw_interface.getInertialSensorXRate("inertial_sensor_1"), 0);
	EXPECT_EQ(hw_interface.getInertialSensorYRate("inertial_sensor_1"), 0);
	EXPECT_EQ(hw_interface.getInertialSensorZRate("inertial_sensor_1"), 0);
	EXPECT_EQ(hw_interface.getInertialSensorHeading("inertial_sensor_1"), 0);

	auto sensor_data_ptr = hw_interface.getDeviceData<InertialSensorDeviceData>("inertial_sensor_1");
	sensor_data_ptr->x_accel = getRandomFloat();
	sensor_data_ptr->y_accel = getRandomFloat();
	sensor_data_ptr->z_accel = getRandomFloat();
	sensor_data_ptr->x_rate = getRandomFloat();
	sensor_data_ptr->y_rate = getRandomFloat();
	sensor_data_ptr->z_rate = getRandomFloat();
	sensor_data_ptr->heading = getRandomFloat();
	hw_interface.setDeviceData(sensor_data_ptr);

	EXPECT_EQ(hw_interface.getInertialSensorXAccel("inertial_sensor_1"), sensor_data_ptr->x_accel);
	EXPECT_EQ(hw_interface.getInertialSensorYAccel("inertial_sensor_1"), sensor_data_ptr->y_accel);
	EXPECT_EQ(hw_interface.getInertialSensorZAccel("inertial_sensor_1"), sensor_data_ptr->z_accel);
	EXPECT_EQ(hw_interface.getInertialSensorXRate("inertial_sensor_1"), sensor_data_ptr->x_rate);
	EXPECT_EQ(hw_interface.getInertialSensorYRate("inertial_sensor_1"), sensor_data_ptr->y_rate);
	EXPECT_EQ(hw_interface.getInertialSensorZRate("inertial_sensor_1"), sensor_data_ptr->z_rate);
	EXPECT_EQ(hw_interface.getInertialSensorHeading("inertial_sensor_1"), sensor_data_ptr->heading);
}

TEST_F(RobotHardwareInterfaceTestFixture, testSetMotorPositionCommand){
	RobotHardwareInterface hw_interface(device_config_map_ptr_dual_joy_, hardware_type_e::COPROCESSOR);

	// Default
	auto motor_data_ptr = hw_interface.getDeviceData<MotorDeviceData>("left_drive_motor");
	EXPECT_EQ(motor_data_ptr->position_command, 0);
	EXPECT_EQ(motor_data_ptr->position_control, false);

	auto pos_cmd = getRandomFloat();
	hw_interface.setMotorPositionCommand("left_drive_motor", pos_cmd);

	auto motor_data_ptr_updated = hw_interface.getDeviceData<MotorDeviceData>("left_drive_motor");
	EXPECT_EQ(motor_data_ptr_updated->position_command, pos_cmd);
	EXPECT_EQ(motor_data_ptr_updated->position_control, true);
}

TEST_F(RobotHardwareInterfaceTestFixture, testSetMotorVelocityCommandRPM){
	RobotHardwareInterface hw_interface(device_config_map_ptr_dual_joy_, hardware_type_e::COPROCESSOR);

	// Default
	auto motor_data_ptr = hw_interface.getDeviceData<MotorDeviceData>("left_drive_motor");
	EXPECT_EQ(motor_data_ptr->velocity_command, 0);
	EXPECT_EQ(motor_data_ptr->velocity_control, false);

	auto vel_cmd = getRandomFloat();
	hw_interface.setMotorVelocityCommandRPM("left_drive_motor", vel_cmd);

	auto motor_data_ptr_updated = hw_interface.getDeviceData<MotorDeviceData>("left_drive_motor");
	EXPECT_EQ(motor_data_ptr_updated->velocity_command, vel_cmd);
	EXPECT_EQ(motor_data_ptr_updated->velocity_control, true);
}

TEST_F(RobotHardwareInterfaceTestFixture, testSetMotorVoltageCommandPercent){
	RobotHardwareInterface hw_interface(device_config_map_ptr_dual_joy_, hardware_type_e::COPROCESSOR);

	// Default
	auto motor_data_ptr = hw_interface.getDeviceData<MotorDeviceData>("left_drive_motor");
	EXPECT_EQ(motor_data_ptr->voltage_command, 0);
	EXPECT_EQ(motor_data_ptr->voltage_control, false);

	auto vlt_cmd = getRandomFloat();
	hw_interface.setMotorVoltageCommandPercent("left_drive_motor", vlt_cmd);

	auto motor_data_ptr_updated = hw_interface.getDeviceData<MotorDeviceData>("left_drive_motor");
	EXPECT_EQ(motor_data_ptr_updated->voltage_command, vlt_cmd);
	EXPECT_EQ(motor_data_ptr_updated->voltage_control, true);
}

TEST_F(RobotHardwareInterfaceTestFixture, testSetMotorTorqueCommandPercent){
	RobotHardwareInterface hw_interface(device_config_map_ptr_dual_joy_, hardware_type_e::COPROCESSOR);

	// Default
	auto motor_data_ptr = hw_interface.getDeviceData<MotorDeviceData>("left_drive_motor");
	EXPECT_EQ(motor_data_ptr->torque_command, 0);
	EXPECT_EQ(motor_data_ptr->torque_control, false);

	auto trq_cmd = getRandomFloat();
	hw_interface.setMotorTorqueCommandPercent("left_drive_motor", trq_cmd);

	auto motor_data_ptr_updated = hw_interface.getDeviceData<MotorDeviceData>("left_drive_motor");
	EXPECT_EQ(motor_data_ptr_updated->torque_command, trq_cmd);
	EXPECT_EQ(motor_data_ptr_updated->torque_control, true);
}

TEST_F(RobotHardwareInterfaceTestFixture, testMotorStateSetters){
	RobotHardwareInterface hw_interface(device_config_map_ptr_dual_joy_, hardware_type_e::COPROCESSOR);

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

	EXPECT_EQ(motor_data_ptr_updated->position_control, true);
	EXPECT_EQ(motor_data_ptr_updated->velocity_control, true);
	EXPECT_EQ(motor_data_ptr_updated->voltage_control, true);
	EXPECT_EQ(motor_data_ptr_updated->torque_control, true);
}

TEST_F(RobotHardwareInterfaceTestFixture, testSetMotorCommandAndControlMode){
	RobotHardwareInterface hw_interface(device_config_map_ptr_dual_joy_, hardware_type_e::COPROCESSOR);

	// Default
	auto pos_cmd = getRandomFloat();
	auto vel_cmd = getRandomFloat();
	auto vlt_cmd = getRandomFloat();
	auto trq_cmd = getRandomFloat();
	bool pos_ctl = getRandomBool();
	bool vel_ctl = getRandomBool();
	bool vlt_ctl = getRandomBool();
	bool trq_ctl = getRandomBool();

	hw_interface.setMotorCommand("left_drive_motor", pos_cmd, vel_cmd, vlt_cmd, trq_cmd);
	hw_interface.setMotorControlMode("left_drive_motor", pos_ctl, vel_ctl, vlt_ctl, trq_ctl);

	// New data has been updated
	auto motor_data_ptr = hw_interface.getDeviceData<MotorDeviceData>("left_drive_motor");

	EXPECT_EQ(motor_data_ptr->position_command, pos_cmd);
	EXPECT_EQ(motor_data_ptr->velocity_command, vel_cmd);
	EXPECT_EQ(motor_data_ptr->voltage_command, vlt_cmd);
	EXPECT_EQ(motor_data_ptr->torque_command, trq_cmd);

	EXPECT_EQ(motor_data_ptr->position_control, pos_ctl);
	EXPECT_EQ(motor_data_ptr->velocity_control, vel_ctl);
	EXPECT_EQ(motor_data_ptr->voltage_control, vlt_ctl);
	EXPECT_EQ(motor_data_ptr->torque_control, trq_ctl);
}

TEST_F(RobotHardwareInterfaceTestFixture, testSetMotorCurrentLimitMilliAmps){
	RobotHardwareInterface hw_interface(device_config_map_ptr_dual_joy_, hardware_type_e::COPROCESSOR);

	// Default
	EXPECT_EQ(hw_interface.getDeviceData<MotorDeviceData>("left_drive_motor")->current_limit, 0);

	// Random value that doesn't exceed limits
	auto rand_current = fabs(fmod(getRandomFloat(), 2500));
	hw_interface.setMotorCurrentLimitMilliAmps("left_drive_motor", rand_current);
	EXPECT_EQ(hw_interface.getDeviceData<MotorDeviceData>("left_drive_motor")->current_limit, rand_current);

	// Lower limit
	hw_interface.setMotorCurrentLimitMilliAmps("left_drive_motor", -1);
	EXPECT_EQ(hw_interface.getDeviceData<MotorDeviceData>("left_drive_motor")->current_limit, 0.0);

	// Upper limit
	hw_interface.setMotorCurrentLimitMilliAmps("left_drive_motor", 2600);
	EXPECT_EQ(hw_interface.getDeviceData<MotorDeviceData>("left_drive_motor")->current_limit, 2500);
}

TEST_F(RobotHardwareInterfaceTestFixture, testThrowsOnMismatchedDevices){
	RobotHardwareInterface hw_interface(device_config_map_ptr_dual_joy_, hardware_type_e::COPROCESSOR);

	// Call Motor Methods on Rotation Sensor
	EXPECT_THROW(hw_interface.setMotorPositionCommand("rotation_sensor_1", getRandomFloat()), std::runtime_error);
	EXPECT_THROW(hw_interface.setMotorVelocityCommandRPM("rotation_sensor_1", getRandomFloat()), std::runtime_error);
	EXPECT_THROW(hw_interface.setMotorVoltageCommandPercent("rotation_sensor_1", getRandomFloat()), std::runtime_error);
	EXPECT_THROW(hw_interface.setMotorTorqueCommandPercent("rotation_sensor_1", getRandomFloat()), std::runtime_error);
	EXPECT_THROW(hw_interface.setMotorCurrentLimitMilliAmps("rotation_sensor_1", getRandomFloat()), std::runtime_error);
	EXPECT_THROW(hw_interface.getMotorPosition("rotation_sensor_1"), std::runtime_error);
	EXPECT_THROW(hw_interface.setMotorCommand("rotation_sensor_1", 0.0, 0.0, 0.0, 0.0), std::runtime_error);
	EXPECT_THROW(hw_interface.setMotorControlMode("rotation_sensor_1", false, false, false, false), std::runtime_error);

	// Call Rotation Sensor Methods on Motor
	EXPECT_THROW(hw_interface.getRotationSensorAngleDegrees("left_drive_motor"), std::runtime_error);
	EXPECT_THROW(hw_interface.getRotationSensorPositionDegrees("left_drive_motor"), std::runtime_error);
	EXPECT_THROW(hw_interface.getRotationSensorVelocityRPM("left_drive_motor"), std::runtime_error);
}

TEST_F(RobotHardwareInterfaceTestFixture, testThrowsOnBadDeviceCast){
	RobotHardwareInterface hw_interface(device_config_map_ptr_dual_joy_, hardware_type_e::COPROCESSOR);

	EXPECT_THROW(hw_interface.getDeviceConfig<JoystickDeviceConfig>("left_drive_motor"), std::runtime_error);
	EXPECT_THROW(hw_interface.getDeviceConfig<MotorDeviceConfig>("rotation_sensor_1"), std::runtime_error);
	EXPECT_THROW(hw_interface.getDeviceConfig<RotationSensorDeviceConfig>(MAIN_JOYSTICK_NAME), std::runtime_error);

	EXPECT_THROW(hw_interface.getDeviceData<JoystickDeviceData>("left_drive_motor"), std::runtime_error);
	EXPECT_THROW(hw_interface.getDeviceData<MotorDeviceData>("rotation_sensor_1"), std::runtime_error);
	EXPECT_THROW(hw_interface.getDeviceData<RotationSensorDeviceData>(MAIN_JOYSTICK_NAME), std::runtime_error);
}