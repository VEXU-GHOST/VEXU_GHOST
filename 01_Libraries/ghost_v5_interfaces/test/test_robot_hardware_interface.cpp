#include <gtest/gtest.h>
#include "ghost_v5_interfaces/devices/device_config_map.hpp"
#include "ghost_v5_interfaces/devices/joystick_device_interface.hpp"
#include "ghost_v5_interfaces/devices/motor_device_interface.hpp"
#include "ghost_v5_interfaces/devices/rotation_sensor_device_interface.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"
#include "ghost_v5_interfaces/util/device_config_factory_utils.hpp"
#include "yaml-cpp/yaml.h"

#include <algorithm>

using ghost_v5_interfaces::util::loadRobotConfigFromYAML;
using namespace ghost_v5_interfaces;

class RobotHardwareInterfaceTestFixture : public ::testing::Test {
public:
	void SetUp() override {
		std::string config_path = std::string(getenv("HOME")) + "/VEXU_GHOST/01_Libraries/ghost_v5_interfaces/test/config/example_robot.yaml";
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

TEST_F(RobotHardwareInterfaceTestFixture, testSetAndRetrieveDeviceData){
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

TEST_F(RobotHardwareInterfaceTestFixture, testSerializationPipeline){
	RobotHardwareInterface hw_interface(device_config_map_ptr_, hardware_type_e::COPROCESSOR);
	auto motor_data = std::make_shared<MotorDeviceData>();

	motor_data->desired_position = (float) rand();
	motor_data->desired_velocity = (float) rand();
	motor_data->desired_torque = (float) rand();
	motor_data->desired_voltage = (float) rand();
	motor_data->current_limit = (float) rand();
	motor_data->position_control = (bool) rand();
	motor_data->velocity_control = (bool) rand();
	motor_data->torque_control = (bool) rand();
	motor_data->voltage_control = (bool) rand();
	motor_data->curr_position = (float) rand();
	motor_data->curr_velocity_rpm = (float) rand();
	motor_data->curr_torque_nm = (float) rand();
	motor_data->curr_voltage_mv = (float) rand();
	motor_data->curr_current_ma = (float) rand();
	motor_data->curr_power_w = (float) rand();
	motor_data->curr_temp_c = (float) rand();
	hw_interface.setDeviceData("left_drive_motor", motor_data);

	auto joy_data = std::make_shared<JoystickDeviceData>();

	joy_data->left_x = (float) rand();
	joy_data->left_y = (float) rand();
	joy_data->right_x = (float) rand();
	joy_data->right_y = (float) rand();
	joy_data->btn_a = (bool) rand();
	joy_data->btn_b = (bool) rand();
	joy_data->btn_x = (bool) rand();
	joy_data->btn_y = (bool) rand();
	joy_data->btn_r1 = (bool) rand();
	joy_data->btn_r2 = (bool) rand();
	joy_data->btn_l1 = (bool) rand();
	joy_data->btn_l2 = (bool) rand();
	joy_data->btn_u = (bool) rand();
	joy_data->btn_l = (bool) rand();
	joy_data->btn_r = (bool) rand();
	joy_data->btn_d = (bool) rand();
	joy_data->is_master = (bool) rand();

	hw_interface.setJoystickData(joy_data);

	hw_interface.setCompetitionState(competition_state_e::AUTONOMOUS);

	RobotHardwareInterface hw_interface_copy(device_config_map_ptr_, hardware_type_e::COPROCESSOR);
	std::vector<unsigned char> serial_data = hw_interface.serialize();
	hw_interface_copy.deserialize(serial_data);

	EXPECT_EQ(hw_interface, hw_interface_copy);
}