#include <filesystem>
#include <dlfcn.h>

#include <ghost_v5_interfaces/devices/device_config_map.hpp>
#include <ghost_v5_interfaces/devices/motor_device_interface.hpp>
#include <ghost_v5_interfaces/devices/rotation_sensor_device_interface.hpp>
#include <ghost_v5_interfaces/test/device_test_utils.hpp>
#include <ghost_v5_interfaces/util/device_config_factory_utils.hpp>

#include <gtest/gtest.h>
#include "yaml-cpp/yaml.h"

using namespace ghost_v5_interfaces::devices;
using namespace ghost_v5_interfaces::test_util;
using namespace ghost_v5_interfaces::util;
using namespace ghost_v5_interfaces;

class DeviceConfigMapTestFixture : public ::testing::Test {
protected:
	void SetUp() override {
	}

	std::shared_ptr<DeviceConfigMap> getExpectedRobotConfig(bool use_partner_joystick = false){
		auto robot_config_ptr = std::make_shared<DeviceConfigMap>();
		auto joy_master = std::make_shared<JoystickDeviceConfig>();
		joy_master->name = MAIN_JOYSTICK_NAME;
		joy_master->port = -1;
		joy_master->type = device_type_e::JOYSTICK;
		robot_config_ptr->addDeviceConfig(joy_master);

		if(use_partner_joystick){
			auto joy_partner = std::make_shared<JoystickDeviceConfig>();
			joy_partner->name = PARTNER_JOYSTICK_NAME;
			joy_partner->port = -2;
			joy_partner->type = device_type_e::JOYSTICK;
			joy_partner->is_partner = true;
			robot_config_ptr->addDeviceConfig(joy_partner);
		}

		// Motor some parameters changed
		auto left_drive_motor = std::make_shared<MotorDeviceConfig>();
		left_drive_motor->name = "left_drive_motor";
		left_drive_motor->port = 1;
		left_drive_motor->type = device_type_e::MOTOR;
		left_drive_motor->encoder_units = ghost_encoder_unit::ENCODER_DEGREES;
		left_drive_motor->gearset = ghost_gearset::GEARSET_600;
		left_drive_motor->brake_mode = ghost_brake_mode::BRAKE_MODE_COAST;
		left_drive_motor->controller_config.pos_gain = 7.5;
		robot_config_ptr->addDeviceConfig(left_drive_motor);

		// Motor with every parameter changed.
		auto test_motor = std::make_shared<MotorDeviceConfig>();
		test_motor->name = "test_motor";
		test_motor->port = 2;
		test_motor->type = device_type_e::MOTOR;
		test_motor->reversed = true;
		test_motor->encoder_units = ghost_encoder_unit::ENCODER_ROTATIONS;
		test_motor->gearset = ghost_gearset::GEARSET_100;
		test_motor->brake_mode = ghost_brake_mode::BRAKE_MODE_BRAKE;
		test_motor->filter_config.cutoff_frequency = 62.0;
		test_motor->filter_config.damping_ratio = 0.10;
		test_motor->filter_config.timestep = 0.21;
		test_motor->model_config.free_speed = 1104.0;
		test_motor->model_config.stall_torque = 52.25;
		test_motor->model_config.free_current = 0.9090;
		test_motor->model_config.stall_current = 21.58;
		test_motor->model_config.nominal_voltage = 1200.0;
		test_motor->model_config.gear_ratio = 2915.0;
		test_motor->controller_config.pos_gain = 8000.0;
		test_motor->controller_config.vel_gain = 118.0;
		test_motor->controller_config.ff_vel_gain = 400.0;
		test_motor->controller_config.ff_torque_gain = 2587.0;
		test_motor->serial_config.send_voltage_command = false;
		test_motor->serial_config.send_torque_data = true;
		test_motor->serial_config.send_power_data = true;
		robot_config_ptr->addDeviceConfig(test_motor);

		// Default motor, minimally required info.
		auto default_motor = std::make_shared<MotorDeviceConfig>();
		default_motor->name = "default_motor";
		default_motor->port = 3;
		default_motor->type = device_type_e::MOTOR;
		robot_config_ptr->addDeviceConfig(default_motor);

		// Changed every param
		auto rotation_sensor_1 = std::make_shared<RotationSensorDeviceConfig>();
		rotation_sensor_1->port = 4;
		rotation_sensor_1->name = "rotation_sensor_1";
		rotation_sensor_1->type = device_type_e::ROTATION_SENSOR;
		rotation_sensor_1->reversed = true;
		rotation_sensor_1->data_rate = 10;
		rotation_sensor_1->serial_config.send_position_data = true;
		rotation_sensor_1->serial_config.send_angle_data = false;
		rotation_sensor_1->serial_config.send_velocity_data = true;
		robot_config_ptr->addDeviceConfig(rotation_sensor_1);

		// Default (minimal required params)
		auto rotation_sensor_2 = std::make_shared<RotationSensorDeviceConfig>();
		rotation_sensor_2->port = 5;
		rotation_sensor_2->name = "rotation_sensor_2";
		rotation_sensor_2->type = device_type_e::ROTATION_SENSOR;
		robot_config_ptr->addDeviceConfig(rotation_sensor_2);

		return robot_config_ptr;
	}
};

/**
 * @brief Test that we can load a DeviceConfigMap from YAML.
 */
TEST_F(DeviceConfigMapTestFixture, testLoadRobotConfigFromYAMLFile){
	auto example_robot_config_file = std::string(getenv("HOME")) + "/VEXU_GHOST/01_Libraries/ghost_v5_interfaces/test/config/example_robot.yaml";
	auto device_config_ptr = loadRobotConfigFromYAMLFile(example_robot_config_file);
	EXPECT_EQ(*device_config_ptr, *getExpectedRobotConfig(false));
}

/**
 * @brief Test that we can load a DeviceConfigMap from YAML with secondary joystick set to true
 */
TEST_F(DeviceConfigMapTestFixture, testLoadRobotConfigFromYAMLSecondaryJoystick){
	auto example_robot_config = YAML::LoadFile(std::string(getenv("HOME")) + "/VEXU_GHOST/01_Libraries/ghost_v5_interfaces/test/config/example_robot.yaml");
	example_robot_config["port_configuration"]["use_partner_joystick"] = true;
	auto device_config_ptr = loadRobotConfigFromYAML(example_robot_config);
	EXPECT_EQ(*device_config_ptr, *getExpectedRobotConfig(true));
}

/**
 * @brief Test that we can load a DeviceConfigMap from YAML and it mismatches if we change joystick settings.
 */
TEST_F(DeviceConfigMapTestFixture, testLoadRobotConfigFromYAML){
	auto example_robot_config = YAML::LoadFile(std::string(getenv("HOME")) + "/VEXU_GHOST/01_Libraries/ghost_v5_interfaces/test/config/example_robot.yaml");
	example_robot_config["port_configuration"]["use_partner_joystick"] = true;
	auto device_config_ptr = loadRobotConfigFromYAML(example_robot_config);
	EXPECT_FALSE(*device_config_ptr == *getExpectedRobotConfig(false));
}

/**
 * @brief Test that mismatched motor port will fail.
 */
TEST_F(DeviceConfigMapTestFixture, testLoadRobotConfigFromYAMLMismatchPortConfig){
	auto example_robot_config = YAML::LoadFile(std::string(getenv("HOME")) + "/VEXU_GHOST/01_Libraries/ghost_v5_interfaces/test/config/example_robot.yaml");
	example_robot_config["port_configuration"]["devices"]["left_drive_motor"]["port"] = 10;
	auto device_config_ptr = loadRobotConfigFromYAML(example_robot_config);
	EXPECT_FALSE(*device_config_ptr == *getExpectedRobotConfig(false));
}

/**
 * @brief Test that mismatched motor config will fail.
 */
TEST_F(DeviceConfigMapTestFixture, testLoadRobotConfigFromYAMLMismatchMotorConfig){
	auto example_robot_config = YAML::LoadFile(std::string(getenv("HOME")) + "/VEXU_GHOST/01_Libraries/ghost_v5_interfaces/test/config/example_robot.yaml");
	example_robot_config["port_configuration"]["device_configurations"]["test_motor_config"]["filter"]["timestep"] = 0.1;
	auto device_config_ptr = loadRobotConfigFromYAML(example_robot_config);
	EXPECT_FALSE(*device_config_ptr == *getExpectedRobotConfig(false));
}

/**
 * @brief Test that we can convert a DeviceConfigMap into source code, compile it, and then load it succesfully.
 * See https://0x00sec.org/t/c-dynamic-loading-of-shared-objects-at-runtime/1498
 *
 * This allows us to store ALL parameters in one YAML and then use them universally across both
 * simulation and hardware and across two different hardware devices (coprocessor and V5 Brain). This also means that ANY
 * robot can have all the relevant info needed to reconstruct it in sim saved in a ROS Bag on the coprocessor.
 *
 * For adding new devices, make sure you can load a default device and a device with all params changed. If it makes it through
 * this test, it should be good to go!
 */
TEST_F(DeviceConfigMapTestFixture, testGenerateCodeFromRobotConfig){
	auto robot_config_ptr = getExpectedRobotConfig(getRandomBool());
	// Create directory for code generation
	std::string codegen_dir = "/tmp/testGenerateCodeFromRobotConfig";
	if(!std::filesystem::exists(std::filesystem::path(codegen_dir))){
		std::filesystem::create_directory(codegen_dir);
	}
	else{
		std::filesystem::remove_all(std::filesystem::path(codegen_dir));
		std::filesystem::create_directory(codegen_dir);
	}

	// Generate code
	std::string include_dir = std::string(getenv("HOME")) + "/VEXU_GHOST/install/";
	std::string src_dir = include_dir + "ghost_v5_interfaces/include/";
	generateCodeFromRobotConfig(robot_config_ptr, src_dir + "test.cpp");

	// Compile
	std::string compile_cmd = "g++ --std=c++17 -fPIC -rdynamic -I" + include_dir +
	                          "ghost_util/include -I" + include_dir + "ghost_v5_interfaces/include -I" + include_dir +
	                          "ghost_estimation/include -I" + include_dir + "ghost_control/include -shared -o" +
	                          codegen_dir + "/test.so " + src_dir + "test.cpp ";
	EXPECT_EQ(std::system(compile_cmd.c_str()), 0);

	// Load library
	std::string lib_dir = codegen_dir + "/test.so";
	void* handle = dlopen(lib_dir.c_str(), RTLD_NOW);
	EXPECT_TRUE(handle != nullptr);

	// Load function symbol
	void *maker = dlsym(handle, "getRobotConfig");
	EXPECT_TRUE(maker != nullptr);

	// Reinterpret to a function we can call
	typedef DeviceConfigMap* (*fptr)();
	fptr func = reinterpret_cast<fptr>(reinterpret_cast<void*>(maker));

	// Call function, wrap new DeviceConfigMap in shared_ptr
	std::shared_ptr<DeviceConfigMap> test_config_map(func());
	EXPECT_EQ(*robot_config_ptr, *test_config_map);
}