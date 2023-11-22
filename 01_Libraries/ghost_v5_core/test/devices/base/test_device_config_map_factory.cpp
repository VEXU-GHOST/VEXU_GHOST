#include <filesystem>
#include <dlfcn.h>

#include <ghost_v5_core/devices/base/device_config_map.hpp>
#include <ghost_v5_core/devices/device_config_factory_utils.hpp>
#include <ghost_v5_core/devices/motor/motor_device_config.hpp>

#include <gtest/gtest.h>
#include "yaml-cpp/yaml.h"

using namespace ghost_v5_core::util;
using namespace ghost_v5_core;

class DeviceConfigMapTestFixture : public ::testing::Test {
protected:
	void SetUp() override {
		robot_config_ptr_ = std::make_shared<DeviceConfigMap>();

		// Default motor, minimally required info.
		std::shared_ptr<MotorDeviceConfig> default_motor = std::make_shared<MotorDeviceConfig>();
		default_motor->name = "default_motor";
		default_motor->port = 3;
		default_motor->type = device_type_e::MOTOR;
		robot_config_ptr_->addDeviceConfig(default_motor);

		// Motor with every parameter changed.
		std::shared_ptr<MotorDeviceConfig> left_drive_motor = std::make_shared<MotorDeviceConfig>();
		left_drive_motor->name = "left_drive_motor";
		left_drive_motor->port = 1;
		left_drive_motor->type = device_type_e::MOTOR;
		left_drive_motor->encoder_units = ghost_encoder_unit::ENCODER_DEGREES;
		left_drive_motor->gearset = ghost_gearset::GEARSET_600;
		left_drive_motor->brake_mode = ghost_brake_mode::BRAKE_MODE_COAST;
		left_drive_motor->controller_config.pos_gain = 7.5;
		robot_config_ptr_->addDeviceConfig(left_drive_motor);

		// Motor with every parameter changed.
		std::shared_ptr<MotorDeviceConfig> test_motor = std::make_shared<MotorDeviceConfig>();
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
		robot_config_ptr_->addDeviceConfig(test_motor);
	}

	std::shared_ptr<DeviceConfigMap> robot_config_ptr_;
};

/**
 * @brief Test that we can load a DeviceConfigMap from YAML.
 */
TEST_F(DeviceConfigMapTestFixture, testLoadRobotConfigFromYAML){
	auto example_robot_config = YAML::LoadFile(std::string(getenv("HOME")) + "/VEXU_GHOST/01_Libraries/ghost_v5_core/test/config/example_robot.yaml");
	auto device_config_ptr = loadRobotConfigFromYAML(example_robot_config);
	EXPECT_EQ(*device_config_ptr, *robot_config_ptr_);
}

/**
 * @brief Test that mismatched YAML will fail.
 */
TEST_F(DeviceConfigMapTestFixture, testLoadRobotConfigFromYAMLMismatchPortConfig){
	auto example_robot_config = YAML::LoadFile(std::string(getenv("HOME")) + "/VEXU_GHOST/01_Libraries/ghost_v5_core/test/config/example_robot.yaml");
	example_robot_config["port_configuration"]["devices"]["left_drive_motor"]["port"] = 10;
	auto device_config_ptr = loadRobotConfigFromYAML(example_robot_config);
	EXPECT_FALSE(*device_config_ptr == *robot_config_ptr_);
}

/**
 * @brief Test that mismatched YAML will fail.
 */
TEST_F(DeviceConfigMapTestFixture, testLoadRobotConfigFromYAMLMismatchMotorConfig){
	auto example_robot_config = YAML::LoadFile(std::string(getenv("HOME")) + "/VEXU_GHOST/01_Libraries/ghost_v5_core/test/config/example_robot.yaml");
	example_robot_config["port_configuration"]["device_configurations"]["test_motor_config"]["filter"]["timestep"] = 0.1;
	auto device_config_ptr = loadRobotConfigFromYAML(example_robot_config);
	EXPECT_FALSE(*device_config_ptr == *robot_config_ptr_);
}

/**
 * @brief Test that we can convert a DeviceConfigMap into source code, compile it, and then load it succesfully.
 * See https://0x00sec.org/t/c-dynamic-loading-of-shared-objects-at-runtime/1498
 *
 * This may feel excessive, but it allows us to store ALL parameters in one YAML and then use them universally across both
 * simulation and hardware and across two different hardware devices (coprocessor and V5 Brain). This also means that ANY
 * robot can have all the relevant info needed to reconstruct it in sim saved in a ROS Bag on the coprocessor.
 *
 * For adding new devices, make sure you can load a default device and a device with all params changed. If it makes it through
 * this test, it should be good to go!
 */
TEST_F(DeviceConfigMapTestFixture, testGenerateCodeFromRobotConfig){
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
	std::string src_dir = include_dir + "ghost_v5_core/include/";
	generateCodeFromRobotConfig(robot_config_ptr_, src_dir + "test.cpp");

	// Compile
	std::string compile_cmd = "g++ --std=c++17 -fPIC -rdynamic -I" + include_dir  + " -shared -o " + codegen_dir + "/test.so " + src_dir + "test.cpp";
	std::system(compile_cmd.c_str());

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
	EXPECT_EQ(*robot_config_ptr_, *test_config_map);
}