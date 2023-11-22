#include <filesystem>
#include <dlfcn.h>

#include <ghost_v5_core/devices/base/device_config_map.hpp>
#include <ghost_v5_core/devices/device_config_factory_utils.hpp>

#include <ghost_v5_core/devices/motor/motor_device_config.hpp>
#include <gtest/gtest.h>

using namespace ghost_v5_core::util;
using namespace ghost_v5_core;

class DeviceConfigMapTestFixture : public ::testing::Test {
protected:
	void SetUp() override {
	}
};

TEST_F(DeviceConfigMapTestFixture, testLoadRobotConfigFromYAML){
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
	// Compare to expected output
	auto robot_config_ptr = std::make_shared<DeviceConfigMap>();

	// Default motor, minimally required info.
	std::shared_ptr<MotorDeviceConfig> motor_2 = std::make_shared<MotorDeviceConfig>();
	motor_2->name = "motor_2";
	motor_2->port = 2;
	motor_2->type = device_type_e::MOTOR;
	robot_config_ptr->addDeviceConfig(motor_2);

	// Motor with every parameter changed.
	std::shared_ptr<MotorDeviceConfig> motor_1 = std::make_shared<MotorDeviceConfig>();
	motor_1->name = "motor_1";
	motor_1->port = 1;
	motor_1->type = device_type_e::MOTOR;
	motor_1->reversed = true;
	motor_1->encoder_units = ghost_encoder_unit::ENCODER_ROTATIONS;
	motor_1->gearset = ghost_gearset::GEARSET_100;
	motor_1->brake_mode = ghost_brake_mode::BRAKE_MODE_BRAKE;
	motor_1->filter_config.cutoff_frequency = 62.0;
	motor_1->filter_config.damping_ratio = 0.10;
	motor_1->filter_config.timestep = 0.21;
	motor_1->model_config.free_speed = 1104.0;
	motor_1->model_config.stall_torque = 52.25;
	motor_1->model_config.free_current = 0.9090;
	motor_1->model_config.stall_current = 21.58;
	motor_1->model_config.nominal_voltage = 1200.0;
	motor_1->model_config.gear_ratio = 2915.0;
	motor_1->controller_config.pos_gain = 8000.0;
	motor_1->controller_config.vel_gain = 118.0;
	motor_1->controller_config.ff_vel_gain = 400.0;
	motor_1->controller_config.ff_torque_gain = 2587.0;
	robot_config_ptr->addDeviceConfig(motor_1);

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
	generateCodeFromRobotConfig(robot_config_ptr, src_dir + "test.cpp");

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
	EXPECT_EQ(*robot_config_ptr, *test_config_map);
}