#include <filesystem>
#include <dlfcn.h>

#include <ghost_v5_core/devices/base/device_config_map.hpp>
#include <ghost_v5_core/devices/device_config_factory_utils.hpp>

#include <ghost_v5_core/devices/motor/motor_device_config.hpp>
#include <gtest/gtest.h>

using ghost_v5_core::device_type_e;
using ghost_v5_core::DeviceConfigMap;
using ghost_v5_core::MotorDeviceConfig;

using ghost_v5_core::util::generateCodeFromRobotConfig;
using ghost_v5_core::util::loadRobotConfigFromYAML;
class DeviceConfigMapTestFixture : public ::testing::Test {
protected:
	void SetUp() override {
	}
};

TEST_F(DeviceConfigMapTestFixture, testYAMLConstructor){
}

TEST_F(DeviceConfigMapTestFixture, testDeviceMapConstructor){
}

/**
 * @brief Test that we can convert a DeviceConfigMap into source code, compile it, and then load it succesfully.
 * See https://0x00sec.org/t/c-dynamic-loading-of-shared-objects-at-runtime/1498
 */
TEST_F(DeviceConfigMapTestFixture, testGenerateCodeFromRobotConfig){
	// Compare to expected output
	auto robot_config_ptr = std::make_shared<DeviceConfigMap>();
	std::shared_ptr<MotorDeviceConfig> motor_1 = std::make_shared<MotorDeviceConfig>();
	motor_1->name = "motor_1";
	motor_1->port = 1;
	motor_1->type = device_type_e::MOTOR;
	robot_config_ptr->addDeviceConfig(motor_1);

	std::shared_ptr<MotorDeviceConfig> motor_2 = std::make_shared<MotorDeviceConfig>();
	motor_2->name = "motor_2";
	motor_2->port = 2;
	motor_2->type = device_type_e::MOTOR;
	robot_config_ptr->addDeviceConfig(motor_2);

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