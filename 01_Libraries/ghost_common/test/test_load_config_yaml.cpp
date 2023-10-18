#include <ghost_common/util/load_robot_config.hpp>
#include <ghost_common/v5_robot_config_defs.hpp>
#include "gtest/gtest.h"
#include "yaml-cpp/yaml.h"

using ghost_common::loadEncoderConfigurations;
using ghost_common::loadMotorConfigStruct;
using ghost_common::loadMotorConfigurations;

using ghost_v5_config::encoder_access_helper;
using ghost_v5_config::motor_access_helper;
using ghost_v5_config::MotorConfigStruct;

class TestLoadConfigYAML : public ::testing::Test {
protected:

	void SetUp() override {
		std::string config_path = std::string(getenv("HOME")) + "/VEXU_GHOST/01_Libraries/ghost_common/test/config/example_robot.yaml";
		config_yaml_ = YAML::LoadFile(config_path);
		expected_motor_config_.ctl__pos_gain = 7.5;

		motor_config_map_.emplace("left_drive", motor_access_helper(1, false, expected_motor_config_));
		motor_config_map_.emplace("right_drive", motor_access_helper(2, true, expected_motor_config_));

		encoder_config_map_.emplace("left_drive", encoder_access_helper(3, false));
		encoder_config_map_.emplace("right_drive", encoder_access_helper(4, true));
	}

	MotorConfigStruct expected_motor_config_{};
	std::unordered_map<std::string, motor_access_helper> motor_config_map_;
	std::unordered_map<std::string, encoder_access_helper> encoder_config_map_;

	YAML::Node config_yaml_;
};

TEST_F(TestLoadConfigYAML, testLoadMotorConfigStruct){
	auto motor_config = loadMotorConfigStruct(
		config_yaml_["hardware_configuration"]["motor_configurations"]["drive_motor_config"]);


	EXPECT_EQ(motor_config, expected_motor_config_);
}

TEST_F(TestLoadConfigYAML, testLoadMotorConfigMap){
	auto motor_config = loadMotorConfigurations(
		config_yaml_["hardware_configuration"]["motors"]);
}

TEST_F(TestLoadConfigYAML, testLoadEncoderConfigMap){
	auto encoder_config = loadEncoderConfigurations(
		config_yaml_["hardware_configuration"]["encoders"]);
}

TEST_F(TestLoadConfigYAML, testEnforcesUniquePorts){
}

TEST_F(TestLoadConfigYAML, testCppCodeGen){
	// std::string prefix_lib = fs::current_path().parent_path().string() + "/build/";
	// std::string compile_command = "gcc -fPIC -shared -O3 " +
	//                               prefix_code + "example.c -o " +
	//                               prefix_lib + "lib_example.so";

	// std::cout << compile_command << std::endl;
}

int main(int argc, char **argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}