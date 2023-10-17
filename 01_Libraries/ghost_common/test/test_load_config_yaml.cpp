#include <ghost_common/util/load_robot_config.hpp>
#include <ghost_common/v5_robot_config_defs.hpp>
#include "gtest/gtest.h"
#include "yaml-cpp/yaml.h"

using ghost_common::loadEncoderConfigurations;
using ghost_common::loadMotorConfigStruct;
using ghost_common::loadMotorConfigurations;

using ghost_v5_config::MotorConfigStruct;

class TestLoadConfigYAML : public ::testing::Test {
protected:

	void SetUp() override {
		std::string config_path = std::string(getenv("HOME")) + "/VEXU_GHOST/01_Libraries/ghost_common/test/config/example_robot.yaml";
		config_yaml_ = YAML::LoadFile(config_path);
	}

	YAML::Node config_yaml_;
};

TEST_F(TestLoadConfigYAML, testLoadMotorConfigStruct){
	auto motor_config = loadMotorConfigStruct(
		config_yaml_["hardware_configuration"]["motors"]["motor_configurations"]["drive_motor_config"]);
	MotorConfigStruct expected_motor_config{};
	expected_motor_config.ctl__pos_gain = 7.5;

	EXPECT_EQ(motor_config, expected_motor_config);
}

TEST_F(TestLoadConfigYAML, testLoadMotorConfigMap){
}

TEST_F(TestLoadConfigYAML, testLoadEncoderConfigMap){
}

int main(int argc, char **argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}