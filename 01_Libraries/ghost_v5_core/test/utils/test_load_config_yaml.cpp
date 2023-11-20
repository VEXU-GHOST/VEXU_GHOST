#include <ghost_v5_core/util/load_robot_config.hpp>
#include "gtest/gtest.h"
#include "yaml-cpp/yaml.h"

using namespace ghost_v5_core::util;
using namespace ghost_v5_core;


class TestLoadConfigYAML : public ::testing::Test {
protected:

	void SetUp() override {
		std::string config_path = std::string(getenv("HOME")) + "/VEXU_GHOST/01_Libraries/ghost_v5_core/test/config/example_robot.yaml";
		config_yaml_ = YAML::LoadFile(config_path);
		expected_motor_config_.controller.pos_gain = 7.5;

		motor_config_map_.emplace("left_drive", motor_access_helper(1, false, expected_motor_config_));
		motor_config_map_.emplace("right_drive", motor_access_helper(2, true, expected_motor_config_));

		encoder_config_map_.emplace("left_drive", encoder_access_helper(3, false));
		encoder_config_map_.emplace("right_drive", encoder_access_helper(4, true));
	}

	V5MotorInterfaceConfig expected_motor_config_{};
	std::unordered_map<std::string, motor_access_helper> motor_config_map_;
	std::unordered_map<std::string, encoder_access_helper> encoder_config_map_;

	YAML::Node config_yaml_;
};

TEST_F(TestLoadConfigYAML, testLoadMotorModelConfig){
	MotorModelConfig default_model_config{};
	MotorModelConfig loaded_model_config;
	EXPECT_NO_THROW(loaded_model_config = loadMotorModelConfigFromYAML(config_yaml_["hardware_configuration"]["motor_configurations"]["default_config"]["model"]));
	EXPECT_EQ(default_model_config, loaded_model_config);
}

TEST_F(TestLoadConfigYAML, testLoadMotorControllerConfig){
	MotorControllerConfig default_controller_config{};
	MotorControllerConfig loaded_controller_config;
	EXPECT_NO_THROW(loaded_controller_config = loadMotorControllerConfigFromYAML(config_yaml_["hardware_configuration"]["motor_configurations"]["default_config"]["controller"]));
	EXPECT_EQ(default_controller_config, loaded_controller_config);
}

TEST_F(TestLoadConfigYAML, testLoadLowPassFilterConfig){
	LowPassFilterConfig default_filter_config{};
	LowPassFilterConfig loaded_filter_config;
	EXPECT_NO_THROW(loaded_filter_config = loadLowPassFilterConfigFromYAML(config_yaml_["hardware_configuration"]["motor_configurations"]["default_config"]["filter"]));
	EXPECT_EQ(default_filter_config, loaded_filter_config);
}

TEST_F(TestLoadConfigYAML, testLoadV5MotorInterfaceConfigFromYAML){
	auto motor_config = loadV5MotorInterfaceConfigFromYAML(
		config_yaml_["hardware_configuration"]["motor_configurations"]["drive_motor_config"]);
	EXPECT_EQ(motor_config, expected_motor_config_);
}

// TEST_F(TestLoadConfigYAML, testLoadMotorConfigMap){
// 	auto motor_config = loadMotorConfigurations(
// 		config_yaml_["hardware_configuration"]["motors"]);
// }

// TEST_F(TestLoadConfigYAML, testLoadEncoderConfigMap){
// 	auto encoder_config = loadEncoderConfigurations(
// 		config_yaml_["hardware_configuration"]["encoders"]);
// }

// TEST_F(TestLoadConfigYAML, testEnforcesUniquePorts){
// }

// TEST_F(TestLoadConfigYAML, testCppCodeGen){
// 	// std::string prefix_lib = fs::current_path().parent_path().string() + "/build/";
// 	// std::string compile_command = "gcc -fPIC -shared -O3 " +
// 	//                               prefix_code + "example.c -o " +
// 	//                               prefix_lib + "lib_example.so";

// 	// std::cout << compile_command << std::endl;
// }

int main(int argc, char **argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}