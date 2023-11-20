#include <ghost_v5_core/utils/load_config_yaml.hpp>
#include "gtest/gtest.h"
#include "yaml-cpp/yaml.h"

using namespace ghost_v5_core::util;
using namespace ghost_v5_core;

class TestLoadConfigYAML : public ::testing::Test {
protected:

	void SetUp() override {
		std::string config_path = std::string(getenv("HOME")) + "/VEXU_GHOST/01_Libraries/ghost_v5_core/test/config/example_robot.yaml";
		config_yaml_ = YAML::LoadFile(config_path);

		// Expected motor configurations
		default_motor_config_ = std::make_shared<V5MotorConfig>();

		drive_motor_config_ = std::make_shared<V5MotorConfig>();
		drive_motor_config_->gearset = ghost_gearset::GEARSET_600;
		drive_motor_config_->encoder_units = ghost_encoder_unit::ENCODER_DEGREES;
		drive_motor_config_->controller_config.pos_gain = 7.5;

		// Expected Motor Interfaces
		left_drive_motor_interface_ptr_ = std::make_shared<V5MotorInterface>();
		left_drive_motor_interface_ptr_->port = 1;
		left_drive_motor_interface_ptr_->type = device_type_e::Motor;
		left_drive_motor_interface_ptr_->config = drive_motor_config_;

		right_drive_motor_interface_ptr_ = std::make_shared<V5MotorInterface>();
		right_drive_motor_interface_ptr_->port = 2;
		right_drive_motor_interface_ptr_->reversed = true;
		right_drive_motor_interface_ptr_->type = device_type_e::Motor;
		right_drive_motor_interface_ptr_->config = drive_motor_config_;

		device_config_map_.emplace("left_drive", left_drive_motor_interface_ptr_);
		device_config_map_.emplace("right_drive", right_drive_motor_interface_ptr_);
	}

	std::shared_ptr<V5MotorConfig> default_motor_config_;
	std::shared_ptr<V5MotorConfig> drive_motor_config_;
	std::shared_ptr<V5MotorInterface> left_drive_motor_interface_ptr_;
	std::shared_ptr<V5MotorInterface> right_drive_motor_interface_ptr_;
	DeviceConfigMap device_config_map_;

	YAML::Node config_yaml_;
};


TEST_F(TestLoadConfigYAML, testLoadMotorModelConfig){
	DCMotorModel::Config test_model_config{};
	test_model_config.free_speed = 1104.0;
	test_model_config.stall_torque = 52.25;
	test_model_config.free_current = 0.9090;
	test_model_config.stall_current = 21.58;
	test_model_config.nominal_voltage = 1200.0;
	test_model_config.gear_ratio = 2915.0;

	DCMotorModel::Config loaded_model_config;
	EXPECT_NO_THROW(loaded_model_config = loadMotorModelConfigFromYAML(config_yaml_["port_configuration"]["device_configurations"]["test_motor_config"]["model"]));
	EXPECT_EQ(test_model_config, loaded_model_config);
}

TEST_F(TestLoadConfigYAML, testLoadMotorControllerConfig){
	MotorController::Config test_controller_config{};
	test_controller_config.pos_gain = 8000.0;
	test_controller_config.vel_gain = 118.0;
	test_controller_config.ff_vel_gain = 400.0;
	test_controller_config.ff_torque_gain = 2587.0;

	MotorController::Config loaded_controller_config;
	EXPECT_NO_THROW(loaded_controller_config = loadMotorControllerConfigFromYAML(config_yaml_["port_configuration"]["device_configurations"]["test_motor_config"]["controller"]));
	EXPECT_EQ(test_controller_config, loaded_controller_config);
}

TEST_F(TestLoadConfigYAML, testLoadLowPassFilterConfig){
	SecondOrderLowPassFilter::Config test_filter_config{};
	test_filter_config.cutoff_frequency = 62.0;
	test_filter_config.damping_ratio = 0.10;
	test_filter_config.timestep = 0.21;

	SecondOrderLowPassFilter::Config loaded_filter_config;
	EXPECT_NO_THROW(loaded_filter_config = loadLowPassFilterConfigFromYAML(config_yaml_["port_configuration"]["device_configurations"]["test_motor_config"]["filter"]));
	EXPECT_EQ(test_filter_config, loaded_filter_config);
}

TEST_F(TestLoadConfigYAML, testLoadDefaultMotorConfigFromYAML){
	auto motor_config = loadV5MotorConfigFromYAML(
		config_yaml_["port_configuration"]["device_configurations"]["default_motor_config"]);
	EXPECT_EQ(*motor_config, *default_motor_config_);
}

TEST_F(TestLoadConfigYAML, testLoadDriveMotorConfigFromYAML){
	auto motor_config = loadV5MotorConfigFromYAML(
		config_yaml_["port_configuration"]["device_configurations"]["drive_motor_config"]);
	EXPECT_EQ(*motor_config, *drive_motor_config_);
}

// TEST_F(TestLoadConfigYAML, testEnforcesUniquePorts){
// }

// TEST_F(TestLoadConfigYAML, testCppCodeGen){
// 	// std::string prefix_lib = fs::current_path().parent_path().string() + "/build/";
// 	// std::string compile_command = "gcc -fPIC -shared -O3 " +
// 	//                               prefix_code + "example.c -o " +
// 	//                               prefix_lib + "lib_example.so";

// 	// std::cout << compile_command << std::endl;
// }