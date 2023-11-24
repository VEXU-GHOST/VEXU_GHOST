#include <ghost_v5_interfaces/util/load_motor_device_config_yaml.hpp>
#include "gtest/gtest.h"
#include "yaml-cpp/yaml.h"

using namespace ghost_v5_interfaces::util;
using namespace ghost_v5_interfaces;

class TestLoadMotorDeviceConfigYAML : public ::testing::Test {
protected:

	void SetUp() override {
		std::string config_path = std::string(getenv("HOME")) + "/VEXU_GHOST/01_Libraries/ghost_v5_interfaces/test/config/test_load_motor_config.yaml";
		config_yaml_ = YAML::LoadFile(config_path);

		// Expected motor configurations
		motor_empty_config_ = std::make_shared<MotorDeviceConfig>();
		motor_empty_config_->port = 0;
		motor_empty_config_->name = "motor_empty_config";
		motor_empty_config_->type = device_type_e::MOTOR;

		left_drive_motor_config_ = std::make_shared<MotorDeviceConfig>();
		left_drive_motor_config_->gearset = ghost_gearset::GEARSET_600;
		left_drive_motor_config_->encoder_units = ghost_encoder_unit::ENCODER_DEGREES;
		left_drive_motor_config_->controller_config.pos_gain = 7.5;
		left_drive_motor_config_->port = 1;
		left_drive_motor_config_->name = "left_drive_motor";
		left_drive_motor_config_->type = device_type_e::MOTOR;

		right_drive_motor_config_ = std::make_shared<MotorDeviceConfig>();
		right_drive_motor_config_->gearset = ghost_gearset::GEARSET_600;
		right_drive_motor_config_->encoder_units = ghost_encoder_unit::ENCODER_DEGREES;
		right_drive_motor_config_->controller_config.pos_gain = 7.5;
		right_drive_motor_config_->port = 2;
		right_drive_motor_config_->name = "right_drive_motor";
		right_drive_motor_config_->reversed = true;
		right_drive_motor_config_->type = device_type_e::MOTOR;
	}

	std::shared_ptr<MotorDeviceConfig> motor_empty_config_;
	std::shared_ptr<MotorDeviceConfig> motor_no_config_;
	std::shared_ptr<MotorDeviceConfig> left_drive_motor_config_;
	std::shared_ptr<MotorDeviceConfig> right_drive_motor_config_;

	YAML::Node config_yaml_;
};

/**
 * @brief Test that a DCMotorModel::Config struct can be properly loaded from YAML
 */
TEST_F(TestLoadMotorDeviceConfigYAML, testLoadMotorModelConfig){
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

/**
 * @brief Test that a MotorController::Config struct can be properly loaded from YAML
 */
TEST_F(TestLoadMotorDeviceConfigYAML, testLoadMotorControllerConfig){
	MotorController::Config test_controller_config{};
	test_controller_config.pos_gain = 8000.0;
	test_controller_config.vel_gain = 118.0;
	test_controller_config.ff_vel_gain = 400.0;
	test_controller_config.ff_torque_gain = 2587.0;

	MotorController::Config loaded_controller_config;
	EXPECT_NO_THROW(loaded_controller_config = loadMotorControllerConfigFromYAML(config_yaml_["port_configuration"]["device_configurations"]["test_motor_config"]["controller"]));
	EXPECT_EQ(test_controller_config, loaded_controller_config);
}

/**
 * @brief Test that a SecondOrderLowPassFilter::Config struct can be properly loaded from YAML
 */
TEST_F(TestLoadMotorDeviceConfigYAML, testLoadLowPassFilterConfig){
	SecondOrderLowPassFilter::Config test_filter_config{};
	test_filter_config.cutoff_frequency = 62.0;
	test_filter_config.damping_ratio = 0.10;
	test_filter_config.timestep = 0.21;

	SecondOrderLowPassFilter::Config loaded_filter_config;
	EXPECT_NO_THROW(loaded_filter_config = loadLowPassFilterConfigFromYAML(config_yaml_["port_configuration"]["device_configurations"]["test_motor_config"]["filter"]));
	EXPECT_EQ(test_filter_config, loaded_filter_config);
}

/**
 * @brief Test that all fields are properly set when loading the MotorDeviceConfig class
 */
TEST_F(TestLoadMotorDeviceConfigYAML, testLoadCustomMotorConfigFromYAML){
	auto config_node = config_yaml_["port_configuration"];
	std::string motor_name = "right_drive_motor";
	auto config_ptr = std::make_shared<MotorDeviceConfig>();
	loadMotorDeviceConfigFromYAML(config_node, motor_name, config_ptr);
	EXPECT_EQ(*config_ptr, *right_drive_motor_config_);
}

/**
 * @brief Test that missing fields will be populated with the default args for the MotorDeviceConfig class
 */
TEST_F(TestLoadMotorDeviceConfigYAML, testLoadDefaultMotorConfigFromYAML){
	auto config_node = config_yaml_["port_configuration"];
	std::string motor_name = "motor_empty_config";
	auto config_ptr = std::make_shared<MotorDeviceConfig>();
	loadMotorDeviceConfigFromYAML(config_node, motor_name, config_ptr);
	EXPECT_EQ(*config_ptr, *motor_empty_config_);
}

/**
 * @brief Test that if the motor config is not found, it throws.
 */
TEST_F(TestLoadMotorDeviceConfigYAML, testMissingConfigThrows){
	auto config_node = config_yaml_["port_configuration"];
	std::string motor_name = "motor_no_config";
	auto config_ptr = std::make_shared<MotorDeviceConfig>();
	EXPECT_THROW(loadMotorDeviceConfigFromYAML(config_node, motor_name, config_ptr), std::runtime_error);
}

/**
 * @brief Test that if the device config is missing the port attribute, it throws.
 */
TEST_F(TestLoadMotorDeviceConfigYAML, testMissingPortThrowsError){
	auto config_node = config_yaml_["port_configuration"];
	std::string motor_name = "motor_no_port";
	auto config_ptr = std::make_shared<MotorDeviceConfig>();
	EXPECT_THROW(loadMotorDeviceConfigFromYAML(config_node, motor_name, config_ptr), std::runtime_error);
}

/**
 * @brief Test that if the device config is missing the type attribute, it throws
 */
TEST_F(TestLoadMotorDeviceConfigYAML, testMissingTypeThrowsError){
	auto config_node = config_yaml_["port_configuration"];
	std::string motor_name = "motor_no_type";
	auto config_ptr = std::make_shared<MotorDeviceConfig>();
	EXPECT_THROW(loadMotorDeviceConfigFromYAML(config_node, motor_name, config_ptr), std::runtime_error);
}