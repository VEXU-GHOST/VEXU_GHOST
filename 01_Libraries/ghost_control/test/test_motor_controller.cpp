#include "ghost_control/models/dc_motor_model.hpp"

#include "ghost_estimation/filters/second_order_low_pass_filter.hpp"

#include "ghost_control/motor_controller.hpp"

#include "gtest/gtest.h"

using namespace ghost_control;

class TestMotorController : public ::testing::Test {
protected:

	void SetUp() override {
		// Linker error occurs the instant you call DCMotorModel
		motor_controller = std::make_shared<MotorController>(controller_config, filter_config, model_config);
	}

	MotorController::Config controller_config;
	SecondOrderLowPassFilter::Config filter_config;
	DCMotorModel::Config model_config;

	std::shared_ptr<MotorController> motor_controller;
};

// Controller Activity
TEST_F(TestMotorController, testControllerActive){
	motor_controller->setControlMode(true, true, true, false);
	EXPECT_TRUE(motor_controller->controllerActive());
}

// Velocity Filtering
// Position is currently fixed as in delta d is not manually changed (?) << probably adjust this later
TEST_F(TestMotorController, testNoMovement){
	for(int i = 0; i < 5; i++){
		motor_controller->setControlMode(true, true, true, false);

		motor_controller->updateMotor(0.0, 0.0);
		EXPECT_FLOAT_EQ(0.0, motor_controller->getVelocityFilteredRPM());
	}
}

TEST_F(TestMotorController, testConstantPositiveAcceleration){
	float velocity = 1.0;
	for(int i = 0; i < 10; i++){
		motor_controller->setControlMode(true, true, true, false);

		velocity += 1.0;
		motor_controller->updateMotor(0.0, velocity);

		for(int i = 0; i < 1000; i++){
		}
		EXPECT_FLOAT_EQ(velocity, motor_controller->getVelocityFilteredRPM());
	}
}

TEST_F(TestMotorController, testConstantNegativeAcceleration){
	float velocity = 1.0;
	for(int i = 0; i < 10; i++){
		motor_controller->setControlMode(true, true, true, false);

		velocity -= 1.0;
		motor_controller->updateMotor(0.0, velocity);
		EXPECT_FLOAT_EQ(velocity, motor_controller->getVelocityFilteredRPM());
	}
}

TEST_F(TestMotorController, testIncreasingAcceleration){
	float velocity = 1.0;
	for(int i = 0; i < 10; i++){
		motor_controller->setControlMode(true, true, true, false);

		velocity *= 1.5;
		motor_controller->updateMotor(0.0, velocity);
		EXPECT_FLOAT_EQ(velocity, motor_controller->getVelocityFilteredRPM());
	}
}

TEST_F(TestMotorController, testDecreasingAcceleration){
	float velocity = 1.0;
	for(int i = 0; i < 10; i++){
		motor_controller->setControlMode(true, true, true, false);

		velocity /= 1.5;
		motor_controller->updateMotor(0.0, velocity);
		EXPECT_FLOAT_EQ(velocity, motor_controller->getVelocityFilteredRPM());
	}
}

// PD Controller
TEST_F(TestMotorController, testPositionControl){
	motor_controller->setControlMode(true, false, false, false);
	EXPECT_TRUE(true);
}

TEST_F(TestMotorController, testVelocityControl){
	motor_controller->setControlMode(false, true, false, false);
	EXPECT_TRUE(true);
}

// Other tests as needed

int main(int argc, char **argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
