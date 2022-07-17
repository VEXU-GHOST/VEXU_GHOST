/*
 * Filename: test_dc_motor_model
 * Created Date: Sunday July 17th 2022
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 * 
 * Last Modified: Sunday July 17th 2022 4:09:32 pm
 * Modified By: Maxx Wilson
 */

#include "dc_motor_model.hpp"

#include "gtest/gtest.h"

using dc_motor_model::DCMotorModel;

class TestMotorModel: public ::testing::Test {
    protected:

    void SetUp() override {
        motor_393 = DCMotorModel(100, 1.67, 0.1, 3.6, 7.2);
    }

    DCMotorModel motor_393;
};

TEST_F(TestMotorModel, testMotorInit){
    EXPECT_FLOAT_EQ(0.0, motor_393.getSpeed());
    EXPECT_FLOAT_EQ(0.0, motor_393.getVoltage());
    EXPECT_FLOAT_EQ(0.0, motor_393.getCurrentDraw());
    EXPECT_FLOAT_EQ(0.0, motor_393.getTorqueOutput());
}

TEST_F(TestMotorModel, testMotorNominal){
    motor_393.setMotorInput(1.0, 100);
    EXPECT_FLOAT_EQ(7.2, motor_393.getVoltage());
    EXPECT_FLOAT_EQ(100, motor_393.getSpeed());
    EXPECT_FLOAT_EQ(0.1, motor_393.getCurrentDraw());
    EXPECT_FLOAT_EQ(0.0, motor_393.getTorqueOutput());
    
    motor_393.setMotorInput(1.0, 50);
    EXPECT_FLOAT_EQ(50, motor_393.getSpeed());
    EXPECT_FLOAT_EQ(1.85, motor_393.getCurrentDraw());
    EXPECT_FLOAT_EQ(1.67/2, motor_393.getTorqueOutput());

    motor_393.setMotorInput(1.0, 0);
    EXPECT_FLOAT_EQ(0, motor_393.getSpeed());
    EXPECT_FLOAT_EQ(3.6, motor_393.getCurrentDraw());
    EXPECT_FLOAT_EQ(1.67, motor_393.getTorqueOutput());
}

TEST_F(TestMotorModel, testMotorNominalNegativeSpeed){
    motor_393.setMotorInput(1.0, -100);
    EXPECT_FLOAT_EQ(7.2, motor_393.getVoltage());
    EXPECT_FLOAT_EQ(100, motor_393.getSpeed());
    EXPECT_FLOAT_EQ(0.1, motor_393.getCurrentDraw());
    EXPECT_FLOAT_EQ(0.0, motor_393.getTorqueOutput());
}

TEST_F(TestMotorModel, testMotorNominalCurrentLimit){
    motor_393.setMotorInput(1.0, 100);
    motor_393.setMaxCurrent(1.75);
    EXPECT_FLOAT_EQ(7.2, motor_393.getVoltage());
    EXPECT_FLOAT_EQ(100, motor_393.getSpeed());
    EXPECT_FLOAT_EQ(0.1, motor_393.getCurrentDraw());
    EXPECT_FLOAT_EQ(0.0, motor_393.getTorqueOutput());

    motor_393.setMotorInput(1.0, 0);
    EXPECT_FLOAT_EQ(0, motor_393.getSpeed());
    EXPECT_FLOAT_EQ(1.75, motor_393.getCurrentDraw());
    EXPECT_NEAR(0.787, motor_393.getTorqueOutput(), 0.01);
}

TEST_F(TestMotorModel, testMotorHalfPower){
    motor_393.setMotorInput(0.5, 100);
    EXPECT_FLOAT_EQ(3.6, motor_393.getVoltage());
    EXPECT_FLOAT_EQ(0.05, motor_393.getCurrentDraw());
    EXPECT_FLOAT_EQ(0.0, motor_393.getTorqueOutput());
    
    motor_393.setMotorInput(0.5, 50);
    EXPECT_FLOAT_EQ(1.85/2, motor_393.getCurrentDraw());
    EXPECT_FLOAT_EQ(1.67/4, motor_393.getTorqueOutput());

    motor_393.setMotorInput(0.5, 0);
    EXPECT_FLOAT_EQ(3.6/2, motor_393.getCurrentDraw());
    EXPECT_FLOAT_EQ(1.67/2, motor_393.getTorqueOutput());
}

TEST_F(TestMotorModel, testMotorZeroPower){
    motor_393.setMotorInput(0.0, 100);
    EXPECT_FLOAT_EQ(0, motor_393.getVoltage());
    EXPECT_FLOAT_EQ(0.0, motor_393.getCurrentDraw());
    EXPECT_FLOAT_EQ(0.0, motor_393.getTorqueOutput());
    
    motor_393.setMotorInput(0.0, 50);
    EXPECT_FLOAT_EQ(0.0, motor_393.getCurrentDraw());
    EXPECT_FLOAT_EQ(0.0, motor_393.getTorqueOutput());

    motor_393.setMotorInput(0.0, 0);
    EXPECT_FLOAT_EQ(0.0, motor_393.getCurrentDraw());
    EXPECT_FLOAT_EQ(0.0, motor_393.getTorqueOutput());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}