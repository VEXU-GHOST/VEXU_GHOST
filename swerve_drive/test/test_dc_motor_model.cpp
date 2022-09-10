/*
 * Filename: test_dc_motor_model
 * Created Date: Sunday July 17th 2022
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 * 
 * Last Modified: Sunday July 17th 2022 4:09:32 pm
 * Modified By: Maxx Wilson
 */

#include "simulator_plugins/dc_motor_model.hpp"

#include "gtest/gtest.h"

using dc_motor_model::DCMotorModel;

class TestMotorModel: public ::testing::Test {
    protected:

    void SetUp() override {
        free_speed = 100;
        stall_torque = 1.67;
        free_current = 0.1;
        stall_current = 3.6;
        nominal_voltage = 7.2;

        motor_393 = DCMotorModel(
            free_speed,
            stall_torque,
            free_current,
            stall_current,
            nominal_voltage);
    }

    DCMotorModel motor_393;
    float stall_torque;
    float free_speed;
    float stall_current;
    float free_current;
    float nominal_voltage;
};

TEST_F(TestMotorModel, testMotorInit){
    EXPECT_FLOAT_EQ(0.0, motor_393.getSpeedRPM());
    EXPECT_FLOAT_EQ(0.0, motor_393.getVoltage());
    EXPECT_FLOAT_EQ(0.0, motor_393.getMotorCurrent());
    EXPECT_FLOAT_EQ(0.0, motor_393.getTorqueOutput());
}

TEST_F(TestMotorModel, testMotorNominal){
    motor_393.setMotorEffort(1.0);
    motor_393.setMotorSpeedRPM(free_speed);

    EXPECT_FLOAT_EQ(nominal_voltage, motor_393.getVoltage());
    EXPECT_FLOAT_EQ(free_speed, motor_393.getSpeedRPM());
    EXPECT_FLOAT_EQ(free_current, motor_393.getMotorCurrent());
    EXPECT_FLOAT_EQ(0.0, motor_393.getTorqueOutput());
    
    motor_393.setMotorEffort(1.0);
    motor_393.setMotorSpeedRPM(free_speed/2);
    EXPECT_FLOAT_EQ(free_speed/2, motor_393.getSpeedRPM());
    EXPECT_FLOAT_EQ((stall_current+free_current)/2, motor_393.getMotorCurrent());
    EXPECT_FLOAT_EQ(stall_torque/2, motor_393.getTorqueOutput());

    motor_393.setMotorEffort(1.0);
    motor_393.setMotorSpeedRPM(0);
    EXPECT_FLOAT_EQ(0, motor_393.getSpeedRPM());
    EXPECT_FLOAT_EQ(stall_current, motor_393.getMotorCurrent());
    EXPECT_FLOAT_EQ(stall_torque, motor_393.getTorqueOutput());
}

TEST_F(TestMotorModel, testMotorNominalNegativeSpeed){
    motor_393.setMotorEffort(-1.0);
    motor_393.setMotorSpeedRPM(-free_speed);
    EXPECT_FLOAT_EQ(-nominal_voltage, motor_393.getVoltage());
    EXPECT_FLOAT_EQ(-free_speed, motor_393.getSpeedRPM());
    EXPECT_FLOAT_EQ(-free_current, motor_393.getMotorCurrent());
    EXPECT_FLOAT_EQ(0.0, motor_393.getTorqueOutput());

    motor_393.setMotorEffort(1.0);
    motor_393.setMotorSpeedRPM(-free_speed);
    EXPECT_FLOAT_EQ(nominal_voltage, motor_393.getVoltage());
    EXPECT_FLOAT_EQ(-free_speed, motor_393.getSpeedRPM());
    EXPECT_FLOAT_EQ(stall_torque*2, motor_393.getTorqueOutput());}

TEST_F(TestMotorModel, testMotorHalfPower){
    motor_393.setMotorEffort(0.5);
    motor_393.setMotorSpeedRPM(free_speed/2);
    EXPECT_FLOAT_EQ(stall_current, motor_393.getVoltage());
    EXPECT_FLOAT_EQ(free_current/2, motor_393.getMotorCurrent());
    EXPECT_FLOAT_EQ(0, motor_393.getTorqueOutput());
    
    motor_393.setMotorEffort(0.5);
    motor_393.setMotorSpeedRPM(free_speed);
    EXPECT_FLOAT_EQ(free_current - stall_current/2, motor_393.getMotorCurrent());
    EXPECT_FLOAT_EQ(-stall_torque/2, motor_393.getTorqueOutput());

    motor_393.setMotorEffort(0.5);
    motor_393.setMotorSpeedRPM(0);
    EXPECT_FLOAT_EQ(stall_current/2, motor_393.getMotorCurrent());
    EXPECT_FLOAT_EQ(stall_torque/2, motor_393.getTorqueOutput());

    motor_393.setMotorEffort(0.5);
    motor_393.setMotorSpeedRPM(-free_speed/2);
    EXPECT_FLOAT_EQ(free_current/2 + (stall_current-free_current), motor_393.getMotorCurrent());
    EXPECT_FLOAT_EQ(stall_torque, motor_393.getTorqueOutput());
}

TEST_F(TestMotorModel, testMotorHalfPowerNegative){
    motor_393.setMotorEffort(-0.5);
    motor_393.setMotorSpeedRPM(-free_speed);
    EXPECT_FLOAT_EQ(-nominal_voltage/2, motor_393.getVoltage());
    EXPECT_FLOAT_EQ(-free_current/2 + (stall_current-free_current)/2, motor_393.getMotorCurrent());
    EXPECT_FLOAT_EQ(stall_torque/2, motor_393.getTorqueOutput());
    
    motor_393.setMotorEffort(-0.5);
    motor_393.setMotorSpeedRPM(-50);
    EXPECT_FLOAT_EQ(-free_current/2, motor_393.getMotorCurrent());
    EXPECT_FLOAT_EQ(0.0, motor_393.getTorqueOutput());

    motor_393.setMotorEffort(-0.5);
    motor_393.setMotorSpeedRPM(0);
    EXPECT_FLOAT_EQ(-stall_current/2, motor_393.getMotorCurrent());
    EXPECT_FLOAT_EQ(-stall_torque/2, motor_393.getTorqueOutput());
}

TEST_F(TestMotorModel, testMotorZeroPower){
    motor_393.setMotorEffort(0.0);
    motor_393.setMotorSpeedRPM(free_speed);
    EXPECT_FLOAT_EQ(0, motor_393.getVoltage());
    EXPECT_FLOAT_EQ(free_current - stall_current, motor_393.getMotorCurrent());
    EXPECT_FLOAT_EQ(-stall_torque, motor_393.getTorqueOutput());
    
    motor_393.setMotorEffort(0.0);
    motor_393.setMotorSpeedRPM(50);
    EXPECT_FLOAT_EQ((free_current-stall_current)/2, motor_393.getMotorCurrent());
    EXPECT_FLOAT_EQ(-stall_torque/2, motor_393.getTorqueOutput());

    motor_393.setMotorEffort(0.0);
    motor_393.setMotorSpeedRPM(0);
    EXPECT_FLOAT_EQ(0.0, motor_393.getMotorCurrent());
    EXPECT_FLOAT_EQ(0.0, motor_393.getTorqueOutput());
}

// TEST_F(TestMotorModel, testMotorGearRatio){
//     motor_393.setGearRatio(0.5);
//     motor_393.setMotorEffort(1.0);
//     motor_393.setMotorSpeedRPM(50);
//     EXPECT_FLOAT_EQ(nominal_voltage, motor_393.getVoltage());
//     EXPECT_FLOAT_EQ(50, motor_393.getSpeedRPM());
//     EXPECT_FLOAT_EQ(free_current, motor_393.getMotorCurrent());
//     EXPECT_FLOAT_EQ(0.0, motor_393.getTorqueOutput());
    
//     motor_393.setMotorEffort(1.0);
//     motor_393.setMotorSpeedRPM(25);
//     EXPECT_FLOAT_EQ(25, motor_393.getSpeedRPM());
//     EXPECT_FLOAT_EQ(1.85, motor_393.getMotorCurrent());
//     EXPECT_FLOAT_EQ(stall_torque, motor_393.getTorqueOutput());

//     motor_393.setMotorEffort(1.0);
//     motor_393.setMotorSpeedRPM(0);
//     EXPECT_FLOAT_EQ(0, motor_393.getSpeedRPM());
//     EXPECT_FLOAT_EQ(stall_current, motor_393.getMotorCurrent());
//     EXPECT_FLOAT_EQ(2*stall_torque, motor_393.getTorqueOutput());
// }

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}