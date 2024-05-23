/*
 *   Copyright (c) 2024 Maxx Wilson
 *   All rights reserved.

 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

#include "ghost_control/models/dc_motor_model.hpp"

#include "gtest/gtest.h"

using ghost_control::DCMotorModel;

class TestMotorModel : public ::testing::Test
{
protected:
  void SetUp() override
  {
    free_speed = 100;
    stall_torque = 1.67;
    free_current = 0.1;
    stall_current = 3.6;
    nominal_voltage = 7.2;

    motor_393_ptr = std::make_shared<DCMotorModel>(
      free_speed,
      stall_torque,
      free_current,
      stall_current,
      nominal_voltage);
  }

  std::shared_ptr<DCMotorModel> motor_393_ptr;
  float stall_torque;
  float free_speed;
  float stall_current;
  float free_current;
  float nominal_voltage;
};

TEST_F(TestMotorModel, testMotorInit) {
  EXPECT_FLOAT_EQ(0.0, motor_393_ptr->getSpeedRPM());
  EXPECT_FLOAT_EQ(0.0, motor_393_ptr->getVoltage());
  EXPECT_FLOAT_EQ(0.0, motor_393_ptr->getMotorCurrent());
  EXPECT_FLOAT_EQ(0.0, motor_393_ptr->getTorqueOutput());
}

TEST_F(TestMotorModel, testMotorNominal) {
  motor_393_ptr->setMotorEffort(1.0);
  motor_393_ptr->setMotorSpeedRPM(free_speed);

  EXPECT_FLOAT_EQ(nominal_voltage, motor_393_ptr->getVoltage());
  EXPECT_FLOAT_EQ(free_speed, motor_393_ptr->getSpeedRPM());
  EXPECT_FLOAT_EQ(free_current, motor_393_ptr->getMotorCurrent());
  EXPECT_FLOAT_EQ(0.0, motor_393_ptr->getTorqueOutput());

  motor_393_ptr->setMotorEffort(1.0);
  motor_393_ptr->setMotorSpeedRPM(free_speed / 2);
  EXPECT_FLOAT_EQ(free_speed / 2, motor_393_ptr->getSpeedRPM());
  EXPECT_FLOAT_EQ((stall_current + free_current) / 2, motor_393_ptr->getMotorCurrent());
  EXPECT_FLOAT_EQ(stall_torque / 2, motor_393_ptr->getTorqueOutput());

  motor_393_ptr->setMotorEffort(1.0);
  motor_393_ptr->setMotorSpeedRPM(0);
  EXPECT_FLOAT_EQ(0, motor_393_ptr->getSpeedRPM());
  EXPECT_FLOAT_EQ(stall_current, motor_393_ptr->getMotorCurrent());
  EXPECT_FLOAT_EQ(stall_torque, motor_393_ptr->getTorqueOutput());
}

TEST_F(TestMotorModel, testMotorNominalNegativeSpeed) {
  motor_393_ptr->setMotorEffort(-1.0);
  motor_393_ptr->setMotorSpeedRPM(-free_speed);
  EXPECT_FLOAT_EQ(-nominal_voltage, motor_393_ptr->getVoltage());
  EXPECT_FLOAT_EQ(-free_speed, motor_393_ptr->getSpeedRPM());
  EXPECT_FLOAT_EQ(-free_current, motor_393_ptr->getMotorCurrent());
  EXPECT_FLOAT_EQ(0.0, motor_393_ptr->getTorqueOutput());

  motor_393_ptr->setMotorEffort(1.0);
  motor_393_ptr->setMotorSpeedRPM(-free_speed);
  EXPECT_FLOAT_EQ(nominal_voltage, motor_393_ptr->getVoltage());
  EXPECT_FLOAT_EQ(-free_speed, motor_393_ptr->getSpeedRPM());
  EXPECT_FLOAT_EQ(stall_torque * 2, motor_393_ptr->getTorqueOutput());
}

TEST_F(TestMotorModel, testMotorHalfPower) {
  motor_393_ptr->setMotorEffort(0.5);
  motor_393_ptr->setMotorSpeedRPM(free_speed / 2);
  EXPECT_FLOAT_EQ(stall_current, motor_393_ptr->getVoltage());
  EXPECT_FLOAT_EQ(free_current / 2, motor_393_ptr->getMotorCurrent());
  EXPECT_FLOAT_EQ(0, motor_393_ptr->getTorqueOutput());

  motor_393_ptr->setMotorEffort(0.5);
  motor_393_ptr->setMotorSpeedRPM(free_speed);
  EXPECT_FLOAT_EQ(free_current - stall_current / 2, motor_393_ptr->getMotorCurrent());
  EXPECT_FLOAT_EQ(-stall_torque / 2, motor_393_ptr->getTorqueOutput());

  motor_393_ptr->setMotorEffort(0.5);
  motor_393_ptr->setMotorSpeedRPM(0);
  EXPECT_FLOAT_EQ(stall_current / 2, motor_393_ptr->getMotorCurrent());
  EXPECT_FLOAT_EQ(stall_torque / 2, motor_393_ptr->getTorqueOutput());

  motor_393_ptr->setMotorEffort(0.5);
  motor_393_ptr->setMotorSpeedRPM(-free_speed / 2);
  EXPECT_FLOAT_EQ(
    free_current / 2 + (stall_current - free_current),
    motor_393_ptr->getMotorCurrent());
  EXPECT_FLOAT_EQ(stall_torque, motor_393_ptr->getTorqueOutput());
}

TEST_F(TestMotorModel, testMotorHalfPowerNegative) {
  motor_393_ptr->setMotorEffort(-0.5);
  motor_393_ptr->setMotorSpeedRPM(-free_speed);
  EXPECT_FLOAT_EQ(-nominal_voltage / 2, motor_393_ptr->getVoltage());
  EXPECT_FLOAT_EQ(
    -free_current / 2 + (stall_current - free_current) / 2,
    motor_393_ptr->getMotorCurrent());
  EXPECT_FLOAT_EQ(stall_torque / 2, motor_393_ptr->getTorqueOutput());

  motor_393_ptr->setMotorEffort(-0.5);
  motor_393_ptr->setMotorSpeedRPM(-50);
  EXPECT_FLOAT_EQ(-free_current / 2, motor_393_ptr->getMotorCurrent());
  EXPECT_FLOAT_EQ(0.0, motor_393_ptr->getTorqueOutput());

  motor_393_ptr->setMotorEffort(-0.5);
  motor_393_ptr->setMotorSpeedRPM(0);
  EXPECT_FLOAT_EQ(-stall_current / 2, motor_393_ptr->getMotorCurrent());
  EXPECT_FLOAT_EQ(-stall_torque / 2, motor_393_ptr->getTorqueOutput());
}

TEST_F(TestMotorModel, testMotorZeroPower) {
  motor_393_ptr->setMotorEffort(0.0);
  motor_393_ptr->setMotorSpeedRPM(free_speed);
  EXPECT_FLOAT_EQ(0, motor_393_ptr->getVoltage());
  EXPECT_FLOAT_EQ(free_current - stall_current, motor_393_ptr->getMotorCurrent());
  EXPECT_FLOAT_EQ(-stall_torque, motor_393_ptr->getTorqueOutput());

  motor_393_ptr->setMotorEffort(0.0);
  motor_393_ptr->setMotorSpeedRPM(50);
  EXPECT_FLOAT_EQ((free_current - stall_current) / 2, motor_393_ptr->getMotorCurrent());
  EXPECT_FLOAT_EQ(-stall_torque / 2, motor_393_ptr->getTorqueOutput());

  motor_393_ptr->setMotorEffort(0.0);
  motor_393_ptr->setMotorSpeedRPM(0);
  EXPECT_FLOAT_EQ(0.0, motor_393_ptr->getMotorCurrent());
  EXPECT_FLOAT_EQ(0.0, motor_393_ptr->getTorqueOutput());
}

// TEST_F(TestMotorModel, testMotorGearRatio){
//     motor_393_ptr->setGearRatio(0.5);
//     motor_393_ptr->setMotorEffort(1.0);
//     motor_393_ptr->setMotorSpeedRPM(50);
//     EXPECT_FLOAT_EQ(nominal_voltage, motor_393_ptr->getVoltage());
//     EXPECT_FLOAT_EQ(50, motor_393_ptr->getSpeedRPM());
//     EXPECT_FLOAT_EQ(free_current, motor_393_ptr->getMotorCurrent());
//     EXPECT_FLOAT_EQ(0.0, motor_393_ptr->getTorqueOutput());

//     motor_393_ptr->setMotorEffort(1.0);
//     motor_393_ptr->setMotorSpeedRPM(25);
//     EXPECT_FLOAT_EQ(25, motor_393_ptr->getSpeedRPM());
//     EXPECT_FLOAT_EQ(1.85, motor_393_ptr->getMotorCurrent());
//     EXPECT_FLOAT_EQ(stall_torque, motor_393_ptr->getTorqueOutput());

//     motor_393_ptr->setMotorEffort(1.0);
//     motor_393_ptr->setMotorSpeedRPM(0);
//     EXPECT_FLOAT_EQ(0, motor_393_ptr->getSpeedRPM());
//     EXPECT_FLOAT_EQ(stall_current, motor_393_ptr->getMotorCurrent());
//     EXPECT_FLOAT_EQ(2*stall_torque, motor_393_ptr->getTorqueOutput());
// }
