#include <gtest/gtest.h>
#include <ghost_control/pid_controller.hpp>

using namespace ghost_control;

TEST(PIDControllerTest, ProportionalOnly) {
  PIDGains gains = {1.0, 0.0, 0.0};
  PIDController pid(gains);
  double output = pid.calculateCommand(5.0, 0.0);
  EXPECT_DOUBLE_EQ(output, 5.0);
}

TEST(PIDControllerTest, IntegralOnly_NoLimit) {
  PIDGains gains = {0.0, 1.0, 0.0, 2.0};
  PIDController pid(gains);
  double output1 = pid.calculateCommand(1.0, 0.0);
  double output2 = pid.calculateCommand(1.0, 0.0);
  EXPECT_DOUBLE_EQ(output1, 0.01);
  EXPECT_DOUBLE_EQ(output2, 0.02);
}

TEST(PIDControllerTest, IntegralWindupLimit) {
  PIDGains gains = {0.0, 1.0, 0.0, 2.0}; // limit to ±2
  PIDController pid(gains);

  for (int i = 0; i < 1000; ++i)
    pid.calculateCommand(1.0, 0.0);

  double output = pid.calculateCommand(1.0, 0.0);
  EXPECT_LE(output, 2.0);
}

TEST(PIDControllerTest, DerivativeOnly) {
  PIDGains gains = {0.0, 0.0, 2.0};
  PIDController pid(gains);
  double output = pid.calculateCommand(0.0, -3.0);
  EXPECT_DOUBLE_EQ(output, -6.0);
}

TEST(PIDControllerTest, FullPID) {
  PIDGains gains = {1.0, 0.5, 0.2, 10.0};
  PIDController pid(gains);

  // First call
  double output1 = pid.calculateCommand(2.0, 0.5);  // P=2, I=1, D=0.1
  EXPECT_NEAR(output1, 2.11, 1e-6);

  // Next call with same error
  double output2 = pid.calculateCommand(2.0, 0.5);  // P=2, I=2, D=0.1
  EXPECT_NEAR(output2, 2.12, 1e-6);
}

TEST(PIDControllerTest, IntegralResetOnErrorSignChange) {
  PIDGains gains = {0.0, 1.0, 0.0, 1.0};
  PIDController pid(gains);

  pid.calculateCommand(1.0, 0.0);
  pid.calculateCommand(1.0, 0.0);
  double output = pid.calculateCommand(-1.0, 0.0);  // Should reset integral

  EXPECT_DOUBLE_EQ(output, -0.01);  // Integral sum = -1.0 (not -1 + 2)
}

TEST(PIDControllerTest, ResetMethodClearsIntegral) {
  PIDGains gains = {0.0, 1.0, 0.0, 1.0};
  PIDController pid(gains);

  pid.calculateCommand(1.0, 0.0);
  pid.calculateCommand(1.0, 0.0);
  pid.reset();

  double output = pid.calculateCommand(1.0, 0.0);
  EXPECT_DOUBLE_EQ(output, 0.01);  // Should not accumulate from earlier calls
}