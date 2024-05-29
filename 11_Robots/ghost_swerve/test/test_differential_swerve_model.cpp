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

#include <ghost_swerve/swerve_model_test_fixture.hpp>

using namespace ghost_swerve::test;
using namespace ghost_swerve;
using namespace ghost_util;


TEST_F(SwerveModelTestFixture, testDifferentialSwerveJacobianPureSteering) {
  // Get random velocity inputs
  auto motor_vel = getRandomFloat();
  auto joint_vels = Eigen::Vector2d(motor_vel, motor_vel);
  auto expected_output = Eigen::Vector2d(
    0.0,
    motor_vel * m_config.steering_ratio);

  EXPECT_TRUE(expected_output.isApprox(m_diff_model_ptr->getModuleJacobian() * joint_vels)) <<
    "[testDifferentialSwerveJacobianPureSteering] Error: Calculation did not match expected output."
                                                                                            <<
    " Expected: " << std::endl << expected_output << std::endl << " Calculated: " << std::endl <<
    m_diff_model_ptr->getModuleJacobian() * joint_vels << std::endl;
}

TEST_F(SwerveModelTestFixture, testDifferentialSwerveJacobianPureWheelActuation) {
  // Get random velocity inputs
  auto motor_vel = getRandomFloat();
  auto joint_vels = Eigen::Vector2d(motor_vel, -motor_vel);
  auto expected_output = Eigen::Vector2d(
    motor_vel * m_config.wheel_ratio,
    0.0);

  EXPECT_TRUE(expected_output.isApprox(m_diff_model_ptr->getModuleJacobian() * joint_vels)) <<
    "[testDifferentialSwerveJacobianPureWheelActuation] Error: Calculation did not match expected output."
                                                                                            <<
    " Expected: " << std::endl << expected_output << std::endl << " Calculated: " << std::endl <<
    m_diff_model_ptr->getModuleJacobian() * joint_vels << std::endl;
}

TEST_F(SwerveModelTestFixture, testDifferentialSwerveJacobianCombinedMotion) {
  // Get random velocity inputs
  auto joint_vels = Eigen::Vector2d(getRandomFloat(), getRandomFloat());
  auto vel_avg = (joint_vels[0] + joint_vels[1]) / 2;
  auto vel_diff = joint_vels[0] - joint_vels[1];
  auto expected_output = Eigen::Vector2d(
    vel_diff * m_config.wheel_ratio / 2.0,
    vel_avg * m_config.steering_ratio);

  EXPECT_TRUE(expected_output.isApprox(m_diff_model_ptr->getModuleJacobian() * joint_vels)) <<
    "[testDifferentialSwerveJacobianCombinedMotion] Error: Calculation did not match expected output."
                                                                                            <<
    " Expected: " << std::endl << expected_output << std::endl << " Calculated: " << std::endl <<
    m_diff_model_ptr->getModuleJacobian() * joint_vels << std::endl;
}
