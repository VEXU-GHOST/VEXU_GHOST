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

TEST_F(SwerveModelTestFixture, testCoaxialSwerveJacobians) {
  // Get random velocity inputs
  auto joint_vels = Eigen::Vector2d(getRandomFloat(), getRandomFloat());
  auto expected_output = Eigen::Vector2d(
    joint_vels[0] * m_config.wheel_ratio,
    joint_vels[1] * m_config.steering_ratio);

  EXPECT_TRUE(expected_output.isApprox(m_coax_model_ptr->getModuleJacobian() * joint_vels)) <<
    "[testCoaxialSwerveJacobians] Error: Calculation did not match expected output." <<
    " Expected: " << std::endl << expected_output << std::endl << " Calculated: " << std::endl <<
    m_coax_model_ptr->getModuleJacobian() * joint_vels << std::endl;
}
