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

TEST_F(SwerveModelTestFixture, testConstructors) {
  // Coaxial constructor
  m_config.module_type = swerve_type_e::COAXIAL;
  EXPECT_NO_THROW(auto model = SwerveModel(m_config));

  // Differential constructor
  m_config.module_type = swerve_type_e::DIFFERENTIAL;
  EXPECT_NO_THROW(auto model = SwerveModel(m_config));
}

TEST_F(SwerveModelTestFixture, testDefaultModuleStatesArePopulated) {
  // Check each model
  for (const auto & model : m_models) {
    // Check each module
    for (const auto & [name, _] : m_config.module_positions) {
      EXPECT_NO_THROW(model->getCurrentModuleState(name));
      EXPECT_EQ(model->getCurrentModuleState(name), ModuleState());
    }
  }
}

TEST_F(SwerveModelTestFixture, testMaxBaseVelocities) {
  for (const auto & model : m_models) {
    EXPECT_EQ(model->getMaxBaseLinearVelocity(), m_config.max_wheel_lin_vel);
    EXPECT_EQ(
      model->getMaxBaseAngularVelocity(),
      m_config.max_wheel_lin_vel / (double) Eigen::Vector2d(5.5, 5.5).norm());
  }
}

TEST_F(SwerveModelTestFixture, testModuleJacobians) {
  for (const auto & model : m_models) {
    checkInverse(model->getModuleJacobian(), model->getModuleJacobianInverse());
    checkInverse(model->getModuleJacobianTranspose(), model->getModuleJacobianInverseTranspose());
    EXPECT_TRUE(model->getModuleJacobian().isApprox(model->getModuleJacobianTranspose().transpose()));
  }
}

TEST_F(SwerveModelTestFixture, testInvalidModuleNameThrows) {
  for (const auto & model : m_models) {
    EXPECT_THROW(model->setModuleState("nonexistent", getRandomModuleState()), std::runtime_error);
    EXPECT_THROW(model->getCurrentModuleState("nonexistent"), std::runtime_error);
    EXPECT_THROW(model->getPreviousModuleState("nonexistent"), std::runtime_error);
  }
}

TEST_F(SwerveModelTestFixture, testStateGetterSetters) {
  for (const auto & model : m_models) {
    for (const auto & [name, _] : m_config.module_positions) {
      auto state = getRandomModuleState();
      EXPECT_NO_THROW(model->setModuleState(name, state));

      state.steering_angle = ghost_util::WrapAngle360(state.steering_angle);
      EXPECT_NO_THROW(auto s = model->getCurrentModuleState(name));
      EXPECT_EQ(state, model->getCurrentModuleState(name));
    }
  }
}

TEST_F(SwerveModelTestFixture, testCommandGetterSetters) {
  for (const auto & model : m_models) {
    for (const auto & [name, _] : m_config.module_positions) {
      auto cmd = getRandomModuleCommand();
      EXPECT_NO_THROW(model->setModuleCommand(name, cmd));

      cmd.steering_angle_command = ghost_util::WrapAngle360(cmd.steering_angle_command);
      EXPECT_NO_THROW(auto s = model->getModuleCommand(name));
      EXPECT_EQ(cmd, model->getModuleCommand(name));
    }
  }
}

TEST_F(SwerveModelTestFixture, testLastModuleStatesAreSaved) {
  for (const auto & model : m_models) {
    for (const auto & [name, _] : m_config.module_positions) {
      auto last_state = getRandomModuleState();
      auto curr_state = getRandomModuleState();
      model->setModuleState(name, last_state);
      model->setModuleState(name, curr_state);

      last_state.steering_angle = ghost_util::WrapAngle360(last_state.steering_angle);
      curr_state.steering_angle = ghost_util::WrapAngle360(curr_state.steering_angle);

      EXPECT_EQ(last_state, model->getPreviousModuleState(name));
      EXPECT_EQ(curr_state, model->getCurrentModuleState(name));
    }
  }
}
