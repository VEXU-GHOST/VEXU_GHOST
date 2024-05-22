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
