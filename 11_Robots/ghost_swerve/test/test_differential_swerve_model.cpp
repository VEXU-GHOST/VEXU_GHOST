#include <ghost_swerve/swerve_model_test_fixture.hpp>

using namespace ghost_swerve::test;
using namespace ghost_swerve;
using namespace ghost_util;


TEST_F(SwerveModelTestFixture, testDifferentialSwerveJacobianPureSteering){
	// Get random velocity inputs
	auto motor_vel = getRandomFloat();
	auto joint_vels = Eigen::Vector2d(motor_vel, motor_vel);
	auto expected_output = Eigen::Vector2d(
		0.0,
		motor_vel * m_config.steering_ratio);

	EXPECT_TRUE(expected_output.isApprox(m_diff_model_ptr->getModuleJacobian() * joint_vels)) <<
	        "[testDifferentialSwerveJacobianPureSteering] Error: Calculation did not match expected output." <<
	        " Expected: " << std::endl << expected_output << std::endl << " Calculated: " << std::endl <<
	        m_diff_model_ptr->getModuleJacobian() * joint_vels << std::endl;
}

TEST_F(SwerveModelTestFixture, testDifferentialSwerveJacobianPureWheelActuation){
	// Get random velocity inputs
	auto motor_vel = getRandomFloat();
	auto joint_vels = Eigen::Vector2d(motor_vel, -motor_vel);
	auto expected_output = Eigen::Vector2d(
		motor_vel * m_config.wheel_ratio,
		0.0);

	EXPECT_TRUE(expected_output.isApprox(m_diff_model_ptr->getModuleJacobian() * joint_vels)) <<
	        "[testDifferentialSwerveJacobianPureWheelActuation] Error: Calculation did not match expected output." <<
	        " Expected: " << std::endl << expected_output << std::endl << " Calculated: " << std::endl <<
	        m_diff_model_ptr->getModuleJacobian() * joint_vels << std::endl;
}

TEST_F(SwerveModelTestFixture, testDifferentialSwerveJacobianCombinedMotion){
	// Get random velocity inputs
	auto joint_vels = Eigen::Vector2d(getRandomFloat(), getRandomFloat());
	auto vel_avg = (joint_vels[0] + joint_vels[1]) / 2;
	auto vel_diff = joint_vels[0] - joint_vels[1];
	auto expected_output = Eigen::Vector2d(
		vel_diff * m_config.wheel_ratio / 2.0,
		vel_avg * m_config.steering_ratio);

	EXPECT_TRUE(expected_output.isApprox(m_diff_model_ptr->getModuleJacobian() * joint_vels)) <<
	        "[testDifferentialSwerveJacobianCombinedMotion] Error: Calculation did not match expected output." <<
	        " Expected: " << std::endl << expected_output << std::endl << " Calculated: " << std::endl <<
	        m_diff_model_ptr->getModuleJacobian() * joint_vels << std::endl;
}