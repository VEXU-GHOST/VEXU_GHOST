#include "eigen3/Eigen/Geometry"
#include <ghost_swerve/swerve_model.hpp>
#include <ghost_util/test_util.hpp>
#include <gtest/gtest.h>


using namespace ghost_swerve;
using namespace ghost_util;

class SwerveModelTestFixture : public ::testing::Test {
public:
	void SetUp() override {
		config_.max_wheel_lin_vel = 2.0;
		config_.steering_ratio = 13.0 / 44.0;
		config_.wheel_ratio = 13.0 / 44.0 * 30.0 / 14.0;

		// Mobile robots use forward as X, left as Y, and up as Z so that travelling forward is zero degree heading.
		// No, I don't like it either.
		config_.module_positions["front_right"] = Eigen::Vector2f(5.5, -5.5);
		config_.module_positions["front_left"] = Eigen::Vector2f(5.5, 5.5);
		config_.module_positions["back_right"] = Eigen::Vector2f(-5.5, -5.5);
		config_.module_positions["back_left"] = Eigen::Vector2f(-5.5, 5.5);
	}

	static void checkInverse(Eigen::MatrixXf m, Eigen::MatrixXf m_inv){
		// Matrices are square and equal size
		EXPECT_EQ(m.rows(), m.cols());
		EXPECT_EQ(m_inv.rows(), m_inv.cols());
		EXPECT_EQ(m.rows(), m_inv.rows());

		// Matrix times inverse should be identity
		auto I1 = m * m_inv;
		auto I2 = m_inv * m;

		for(int row = 0; row < m.rows(); row++){
			for(int col = 0; col < m.cols(); col++){
				auto val = (row == col) ? 1.0 : 0.0;
				EXPECT_NEAR(I1(row, col), val, 1e-6);
				EXPECT_NEAR(I2(row, col), val, 1e-6);
			}
		}
	}

	SwerveConfig config_;
};

TEST_F(SwerveModelTestFixture, testConstructors){
	// Coaxial constructor
	config_.module_type = swerve_type_e::COAXIAL;
	EXPECT_NO_THROW(auto model = SwerveModel(config_));

	// Differential constructor
	config_.module_type = swerve_type_e::DIFFERENTIAL;
	EXPECT_NO_THROW(auto model = SwerveModel(config_));
}

TEST_F(SwerveModelTestFixture, testMaxBaseVelocities){
	config_.module_type = swerve_type_e::COAXIAL;
	SwerveModel coax_model(config_);

	config_.module_type = swerve_type_e::DIFFERENTIAL;
	SwerveModel diff_model(config_);

	std::vector<SwerveModel> models{coax_model, diff_model};

	for(const auto& model : models){
		EXPECT_EQ(model.getMaxBaseLinearVelocity(), config_.max_wheel_lin_vel);
		EXPECT_EQ(model.getMaxBaseAngularVelocity(), config_.max_wheel_lin_vel / (double) Eigen::Vector2f(5.5, 5.5).norm());
	}
}

TEST_F(SwerveModelTestFixture, testCoaxialSwerveJacobians){
	config_.module_type = swerve_type_e::COAXIAL;
	SwerveModel coax_model(config_);

	// Get random velocity inputs
	auto joint_vels = Eigen::Vector2f(getRandomFloat(), getRandomFloat());
	auto expected_output = Eigen::Vector2f(
		joint_vels[0] * config_.wheel_ratio,
		joint_vels[1] * config_.steering_ratio);

	EXPECT_TRUE(expected_output.isApprox(coax_model.getModuleJacobian() * joint_vels)) <<
	        "[testCoaxialSwerveJacobians] Error: Calculation did not match expected output." <<
	        " Expected: " << std::endl << expected_output << std::endl << " Calculated: " << std::endl <<
	        coax_model.getModuleJacobian() * joint_vels << std::endl;
	;
	checkInverse(coax_model.getModuleJacobian(), coax_model.getModuleJacobianInverse());
}

TEST_F(SwerveModelTestFixture, testDifferentialSwerveJacobianPureSteering){
	config_.module_type = swerve_type_e::DIFFERENTIAL;
	SwerveModel diff_model(config_);

	// Get random velocity inputs
	auto motor_vel = getRandomFloat();
	auto joint_vels = Eigen::Vector2f(motor_vel, motor_vel);
	auto expected_output = Eigen::Vector2f(
		0.0,
		motor_vel * config_.steering_ratio);

	EXPECT_TRUE(expected_output.isApprox(diff_model.getModuleJacobian() * joint_vels)) <<
	        "[testDifferentialSwerveJacobianPureSteering] Error: Calculation did not match expected output." <<
	        " Expected: " << std::endl << expected_output << std::endl << " Calculated: " << std::endl <<
	        diff_model.getModuleJacobian() * joint_vels << std::endl;
	;
	checkInverse(diff_model.getModuleJacobian(), diff_model.getModuleJacobianInverse());
}

TEST_F(SwerveModelTestFixture, testDifferentialSwerveJacobianPureWheelActuation){
	config_.module_type = swerve_type_e::DIFFERENTIAL;
	SwerveModel diff_model(config_);

	// Get random velocity inputs
	auto motor_vel = getRandomFloat();
	auto joint_vels = Eigen::Vector2f(motor_vel, -motor_vel);
	auto expected_output = Eigen::Vector2f(
		motor_vel * config_.wheel_ratio,
		0.0);

	EXPECT_TRUE(expected_output.isApprox(diff_model.getModuleJacobian() * joint_vels)) <<
	        "[testDifferentialSwerveJacobianPureWheelActuation] Error: Calculation did not match expected output." <<
	        " Expected: " << std::endl << expected_output << std::endl << " Calculated: " << std::endl <<
	        diff_model.getModuleJacobian() * joint_vels << std::endl;
	;
	checkInverse(diff_model.getModuleJacobian(), diff_model.getModuleJacobianInverse());
}

TEST_F(SwerveModelTestFixture, testDifferentialSwerveJacobianCombinedMotion){
	config_.module_type = swerve_type_e::DIFFERENTIAL;
	SwerveModel diff_model(config_);

	// Get random velocity inputs
	auto joint_vels = Eigen::Vector2f(getRandomFloat(), getRandomFloat());
	auto vel_avg = (joint_vels[0] + joint_vels[1]) / 2;
	auto vel_diff = joint_vels[0] - joint_vels[1];
	auto expected_output = Eigen::Vector2f(
		vel_diff * config_.wheel_ratio / 2.0,
		vel_avg * config_.steering_ratio);

	EXPECT_TRUE(expected_output.isApprox(diff_model.getModuleJacobian() * joint_vels)) <<
	        "[testDifferentialSwerveJacobianCombinedMotion] Error: Calculation did not match expected output." <<
	        " Expected: " << std::endl << expected_output << std::endl << " Calculated: " << std::endl <<
	        diff_model.getModuleJacobian() * joint_vels << std::endl;

	checkInverse(diff_model.getModuleJacobian(), diff_model.getModuleJacobianInverse());
}