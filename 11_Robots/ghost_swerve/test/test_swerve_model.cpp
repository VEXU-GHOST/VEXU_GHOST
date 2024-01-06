#include <ghost_swerve/swerve_model.hpp>
#include <gtest/gtest.h>

using namespace ghost_swerve;

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
}

TEST_F(SwerveModelTestFixture, testDifferentialSwerveJacobians){
	config_.module_type = swerve_type_e::DIFFERENTIAL;
	SwerveModel diff_model(config_);
}