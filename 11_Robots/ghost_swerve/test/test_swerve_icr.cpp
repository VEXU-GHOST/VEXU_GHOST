#include <ghost_swerve/swerve_model_test_fixture.hpp>

using namespace ghost_swerve::test;
using namespace ghost_swerve;
using namespace ghost_util;


TEST_F(SwerveModelTestFixture, testICRZeroCase){
	m_config.module_type = swerve_type_e::DIFFERENTIAL;
	SwerveModel diff_model(m_config);

	std::unordered_map<std::string, double> steering_positions;
	steering_positions["front_right"] = 45.0;
	steering_positions["front_left"] = -45.0;
	steering_positions["back_right"] = -45.0;
	steering_positions["back_left"] = 45.0;

	std::unordered_map<std::string, double> steering_velocities;
	steering_velocities["front_right"] = 0.0;
	steering_velocities["front_left"] = 0.0;
	steering_velocities["back_right"] = 0.0;
	steering_velocities["back_left"] = 0.0;

	diff_model.updateSteeringStates(steering_positions, steering_velocities);
}

TEST_F(SwerveModelTestFixture, testICRAtBackLeft){
	m_config.module_type = swerve_type_e::DIFFERENTIAL;
	SwerveModel diff_model(m_config);

	std::unordered_map<std::string, double> steering_positions;
	steering_positions["front_right"] = 45.0;
	steering_positions["front_left"] = 90.0;
	steering_positions["back_right"] = 0.0;
	steering_positions["back_left"] = getRandomDouble(600);

	std::unordered_map<std::string, double> steering_velocities;
	steering_velocities["front_right"] = 0.0;
	steering_velocities["front_left"] = 0.0;
	steering_velocities["back_right"] = 0.0;
	steering_velocities["back_left"] = 0.0;

	diff_model.updateSteeringStates(steering_positions, steering_velocities);
}

TEST_F(SwerveModelTestFixture, testICRInfinity){
	m_config.module_type = swerve_type_e::DIFFERENTIAL;
	SwerveModel diff_model(m_config);

	std::unordered_map<std::string, double> steering_positions;
	steering_positions["front_right"] = 0.0;
	steering_positions["front_left"] = 0.0;
	steering_positions["back_right"] = 0.0;
	steering_positions["back_left"] = 0.0;

	std::unordered_map<std::string, double> steering_velocities;
	steering_velocities["front_right"] = 0.0;
	steering_velocities["front_left"] = 0.0;
	steering_velocities["back_right"] = 0.0;
	steering_velocities["back_left"] = 0.0;

	diff_model.updateSteeringStates(steering_positions, steering_velocities);
}