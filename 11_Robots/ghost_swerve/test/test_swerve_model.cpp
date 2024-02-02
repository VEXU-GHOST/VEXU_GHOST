#include "eigen3/Eigen/Geometry"
#include <ghost_swerve/swerve_model.hpp>
#include <ghost_util/test_util.hpp>
#include <gtest/gtest.h>


using namespace ghost_swerve;
using namespace ghost_util;

class SwerveModelTestFixture : public ::testing::Test {
public:
	void SetUp() override {
		m_config.max_wheel_lin_vel = 2.0;
		m_config.steering_ratio = 13.0 / 44.0;
		m_config.wheel_ratio = 13.0 / 44.0 * 30.0 / 14.0;

		// Mobile robots use forward as X, left as Y, and up as Z so that travelling forward is zero degree heading.
		// No, I don't like it either.
		m_config.module_positions["front_right"] = Eigen::Vector2d(5.5, -5.5);
		m_config.module_positions["front_left"] = Eigen::Vector2d(5.5, 5.5);
		m_config.module_positions["back_right"] = Eigen::Vector2d(-5.5, -5.5);
		m_config.module_positions["back_left"] = Eigen::Vector2d(-5.5, 5.5);
	}

	static void checkInverse(Eigen::MatrixXd m, Eigen::MatrixXd m_inv){
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
				EXPECT_NEAR(I1(row, col), val, m_eps);
				EXPECT_NEAR(I2(row, col), val, m_eps);
			}
		}
	}

	SwerveConfig m_config;
	static constexpr double m_eps = 1e-6;
};

TEST_F(SwerveModelTestFixture, testConstructors){
	// Coaxial constructor
	m_config.module_type = swerve_type_e::COAXIAL;
	EXPECT_NO_THROW(auto model = SwerveModel(m_config));

	// Differential constructor
	m_config.module_type = swerve_type_e::DIFFERENTIAL;
	EXPECT_NO_THROW(auto model = SwerveModel(m_config));
}

TEST_F(SwerveModelTestFixture, testDefaultModuleStatesArePopulated){
	m_config.module_type = swerve_type_e::COAXIAL;
	SwerveModel coax_model(m_config);

	m_config.module_type = swerve_type_e::DIFFERENTIAL;
	SwerveModel diff_model(m_config);
	for(const auto& [name, val] : m_config.module_positions){
		EXPECT_NO_THROW(coax_model.getModuleState(name));
		EXPECT_NO_THROW(diff_model.getModuleState(name));
		EXPECT_EQ(coax_model.getModuleState(name), ModuleState());
		EXPECT_EQ(diff_model.getModuleState(name), ModuleState());
	}
}

TEST_F(SwerveModelTestFixture, testMaxBaseVelocities){
	m_config.module_type = swerve_type_e::COAXIAL;
	SwerveModel coax_model(m_config);

	m_config.module_type = swerve_type_e::DIFFERENTIAL;
	SwerveModel diff_model(m_config);

	std::vector<SwerveModel> models{coax_model, diff_model};

	for(const auto& model : models){
		EXPECT_EQ(model.getMaxBaseLinearVelocity(), m_config.max_wheel_lin_vel);
		EXPECT_EQ(model.getMaxBaseAngularVelocity(), m_config.max_wheel_lin_vel / (double) Eigen::Vector2d(5.5, 5.5).norm());
	}
}

TEST_F(SwerveModelTestFixture, testCoaxialSwerveJacobians){
	m_config.module_type = swerve_type_e::COAXIAL;
	SwerveModel coax_model(m_config);

	// Get random velocity inputs
	auto joint_vels = Eigen::Vector2d(getRandomFloat(), getRandomFloat());
	auto expected_output = Eigen::Vector2d(
		joint_vels[0] * m_config.wheel_ratio,
		joint_vels[1] * m_config.steering_ratio);

	EXPECT_TRUE(expected_output.isApprox(coax_model.getModuleJacobian() * joint_vels)) <<
	        "[testCoaxialSwerveJacobians] Error: Calculation did not match expected output." <<
	        " Expected: " << std::endl << expected_output << std::endl << " Calculated: " << std::endl <<
	        coax_model.getModuleJacobian() * joint_vels << std::endl;
	;
	checkInverse(coax_model.getModuleJacobian(), coax_model.getModuleJacobianInverse());
}

TEST_F(SwerveModelTestFixture, testDifferentialSwerveJacobianPureSteering){
	m_config.module_type = swerve_type_e::DIFFERENTIAL;
	SwerveModel diff_model(m_config);

	// Get random velocity inputs
	auto motor_vel = getRandomFloat();
	auto joint_vels = Eigen::Vector2d(motor_vel, motor_vel);
	auto expected_output = Eigen::Vector2d(
		0.0,
		motor_vel * m_config.steering_ratio);

	EXPECT_TRUE(expected_output.isApprox(diff_model.getModuleJacobian() * joint_vels)) <<
	        "[testDifferentialSwerveJacobianPureSteering] Error: Calculation did not match expected output." <<
	        " Expected: " << std::endl << expected_output << std::endl << " Calculated: " << std::endl <<
	        diff_model.getModuleJacobian() * joint_vels << std::endl;
	;
	checkInverse(diff_model.getModuleJacobian(), diff_model.getModuleJacobianInverse());
}

TEST_F(SwerveModelTestFixture, testDifferentialSwerveJacobianPureWheelActuation){
	m_config.module_type = swerve_type_e::DIFFERENTIAL;
	SwerveModel diff_model(m_config);

	// Get random velocity inputs
	auto motor_vel = getRandomFloat();
	auto joint_vels = Eigen::Vector2d(motor_vel, -motor_vel);
	auto expected_output = Eigen::Vector2d(
		motor_vel * m_config.wheel_ratio,
		0.0);

	EXPECT_TRUE(expected_output.isApprox(diff_model.getModuleJacobian() * joint_vels)) <<
	        "[testDifferentialSwerveJacobianPureWheelActuation] Error: Calculation did not match expected output." <<
	        " Expected: " << std::endl << expected_output << std::endl << " Calculated: " << std::endl <<
	        diff_model.getModuleJacobian() * joint_vels << std::endl;
	;
	checkInverse(diff_model.getModuleJacobian(), diff_model.getModuleJacobianInverse());
}

TEST_F(SwerveModelTestFixture, testDifferentialSwerveJacobianCombinedMotion){
	m_config.module_type = swerve_type_e::DIFFERENTIAL;
	SwerveModel diff_model(m_config);

	// Get random velocity inputs
	auto joint_vels = Eigen::Vector2d(getRandomFloat(), getRandomFloat());
	auto vel_avg = (joint_vels[0] + joint_vels[1]) / 2;
	auto vel_diff = joint_vels[0] - joint_vels[1];
	auto expected_output = Eigen::Vector2d(
		vel_diff * m_config.wheel_ratio / 2.0,
		vel_avg * m_config.steering_ratio);

	EXPECT_TRUE(expected_output.isApprox(diff_model.getModuleJacobian() * joint_vels)) <<
	        "[testDifferentialSwerveJacobianCombinedMotion] Error: Calculation did not match expected output." <<
	        " Expected: " << std::endl << expected_output << std::endl << " Calculated: " << std::endl <<
	        diff_model.getModuleJacobian() * joint_vels << std::endl;

	checkInverse(diff_model.getModuleJacobian(), diff_model.getModuleJacobianInverse());
}

TEST_F(SwerveModelTestFixture, testStateSettersAndGettersDifferential){
	m_config.module_type = swerve_type_e::DIFFERENTIAL;
	SwerveModel diff_model(m_config);

	for(int i = 0; i < 10; i++){
		std::unordered_map<std::string, Eigen::Vector2d> joint_positions;
		std::unordered_map<std::string, Eigen::Vector2d> joint_velocities;
		std::unordered_map<std::string, Eigen::Vector2d> module_positions;
		std::unordered_map<std::string, Eigen::Vector2d> module_velocities;
		for(const auto& [name, _] : m_config.module_positions){
			// Get random velocity inputs
			joint_positions[name] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));
			joint_velocities[name] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));

			auto pos = joint_positions[name];
			auto vel = joint_velocities[name];
			module_positions[name] = Eigen::Vector2d(
				(pos[0] - pos[1]) * m_config.wheel_ratio / 2.0,
				(pos[0] + pos[1]) / 2 * m_config.steering_ratio);
			module_velocities[name] = Eigen::Vector2d(
				(vel[0] - vel[1]) * m_config.wheel_ratio / 2.0,
				(vel[0] + vel[1]) / 2 * m_config.steering_ratio);
		}

		diff_model.updateRobotStates(joint_positions, joint_velocities);

		for(const auto& [name, _] : m_config.module_positions){
			EXPECT_NEAR(diff_model.getModuleState(name).wheel_position, module_positions.at(name)[0], m_eps);
			EXPECT_NEAR(diff_model.getModuleState(name).steering_position, module_positions.at(name)[1], m_eps);
			EXPECT_NEAR(diff_model.getModuleState(name).wheel_velocity, module_velocities.at(name)[0], m_eps);
			EXPECT_NEAR(diff_model.getModuleState(name).steering_velocity, module_velocities.at(name)[1], m_eps);
		}
	}
}

TEST_F(SwerveModelTestFixture, testStateSettersAndGettersCoaxial){
	m_config.module_type = swerve_type_e::COAXIAL;
	SwerveModel coax_model(m_config);

	for(int i = 0; i < 10; i++){
		std::unordered_map<std::string, Eigen::Vector2d> joint_positions;
		std::unordered_map<std::string, Eigen::Vector2d> joint_velocities;
		std::unordered_map<std::string, Eigen::Vector2d> module_positions;
		std::unordered_map<std::string, Eigen::Vector2d> module_velocities;
		for(const auto& [name, _] : m_config.module_positions){
			// Get random velocity inputs
			joint_positions[name] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));
			joint_velocities[name] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));

			auto pos = joint_positions[name];
			auto vel = joint_velocities[name];
			module_positions[name] = Eigen::Vector2d(
				pos[0] * m_config.wheel_ratio,
				pos[1] * m_config.steering_ratio);

			module_velocities[name] = Eigen::Vector2d(
				vel[0] * m_config.wheel_ratio,
				vel[1] * m_config.steering_ratio);
		}

		coax_model.updateRobotStates(joint_positions, joint_velocities);

		for(const auto& [name, _] : m_config.module_positions){
			EXPECT_NEAR(coax_model.getModuleState(name).wheel_position, module_positions.at(name)[0], m_eps);
			EXPECT_NEAR(coax_model.getModuleState(name).steering_position, module_positions.at(name)[1], m_eps);
			EXPECT_NEAR(coax_model.getModuleState(name).wheel_velocity, module_velocities.at(name)[0], m_eps);
			EXPECT_NEAR(coax_model.getModuleState(name).steering_velocity, module_velocities.at(name)[1], m_eps);
		}
	}
}

TEST_F(SwerveModelTestFixture, testStateSettersAndGettersDifferentialWithSteeringEncoder){
	m_config.module_type = swerve_type_e::DIFFERENTIAL;
	SwerveModel diff_model(m_config);

	for(int i = 0; i < 10; i++){
		std::unordered_map<std::string, Eigen::Vector2d> joint_positions;
		std::unordered_map<std::string, Eigen::Vector2d> joint_velocities;
		std::unordered_map<std::string, double> steering_positions;
		std::unordered_map<std::string, double> steering_velocities;
		std::unordered_map<std::string, Eigen::Vector2d> module_positions;
		std::unordered_map<std::string, Eigen::Vector2d> module_velocities;

		for(const auto& [name, _] : m_config.module_positions){
			// Get random velocity inputs
			joint_positions[name] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));
			joint_velocities[name] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));
			steering_positions[name] = getRandomDouble(600);
			steering_velocities[name] = getRandomDouble(600);

			auto pos = joint_positions[name];
			auto vel = joint_velocities[name];
			module_positions[name] = Eigen::Vector2d(
				(pos[0] - pos[1]) * m_config.wheel_ratio / 2.0,
				steering_positions[name]);
			module_velocities[name] = Eigen::Vector2d(
				(vel[0] - vel[1]) * m_config.wheel_ratio / 2.0,
				steering_velocities[name]);
		}

		diff_model.updateRobotStates(joint_positions, joint_velocities, steering_positions, steering_velocities);

		for(const auto& [name, _] : m_config.module_positions){
			EXPECT_NEAR(diff_model.getModuleState(name).wheel_position, module_positions.at(name)[0], m_eps);
			EXPECT_NEAR(diff_model.getModuleState(name).steering_position, steering_positions.at(name), m_eps);
			EXPECT_NEAR(diff_model.getModuleState(name).wheel_velocity, module_velocities.at(name)[0], m_eps);
			EXPECT_NEAR(diff_model.getModuleState(name).steering_velocity, steering_velocities.at(name), m_eps);
		}
	}
}

TEST_F(SwerveModelTestFixture, testStateSettersAndGettersCoaxialWithSteeringEncoder){
	m_config.module_type = swerve_type_e::COAXIAL;
	SwerveModel diff_model(m_config);

	for(int i = 0; i < 10; i++){
		std::unordered_map<std::string, Eigen::Vector2d> joint_positions;
		std::unordered_map<std::string, Eigen::Vector2d> joint_velocities;
		std::unordered_map<std::string, double> steering_positions;
		std::unordered_map<std::string, double> steering_velocities;
		std::unordered_map<std::string, Eigen::Vector2d> module_positions;
		std::unordered_map<std::string, Eigen::Vector2d> module_velocities;

		for(const auto& [name, _] : m_config.module_positions){
			// Get random velocity inputs
			joint_positions[name] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));
			joint_velocities[name] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));
			steering_positions[name] = getRandomDouble(600);
			steering_velocities[name] = getRandomDouble(600);

			auto pos = joint_positions[name];
			auto vel = joint_velocities[name];
			module_positions[name] = Eigen::Vector2d(
				pos[0] * m_config.wheel_ratio,
				steering_positions[name]);

			module_velocities[name] = Eigen::Vector2d(
				vel[0] * m_config.wheel_ratio,
				steering_velocities[name]);
		}

		diff_model.updateRobotStates(joint_positions, joint_velocities, steering_positions, steering_velocities);

		for(const auto& [name, _] : m_config.module_positions){
			EXPECT_NEAR(diff_model.getModuleState(name).wheel_position, module_positions.at(name)[0], m_eps);
			EXPECT_NEAR(diff_model.getModuleState(name).steering_position, steering_positions.at(name), m_eps);
			EXPECT_NEAR(diff_model.getModuleState(name).wheel_velocity, module_velocities.at(name)[0], m_eps);
			EXPECT_NEAR(diff_model.getModuleState(name).steering_velocity, steering_velocities.at(name), m_eps);
		}
	}
}

TEST_F(SwerveModelTestFixture, testInvalidModuleNameThrows){
	m_config.module_type = swerve_type_e::DIFFERENTIAL;
	SwerveModel diff_model(m_config);
	EXPECT_THROW(diff_model.getModuleState("nonexistent"), std::runtime_error);
}

TEST_F(SwerveModelTestFixture, testUpdateModuleStatesWithMismatchedVelocityMapSize){
	m_config.module_type = swerve_type_e::DIFFERENTIAL;
	SwerveModel diff_model(m_config);

	std::unordered_map<std::string, Eigen::Vector2d> joint_positions;
	joint_positions["front_right"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));
	joint_positions["front_left"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));
	joint_positions["back_right"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));
	joint_positions["back_left"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));

	std::unordered_map<std::string, Eigen::Vector2d> joint_velocities;
	joint_velocities["front_right"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));
	joint_velocities["front_left"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));
	joint_velocities["back_right"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));


	EXPECT_THROW(diff_model.updateRobotStates(joint_positions, joint_velocities), std::runtime_error);
}

TEST_F(SwerveModelTestFixture, testUpdateModuleStatesWithIncorrectNames){
	m_config.module_type = swerve_type_e::DIFFERENTIAL;
	SwerveModel diff_model(m_config);

	std::unordered_map<std::string, Eigen::Vector2d> joint_positions;
	joint_positions["front_right"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));
	joint_positions["front_left"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));
	joint_positions["back_right"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));
	joint_positions["back_left"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));

	std::unordered_map<std::string, Eigen::Vector2d> joint_velocities;
	joint_velocities["front_right"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));
	joint_velocities["front_left"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));
	joint_velocities["back_right"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));
	joint_velocities["wrong_name"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));


	EXPECT_THROW(diff_model.updateRobotStates(joint_positions, joint_velocities), std::runtime_error);
}

TEST_F(SwerveModelTestFixture, testUpdateModuleStatesWithMismatchedVelocityMapSizeWithSteering){
	m_config.module_type = swerve_type_e::DIFFERENTIAL;
	SwerveModel diff_model(m_config);

	std::unordered_map<std::string, Eigen::Vector2d> joint_positions;
	joint_positions["front_right"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));
	joint_positions["front_left"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));
	joint_positions["back_right"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));
	joint_positions["back_left"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));

	std::unordered_map<std::string, Eigen::Vector2d> joint_velocities;
	joint_velocities["front_right"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));
	joint_velocities["front_left"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));
	joint_velocities["back_right"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));
	joint_velocities["back_left"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));

	std::unordered_map<std::string, double> steering_positions;
	steering_positions["front_right"] = getRandomDouble(600);
	steering_positions["front_left"] = getRandomDouble(600);
	steering_positions["back_right"] = getRandomDouble(600);
	steering_positions["back_left"] = getRandomDouble(600);

	std::unordered_map<std::string, double> steering_velocities;
	steering_velocities["front_right"] = getRandomDouble(600);
	steering_velocities["front_left"] = getRandomDouble(600);
	steering_velocities["back_right"] = getRandomDouble(600);


	EXPECT_THROW(diff_model.updateRobotStates(joint_positions, joint_velocities, steering_positions, steering_velocities), std::runtime_error);
}

TEST_F(SwerveModelTestFixture, testUpdateModuleStatesWithIncorrectNamesWithSteering){
	m_config.module_type = swerve_type_e::DIFFERENTIAL;
	SwerveModel diff_model(m_config);

	std::unordered_map<std::string, Eigen::Vector2d> joint_positions;
	joint_positions["front_right"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));
	joint_positions["front_left"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));
	joint_positions["back_right"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));
	joint_positions["back_left"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));

	std::unordered_map<std::string, Eigen::Vector2d> joint_velocities;
	joint_velocities["front_right"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));
	joint_velocities["front_left"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));
	joint_velocities["back_right"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));
	joint_velocities["back_left"] = Eigen::Vector2d(getRandomDouble(600), getRandomDouble(600));

	std::unordered_map<std::string, double> steering_positions;
	steering_positions["front_right"] = getRandomDouble(600);
	steering_positions["front_left"] = getRandomDouble(600);
	steering_positions["back_right"] = getRandomDouble(600);
	steering_positions["back_left"] = getRandomDouble(600);

	std::unordered_map<std::string, double> steering_velocities;
	steering_velocities["front_right"] = getRandomDouble(600);
	steering_velocities["front_left"] = getRandomDouble(600);
	steering_velocities["back_right"] = getRandomDouble(600);
	steering_velocities["wrong_name"] = getRandomDouble(600);

	EXPECT_THROW(diff_model.updateRobotStates(joint_positions, joint_velocities, steering_positions, steering_velocities), std::runtime_error);
}