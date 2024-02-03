#include <ghost_swerve/swerve_model_test_fixture.hpp>

using namespace ghost_swerve::test;
using namespace ghost_swerve;
using namespace ghost_util;

TEST_F(SwerveModelTestFixture, testConstructors){
	// Coaxial constructor
	m_config.module_type = swerve_type_e::COAXIAL;
	EXPECT_NO_THROW(auto model = SwerveModel(m_config));

	// Differential constructor
	m_config.module_type = swerve_type_e::DIFFERENTIAL;
	EXPECT_NO_THROW(auto model = SwerveModel(m_config));
}

TEST_F(SwerveModelTestFixture, testDefaultModuleStatesArePopulated){
	// Check each model
	for(const auto& model : m_models){
		// Check each module
		for(const auto& [name, _] : m_config.module_positions){
			EXPECT_NO_THROW(model->getCurrentModuleState(name));
			EXPECT_EQ(model->getCurrentModuleState(name), ModuleState());
		}
	}
}

TEST_F(SwerveModelTestFixture, testMaxBaseVelocities){
	for(const auto& model : m_models){
		EXPECT_EQ(model->getMaxBaseLinearVelocity(), m_config.max_wheel_lin_vel);
		EXPECT_EQ(model->getMaxBaseAngularVelocity(), m_config.max_wheel_lin_vel / (double) Eigen::Vector2d(5.5, 5.5).norm());
	}
}

TEST_F(SwerveModelTestFixture, testModuleJacobians){
	for(const auto& model : m_models){
		checkInverse(model->getModuleJacobian(), model->getModuleJacobianInverse());
		checkInverse(model->getModuleJacobianTranspose(), model->getModuleJacobianInverseTranspose());
		EXPECT_TRUE(model->getModuleJacobian().isApprox(model->getModuleJacobianTranspose().transpose()));
	}
}

TEST_F(SwerveModelTestFixture, testInvalidModuleNameThrows){
	for(const auto& model : m_models){
		EXPECT_THROW(model->setModuleState("nonexistent", getRandomModuleState()), std::runtime_error);
		EXPECT_THROW(model->getCurrentModuleState("nonexistent"), std::runtime_error);
		EXPECT_THROW(model->getPreviousModuleState("nonexistent"), std::runtime_error);
	}
}

TEST_F(SwerveModelTestFixture, testStateGetterSetters){
	for(const auto& model : m_models){
		for(const auto& [name, _] : m_config.module_positions){
			auto state = getRandomModuleState();
			EXPECT_NO_THROW(model->setModuleState(name, state));

			state.steering_position = ghost_util::WrapAngle360(state.steering_position);
			EXPECT_NO_THROW(auto s = model->getCurrentModuleState(name));
			EXPECT_EQ(state, model->getCurrentModuleState(name));
		}
	}
}

TEST_F(SwerveModelTestFixture, testLastModuleStatesAreSaved){
	for(const auto& model : m_models){
		for(const auto& [name, _] : m_config.module_positions){
			auto last_state = getRandomModuleState();
			auto curr_state = getRandomModuleState();
			model->setModuleState(name, last_state);
			model->setModuleState(name, curr_state);

			last_state.steering_position = ghost_util::WrapAngle360(last_state.steering_position);
			curr_state.steering_position = ghost_util::WrapAngle360(curr_state.steering_position);

			EXPECT_EQ(last_state, model->getPreviousModuleState(name));
			EXPECT_EQ(curr_state, model->getCurrentModuleState(name));
		}
	}
}