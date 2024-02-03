#include <ghost_swerve/swerve_model_test_fixture.hpp>

using namespace ghost_swerve::test;
using namespace ghost_swerve;
using namespace ghost_util;


TEST_F(SwerveModelTestFixture, testICRZeroCase){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 0.0, 45, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, 0.0, -45, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, 0.0, -45, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, 0.0, 45, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_FALSE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE(icr_point.isApprox(Eigen::Vector2d(0.0, 0.0)));
}

TEST_F(SwerveModelTestFixture, testICRAtBackLeft){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 0.0, 45, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, 0.0, 90.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, 0.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, 0.0, getRandomDouble(), 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_FALSE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE(icr_point.isApprox(m_diff_model_ptr->getConfig().module_positions.at("back_left")));
}

TEST_F(SwerveModelTestFixture, testICRInfinity){
	m_diff_model_ptr->setModuleState("front_right", ModuleState());
	m_diff_model_ptr->setModuleState("front_left", ModuleState());
	m_diff_model_ptr->setModuleState("back_right", ModuleState());
	m_diff_model_ptr->setModuleState("back_left", ModuleState());

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_TRUE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE(icr_point.isApprox(Eigen::Vector2d(0.0, 1.0)));
}