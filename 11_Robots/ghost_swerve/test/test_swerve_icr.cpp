#include <ghost_swerve/swerve_model_test_fixture.hpp>
#include "matplotlibcpp.h"


using namespace ghost_swerve::test;
using namespace ghost_swerve;
using namespace ghost_util;

using geometry::Line2d;

TEST_F(SwerveModelTestFixture, testICRZeroCase){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 45, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, -45, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, -45, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, 45, 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_FALSE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE((icr_point - Eigen::Vector2d(0.0, 0.0)).norm() < 1e-10);
	EXPECT_TRUE(m_diff_model_ptr->getICRSSE() < 1e-10);
}

TEST_F(SwerveModelTestFixture, testICRZeroCaseRandomDirections){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 45.0 + 180.0 * getRandomInt(10), 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, -45.0 + 180.0 * getRandomInt(10), 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, -45.0 + 180.0 * getRandomInt(10), 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, 45.0 + 180.0 * getRandomInt(10), 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_FALSE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE((icr_point - Eigen::Vector2d(0.0, 0.0)).norm() < 1e-10);
	EXPECT_TRUE(m_diff_model_ptr->getICRSSE() < 1e-10);
}

TEST_F(SwerveModelTestFixture, testICRCenterLeft){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 26.5650511771, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, -90, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, -26.5650511771, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, 90, 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_FALSE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE((icr_point - Eigen::Vector2d(0, 5.5)).norm() < 1e-10);
	EXPECT_TRUE(m_diff_model_ptr->getICRSSE() < 1e-10);
}

TEST_F(SwerveModelTestFixture, testICRCenterRight){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 90, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, -26.5650511771, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, 90, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, 26.5650511771, 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_FALSE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE((icr_point - Eigen::Vector2d(0, -5.5)).norm() < 1e-10);
	EXPECT_TRUE(m_diff_model_ptr->getICRSSE() < 1e-10);
}

TEST_F(SwerveModelTestFixture, testICRRearCenter){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 90 - 26.5650511771, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, -(90 - 26.5650511771), 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, 0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, 0, 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_FALSE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE((icr_point - Eigen::Vector2d(-5.5, 0.0)).norm() < 1e-10);
	EXPECT_TRUE(m_diff_model_ptr->getICRSSE() < 1e-10);
}

TEST_F(SwerveModelTestFixture, testICRFrontCenter){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 0.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, 0.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, -(90 - 26.5650511771), 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, 90 - 26.5650511771, 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_FALSE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE((icr_point - Eigen::Vector2d(5.5, 0.0)).norm() < 1e-10);
	EXPECT_TRUE(m_diff_model_ptr->getICRSSE() < 1e-10);
}

TEST_F(SwerveModelTestFixture, testICRAtBackLeft){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 45.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, 90.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, 0.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, getRandomDouble(), 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_FALSE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE((icr_point - m_diff_model_ptr->getConfig().module_positions.at("back_left")).norm() < 1e-10);
	EXPECT_TRUE(m_diff_model_ptr->getICRSSE() < 1e-10);
}

TEST_F(SwerveModelTestFixture, testICRAtBackRight){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 90.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, -45.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, getRandomDouble(), 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, 0.0, 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_FALSE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE((icr_point - m_diff_model_ptr->getConfig().module_positions.at("back_right")).norm() < 1e-10);
	EXPECT_TRUE(m_diff_model_ptr->getICRSSE() < 1e-10);
}

TEST_F(SwerveModelTestFixture, testICRAtFrontRight){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, getRandomDouble(), 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, 0.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, 90.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, 45.0, 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_FALSE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE((icr_point - m_diff_model_ptr->getConfig().module_positions.at("front_right")).norm() < 1e-10);
	EXPECT_TRUE(m_diff_model_ptr->getICRSSE() < 1e-10);
}

TEST_F(SwerveModelTestFixture, testICRAtFrontLeft){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 0.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, getRandomDouble(), 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, -45.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, 90.0, 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_FALSE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE((icr_point - m_diff_model_ptr->getConfig().module_positions.at("front_left")).norm() < 1e-10);
	EXPECT_TRUE(m_diff_model_ptr->getICRSSE() < 1e-10);
}

TEST_F(SwerveModelTestFixture, testICRInfinityLeft){
	m_diff_model_ptr->setModuleState("front_right", ModuleState());
	m_diff_model_ptr->setModuleState("front_left", ModuleState());
	m_diff_model_ptr->setModuleState("back_right", ModuleState());
	m_diff_model_ptr->setModuleState("back_left", ModuleState());

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_TRUE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE((icr_point - Eigen::Vector2d(0.0, 1.0)).norm() < 1e-10);
	EXPECT_TRUE(m_diff_model_ptr->getICRSSE() < 1e-10);
}

TEST_F(SwerveModelTestFixture, testICRInfinityForward){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, -90.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, -90.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, -90.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, -90.0, 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_TRUE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE((icr_point - Eigen::Vector2d(1.0, 0.0)).norm() < 1e-10);
	EXPECT_TRUE(m_diff_model_ptr->getICRSSE() < 1e-10);
}

TEST_F(SwerveModelTestFixture, testICRInfinityBack){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, -270.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, -270.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, -270.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, -270.0, 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_TRUE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE((icr_point - Eigen::Vector2d(-1.0, 0.0).normalized()).norm() < 1e-10);
	EXPECT_TRUE(m_diff_model_ptr->getICRSSE() < 1e-10);
}

TEST_F(SwerveModelTestFixture, testICRInfinityRight){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 180.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, 180.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, 180.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, 180.0, 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_TRUE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE((icr_point - Eigen::Vector2d(0.0, -1.0).normalized()).norm() < 1e-10);
	EXPECT_TRUE(m_diff_model_ptr->getICRSSE() < 1e-10);
}

TEST_F(SwerveModelTestFixture, testICRInfinityForwardRight){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, -135.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, -135.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, -135.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, -135.0, 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_TRUE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE((icr_point - Eigen::Vector2d(1.0, -1.0).normalized()).norm() < 1e-10);
	EXPECT_TRUE(m_diff_model_ptr->getICRSSE() < 1e-10);
}

TEST_F(SwerveModelTestFixture, testICRInfinityBackRight){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 135.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, 135.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, 135.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, 135.0, 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_TRUE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE((icr_point - Eigen::Vector2d(-1.0, -1.0).normalized()).norm() < 1e-10);
	EXPECT_TRUE(m_diff_model_ptr->getICRSSE() < 1e-10);
}

TEST_F(SwerveModelTestFixture, testICRInfinityBackLeft){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 45.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, 45.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, 45.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, 45.0, 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_TRUE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE((icr_point - Eigen::Vector2d(-1.0, 1.0).normalized()).norm() < 1e-10);
	EXPECT_TRUE(m_diff_model_ptr->getICRSSE() < 1e-10);
}

TEST_F(SwerveModelTestFixture, testICRInfinityForwardLeft){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, -45.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, -45.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, -45.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, -45.0, 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_TRUE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE((icr_point - Eigen::Vector2d(1.0, 1.0).normalized()).norm() < 1e-10);
	EXPECT_TRUE(m_diff_model_ptr->getICRSSE() < 1e-10);
}

TEST_F(SwerveModelTestFixture, testICRInfinityNoisy){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 90.0 + getRandomDouble(0.05), 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, 90.0 + getRandomDouble(0.05), 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, 90.0 + getRandomDouble(0.05), 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, 90.0 + getRandomDouble(0.05), 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_TRUE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE((icr_point - Eigen::Vector2d(-1.0, 0.0).normalized()).norm());
}

// TEST_F(SwerveModelTestFixture, testICRInfinityCollinearEvaluatesToInsideRobot){
// 	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 1.0, 0.0, 0.0));
// 	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, -1.0, 0.0, 0.0));
// 	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, 0.0, 0.0, 0.0));
// 	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, 0.0, 0.0, 0.0));

// 	m_diff_model_ptr->updateSwerveModel();

// 	Eigen::Vector2d icr_point;
// 	std::cout << icr_point << std::endl;
// 	EXPECT_TRUE((icr_point - Eigen::Vector2d(1.0, 0.0).normalized()).norm());
// 	EXPECT_NEAR(m_diff_model_ptr->getICRSSE(), 0.0, 1e-3);
// }

TEST_F(SwerveModelTestFixture, generatePlots){
	const bool GENERATE_PLOTS = false;
	if(GENERATE_PLOTS){
		namespace plt = matplotlibcpp;
		int N = 1000;
		std::vector<double> noise;
		std::vector<double> sse;

		noise.reserve(N);
		sse.reserve(N);

		for(int i = 0; i < N; i++){
			auto noise_max = 20.0 * ((float) i / (float) N);
			m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 90.0 + getRandomDouble(noise_max), 0.0, 0.0));
			m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, 90.0 + getRandomDouble(noise_max), 0.0, 0.0));
			m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, 90.0 + getRandomDouble(noise_max), 0.0, 0.0));
			m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, 90.0 + getRandomDouble(noise_max), 0.0, 0.0));

			// m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 90.0 + getRandomDouble(noise_max), 0.0, 0.0));
			// m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, -45.0 + getRandomDouble(noise_max), 0.0, 0.0));
			// m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, 0.0 + getRandomDouble(noise_max), 0.0, 0.0));
			// m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, 0.0 + getRandomDouble(noise_max), 0.0, 0.0));

			// m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 4.873896422204 + getRandomDouble(noise_max), 0.0, 0.0));
			// m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, 4.799836113904 + getRandomDouble(noise_max), 0.0, 0.0));
			// m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, -4.873896422204 + getRandomDouble(noise_max), 0.0, 0.0));
			// m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, -4.799836113904 + getRandomDouble(noise_max), 0.0, 0.0));

			// m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 90 - 26.5650511771 + getRandomDouble(noise_max), 0.0, 0.0));
			// m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, -(90 - 26.5650511771) + getRandomDouble(noise_max), 0.0, 0.0));
			// m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, 0 + getRandomDouble(noise_max), 0.0, 0.0));
			// m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, 0 + getRandomDouble(noise_max), 0.0, 0.0));

			m_diff_model_ptr->updateSwerveModel();

			noise.push_back(noise_max);
			sse.push_back(m_diff_model_ptr->getICRSSE());
		}

		plt::figure();
		plt::scatter(noise, sse);
		plt::show();
	}
}