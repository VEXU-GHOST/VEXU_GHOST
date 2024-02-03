#pragma once

#include "eigen3/Eigen/Geometry"
#include <ghost_swerve/swerve_model.hpp>
#include <ghost_util/angle_util.hpp>
#include <ghost_util/test_util.hpp>
#include <gtest/gtest.h>

namespace ghost_swerve {

namespace test {

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

		m_config.module_type = swerve_type_e::COAXIAL;
		m_coax_model_ptr = std::make_shared<SwerveModel>(m_config);

		m_config.module_type = swerve_type_e::DIFFERENTIAL;
		m_diff_model_ptr = std::make_shared<SwerveModel>(m_config);

		m_models.push_back(m_coax_model_ptr);
		m_models.push_back(m_diff_model_ptr);
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

	static ModuleState getRandomModuleState(){
		ModuleState s;
		s.wheel_position = ghost_util::getRandomDouble();
		s.wheel_velocity = ghost_util::getRandomDouble();

		s.steering_position = ghost_util::getRandomDouble();
		s.steering_velocity = ghost_util::getRandomDouble();
		return s;
	}
	std::vector<std::shared_ptr<SwerveModel> > m_models;
	std::shared_ptr<SwerveModel> m_coax_model_ptr;
	std::shared_ptr<SwerveModel> m_diff_model_ptr;
	SwerveConfig m_config;
	static constexpr double m_eps = 1e-6;
};

} // namespace test

} // namespace ghost_swerve