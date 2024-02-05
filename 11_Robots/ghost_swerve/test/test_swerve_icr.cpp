#include <ghost_swerve/swerve_model_test_fixture.hpp>

using namespace ghost_swerve::test;
using namespace ghost_swerve;
using namespace ghost_util;

using geometry::Line2d;

TEST_F(SwerveModelTestFixture, testCollinearLines){
	std::vector<Line2d> lines;
	lines.push_back(Line2d(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, getRandomDouble())));
	lines.push_back(Line2d(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, getRandomDouble())));
	auto intersection_points = m_diff_model_ptr->calculateSphericalProjectionAxisIntersections(lines);

	EXPECT_EQ(intersection_points.size(), 1);
	EXPECT_TRUE(intersection_points[0].isApprox(Eigen::Vector3d(0, 1, 0.0).normalized()));
}

TEST_F(SwerveModelTestFixture, throwsOnLessThanTwoLines){
	auto line = Line2d(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 0.0));
	EXPECT_THROW(m_diff_model_ptr->calculateSphericalProjectionAxisIntersections(std::vector<Line2d>{}), std::runtime_error);
	EXPECT_THROW(m_diff_model_ptr->calculateSphericalProjectionAxisIntersections(std::vector<Line2d>{line}), std::runtime_error);
}

TEST_F(SwerveModelTestFixture, testParallelLines){
	std::vector<Line2d> lines;
	auto x = getRandomDouble();
	auto y = getRandomDouble();
	lines.push_back(Line2d(Eigen::Vector2d(x, 0.0), Eigen::Vector2d(x, y)));
	lines.push_back(Line2d(Eigen::Vector2d(-x, 0.0), Eigen::Vector2d(-x, y)));
	auto intersection_points = m_diff_model_ptr->calculateSphericalProjectionAxisIntersections(lines);

	EXPECT_EQ(intersection_points.size(), 1);
	EXPECT_TRUE(intersection_points[0].isApprox(Eigen::Vector3d(0, 1, 0.0).normalized()));
}

TEST_F(SwerveModelTestFixture, testTwoAxesIntersections){
	std::vector<Line2d> lines;
	lines.push_back(Line2d(Eigen::Vector2d(3.0, 3.0), Eigen::Vector2d(3.0, -3.0)));
	lines.push_back(Line2d(Eigen::Vector2d(0.0, -3.0), Eigen::Vector2d(3.0, -3.0)));
	auto intersection_points = m_diff_model_ptr->calculateSphericalProjectionAxisIntersections(lines);
	auto expected = Eigen::Vector3d(3.0, -3.0, 1.0) / Eigen::Vector3d(3.0, -3.0, 1.0).norm();

	EXPECT_EQ(intersection_points.size(), 1);
	EXPECT_TRUE(intersection_points[0].isApprox(expected));
}

TEST_F(SwerveModelTestFixture, testThreeAxesIntersections){
	std::vector<Line2d> lines;
	lines.push_back(Line2d(Eigen::Vector2d(5.0, 0.0), Eigen::Vector2d(-5.0, 10.0)));
	lines.push_back(Line2d(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 5.0)));
	lines.push_back(Line2d(Eigen::Vector2d(-5.0, 0.0), Eigen::Vector2d(-5.0, 10.0)));

	auto intersection_points = m_diff_model_ptr->calculateSphericalProjectionAxisIntersections(lines);
	EXPECT_EQ(intersection_points.size(), 3);
	listContainsEigenVector<Eigen::Vector3d>(intersection_points, Eigen::Vector3d(0.0, 5.0, 1.0).normalized());
	listContainsEigenVector<Eigen::Vector3d>(intersection_points, Eigen::Vector3d(-5.0, 10.0, 1.0).normalized());
	listContainsEigenVector<Eigen::Vector3d>(intersection_points, Eigen::Vector3d(0.0, 1.0, 0.0).normalized());
}

TEST_F(SwerveModelTestFixture, testFourAxesIntersections){
	std::vector<Line2d> lines;
	lines.push_back(Line2d(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 1.0)));
	lines.push_back(Line2d(Eigen::Vector2d(1.0, 0.0), Eigen::Vector2d(0.0, 1.0)));
	lines.push_back(Line2d(Eigen::Vector2d(2.0, 0.0), Eigen::Vector2d(0.0, 1.0)));
	lines.push_back(Line2d(Eigen::Vector2d(3.0, 0.0), Eigen::Vector2d(0.0, 1.0)));

	auto intersection_points = m_diff_model_ptr->calculateSphericalProjectionAxisIntersections(lines);
	EXPECT_EQ(intersection_points.size(), 6);
	for(const auto p : intersection_points){
		EXPECT_TRUE(p.isApprox(Eigen::Vector3d(0.0, 1.0, 1.0).normalized()));
	}
}

TEST_F(SwerveModelTestFixture, testClusterAntipolesThrowsOnEmpty){
	Eigen::Vector3d output;
	EXPECT_THROW(m_diff_model_ptr->averageVectorAntipoles(std::vector<Eigen::Vector3d>(), output), std::runtime_error);
}
TEST_F(SwerveModelTestFixture, testClusterAntipolesSingle){
	std::vector<Eigen::Vector3d> antipole_vectors{
		Eigen::Vector3d(getRandomDouble(), getRandomDouble(), getRandomDouble())
	};

	Eigen::Vector3d avg_point;
	EXPECT_EQ(m_diff_model_ptr->averageVectorAntipoles(antipole_vectors, avg_point), 0.0);
	EXPECT_TRUE(avg_point.isApprox(antipole_vectors[0].normalized()));
	EXPECT_TRUE(avg_point.isApprox(avg_point.normalized()));
}

TEST_F(SwerveModelTestFixture, testClusterAntipolesXAxis){
	std::vector<Eigen::Vector3d> antipole_vectors{
		Eigen::Vector3d(1.0, 0.0, 0.0),
		Eigen::Vector3d(1.0, 0.5, 0.5),
		Eigen::Vector3d(1.0, -0.5, 0.5),
		Eigen::Vector3d(1.0, 0.5, -0.5),
		Eigen::Vector3d(1.0, -0.5, -0.5)
	};

	Eigen::Vector3d avg_point;
	EXPECT_TRUE(m_diff_model_ptr->averageVectorAntipoles(antipole_vectors, avg_point) > 0.0);
	EXPECT_TRUE(avg_point.isApprox(Eigen::Vector3d(1.0, 0.0, 0.0).normalized()));
	EXPECT_TRUE(avg_point.isApprox(avg_point.normalized()));
}

TEST_F(SwerveModelTestFixture, testClusterAntipolesYAxis){
	std::vector<Eigen::Vector3d> antipole_vectors{
		Eigen::Vector3d(0.0, 1.0, 0.0),
		Eigen::Vector3d(0.5, 1.0, 0.5),
		Eigen::Vector3d(-0.5, 1.0, 0.5),
		Eigen::Vector3d(0.5, 1.0, -0.5),
		Eigen::Vector3d(-0.5, 1.0, -0.5)
	};

	Eigen::Vector3d avg_point;
	EXPECT_TRUE(m_diff_model_ptr->averageVectorAntipoles(antipole_vectors, avg_point) > 0.0);
	EXPECT_TRUE(avg_point.isApprox(Eigen::Vector3d(0.0, 1.0, 0.0).normalized()));
	EXPECT_TRUE(avg_point.isApprox(avg_point.normalized()));
}

TEST_F(SwerveModelTestFixture, testClusterAntipolesZAxis){
	std::vector<Eigen::Vector3d> antipole_vectors{
		Eigen::Vector3d(0.0, 0.0, 1.0),
		Eigen::Vector3d(0.5, 0.5, 1.0),
		Eigen::Vector3d(0.5, -0.5, 1.0),
		Eigen::Vector3d(-0.5, 0.5, 1.0),
		Eigen::Vector3d(-0.5, -0.5, 1.0)
	};
	Eigen::Vector3d avg_point;
	EXPECT_TRUE(m_diff_model_ptr->averageVectorAntipoles(antipole_vectors, avg_point) > 0.0);
	EXPECT_TRUE(avg_point.isApprox(Eigen::Vector3d(0.0, 0.0, 1.0).normalized()));
	EXPECT_TRUE(avg_point.isApprox(avg_point.normalized()));
}

TEST_F(SwerveModelTestFixture, testClusterAntipolesRandomInverted){
	std::vector<Eigen::Vector3d> antipole_vectors{
		Eigen::Vector3d(1.0, 0.0, 0.0),
		-Eigen::Vector3d(1.0, 0.5, 0.5),
		-Eigen::Vector3d(1.0, -0.5, 0.5),
		-Eigen::Vector3d(1.0, 0.5, -0.5),
		Eigen::Vector3d(1.0, -0.5, -0.5)
	};
	Eigen::Vector3d avg_point;
	EXPECT_TRUE(m_diff_model_ptr->averageVectorAntipoles(antipole_vectors, avg_point) > 0.0);
	EXPECT_TRUE(avg_point.isApprox(Eigen::Vector3d(1.0, 0.0, 0.0).normalized()));
	EXPECT_TRUE(avg_point.isApprox(avg_point.normalized()));
}

TEST_F(SwerveModelTestFixture, testICRZeroCase){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 45, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, -45, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, -45, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, 45, 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_FALSE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE(icr_point.isApprox(Eigen::Vector2d(0.0, 0.0)));
}

TEST_F(SwerveModelTestFixture, testICRZeroCaseRandomDirections){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 45.0 + 180.0 * getRandomInt(10), 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, -45.0 + 180.0 * getRandomInt(10), 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, -45.0 + 180.0 * getRandomInt(10), 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, 45.0 + 180.0 * getRandomInt(10), 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_FALSE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE(icr_point.isApprox(Eigen::Vector2d(0.0, 0.0)));
}

TEST_F(SwerveModelTestFixture, testICRCenterLeft){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 26.5650511771, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, -90, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, -26.5650511771, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, 90, 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_FALSE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE((icr_point - Eigen::Vector2d(0, 5.5)).norm() < 1e-5);
}

TEST_F(SwerveModelTestFixture, testICRCenterRight){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 90, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, -26.5650511771, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, 90, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, 26.5650511771, 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_FALSE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE((icr_point - Eigen::Vector2d(0, -5.5)).norm() < 1e-5);
}

TEST_F(SwerveModelTestFixture, testICRRearCenter){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 90 - 26.5650511771, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, -(90 - 26.5650511771), 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, 0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, 0, 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_FALSE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE((icr_point - Eigen::Vector2d(-5.5, 0.0)).norm() < 1e-5);
}

TEST_F(SwerveModelTestFixture, testICRFrontCenter){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 0.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, 0.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, -(90 - 26.5650511771), 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, 90 - 26.5650511771, 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_FALSE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE((icr_point - Eigen::Vector2d(5.5, 0.0)).norm() < 1e-5);
}

TEST_F(SwerveModelTestFixture, testICRAtBackLeft){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 45.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, 90.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, 0.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, getRandomDouble(), 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_FALSE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE(icr_point.isApprox(m_diff_model_ptr->getConfig().module_positions.at("back_left")));
}

TEST_F(SwerveModelTestFixture, testICRAtBackRight){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 90.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, -45.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, getRandomDouble(), 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, 0.0, 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_FALSE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE(icr_point.isApprox(m_diff_model_ptr->getConfig().module_positions.at("back_right")));
}

TEST_F(SwerveModelTestFixture, testICRAtFrontRight){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, getRandomDouble(), 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, 0.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, 90.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, 45.0, 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_FALSE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE(icr_point.isApprox(m_diff_model_ptr->getConfig().module_positions.at("front_right")));
}

TEST_F(SwerveModelTestFixture, testICRAtFrontLeft){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 0.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, getRandomDouble(), 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, -45.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, 90.0, 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_FALSE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE(icr_point.isApprox(m_diff_model_ptr->getConfig().module_positions.at("front_left")));
}

TEST_F(SwerveModelTestFixture, testICRInfinityLeft){
	m_diff_model_ptr->setModuleState("front_right", ModuleState());
	m_diff_model_ptr->setModuleState("front_left", ModuleState());
	m_diff_model_ptr->setModuleState("back_right", ModuleState());
	m_diff_model_ptr->setModuleState("back_left", ModuleState());

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_TRUE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE(icr_point.isApprox(Eigen::Vector2d(0.0, 1.0)));
}

TEST_F(SwerveModelTestFixture, testICRInfinityForward){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, -90.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, -90.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, -90.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, -90.0, 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_TRUE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE(icr_point.isApprox(Eigen::Vector2d(1.0, 0.0)));
}

TEST_F(SwerveModelTestFixture, testICRInfinityForwardRight){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, -135.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, -135.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, -135.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, -135.0, 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_TRUE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE(icr_point.isApprox(Eigen::Vector2d(1.0, -1.0).normalized()));
}

TEST_F(SwerveModelTestFixture, testICRInfinityDownRight){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 135.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, 135.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, 135.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, 135.0, 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_TRUE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE(icr_point.isApprox(Eigen::Vector2d(-1.0, -1.0).normalized()));
}

TEST_F(SwerveModelTestFixture, testICRInfinityDown){
	m_diff_model_ptr->setModuleState("front_right", ModuleState(0.0, 90.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("front_left", ModuleState(0.0, 90.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_right", ModuleState(0.0, 90.0, 0.0, 0.0));
	m_diff_model_ptr->setModuleState("back_left", ModuleState(0.0, 90.0, 0.0, 0.0));

	m_diff_model_ptr->updateSwerveModel();

	Eigen::Vector2d icr_point;
	EXPECT_TRUE(m_diff_model_ptr->getICR(icr_point));
	EXPECT_TRUE(icr_point.isApprox(Eigen::Vector2d(-1.0, 0.0).normalized()));
}