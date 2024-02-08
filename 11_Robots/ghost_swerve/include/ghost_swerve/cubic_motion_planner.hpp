#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#inlcude <vector>


class CubicMotionPlanner {
private:
	Eigen::Matrix<double, 4, 4> m_xValues;
	Eigen::Matrix<double, 4, 4> m_yValues;
	std::vector<double> m_xCoeffs;
	std::vector<double> m_xCoeffs;
public:

	void CubicMotionPlanner(Eigen::Matrix<double> xValues, Eigen::Matrix<double> yValues);
	void CubicMotionPlanner();
	std::vector<double> find_coefficients(Eigen::Matrix<double> xValues, Eigen::Matrix<double> yValues);
	std::vector<double> find_coefficients();
}