#pragma once

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include <vector>


class CubicMotionPlanner {
private:
	Eigen::MatrixXf computeCubicCoeff(double t0, double tf, std::vector<double> vec_q0, std::vector<double> vec_qf);

public:
	std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> computeCubicTraj
		(Eigen::MatrixXf A, double t0, double tf, int n);

	CubicMotionPlanner() = default;
};