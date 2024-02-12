#pragma once

#include <vector>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

namespace ghost_planners {

class CubicMotionPlanner {
private:
	static Eigen::MatrixXf computeCubicCoeff(double t0, double tf, std::vector<double> vec_q0, std::vector<double> vec_qf);

public:
	static std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> computeCubicTraj(std::vector<double> vec_q0,
	                                                                                                   std::vector<double> vec_qf, 
																									   double t0, double tf, int n);
	CubicMotionPlanner() = default;
};

} // namespace ghost_planners