// credit:	https://medium.com/geekculture/trajectory-generation-in-c-da36521128aa
//  		https://github.com/markusbuchholz/trajectory_generation_in_cpp/blob/main/trajectory_generation.cpp
#include "ghost_planners/cubic_motion_planner.hpp"

using Eigen::MatrixXd;
using Eigen::MatrixXf;

MatrixXf CubicMotionPlanner::computeCubicCoeff(double t0, double tf, std::vector<double> vec_q0, std::vector<double> vec_qf){
	// vec_q0 = {position, velocity}
	// Ax = B
	// A = cubic function matrix
	// x = coefficients
	// B = initial and final values

	MatrixXf A(4, 4);
	MatrixXf B(4, 1);

	A(0, 0) = 1;
	A(0, 1) = t0;
	A(0, 2) = std::pow(t0, 2);
	A(0, 3) = std::pow(t0, 3);

	A(1, 0) = 0;
	A(1, 1) = 1;
	A(1, 2) = 2 * t0;
	A(1, 3) = 3 * std::pow(t0, 2);

	A(2, 0) = 1;
	A(2, 1) = tf;
	A(2, 2) = std::pow(tf, 2);
	A(2, 3) = std::pow(tf, 3);

	A(3, 0) = 0;
	A(3, 1) = 1;
	A(3, 2) = 2 * tf;
	A(3, 3) = 3 * std::pow(tf, 2);

	B(0, 0) = vec_q0[0];
	B(1, 0) = vec_q0[1];
	B(2, 0) = vec_qf[0];
	B(3, 0) = vec_qf[1];

	return A.inverse() * B;
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> CubicMotionPlanner::computeCubicTraj(MatrixXf A, double t0, double tf, int n){
	// A = coefficients
	// n = number of timesteps
	std::vector<double> a = {A(0, 0), A(1, 0), A(2, 0), A(3, 0)};

	std::vector<double> qd;
	std::vector<double> d_qd;
	std::vector<double> dd_qd;
	std::vector<double> time;
	double step = (tf - t0) / n;
	for(double t = t0; t < tf; t += step){
		double qdi = a[0] + a[1] * t + a[2] * std::pow(t, 2) + a[3] * std::pow(t, 3);
		double d_qdi = a[1] + 2 * a[2] * t + 3 * a[3] * std::pow(t, 2);

		qd.push_back(qdi);
		d_qd.push_back(d_qdi);
		time.push_back(t);
	}

	return std::make_tuple(time, qd, d_qd);
}