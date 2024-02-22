#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace ghost_util {

template <typename T>
T clamp(T val, T min, T max){
	return std::max(min, std::min(val, max));
}

double sign(double val);

bool isPositive(double val);

double linearInterpolate(const std::vector<double> &x_data,
                         const std::vector<double> &y_data,
                         const double desired_x);

double clampedLinearInterpolate(const std::vector<double> &x_data,
                                const std::vector<double> &y_data,
                                const double desired_x);

}