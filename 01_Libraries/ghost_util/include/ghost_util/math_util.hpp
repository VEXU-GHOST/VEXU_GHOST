#pragma once

#include <cmath>

namespace ghost_util {

template <typename T>
T clamp(T val, T min, T max){
	return std::max(min, std::min(val, max));
}

double sign(double val){
	double sign = (val >= 0.0) ? 1.0 : -1.0;
	return sign;
}

}