#pragma once

#include <cmath>

namespace ghost_util {

template <typename T>
T clamp(T val, T min, T max){
	return std::max(min, std::min(val, max));
}

// double

}