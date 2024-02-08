#pragma once

#include <cmath>
#include "eigen3/Eigen/Geometry"
#include <ghost_util/unit_conversion_utils.hpp>

namespace ghost_util {

template <typename VECTOR_T>
double angleBetweenVectorsRadians(const VECTOR_T& v1, const VECTOR_T& v2){
	return acos((v1.dot(v2)) / (v1.norm() * v2.norm()));
}

template <typename VECTOR_T>
double angleBetweenVectorsDegrees(const VECTOR_T& v1, const VECTOR_T& v2){
	return angleBetweenVectorsRadians<VECTOR_T>(v1, v2) * RAD_TO_DEG;
}

} // namespace ghost_util