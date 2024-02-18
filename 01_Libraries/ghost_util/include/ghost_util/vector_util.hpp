#pragma once

#include <cmath>
#include "eigen3/Eigen/Geometry"
#include <ghost_util/unit_conversion_utils.hpp>

namespace ghost_util {

template <typename VECTOR_T>
double angleBetweenVectorsRadians(const VECTOR_T& v1, const VECTOR_T& v2){
	return std::fabs(acos((v1.dot(v2)) / (v1.norm() * v2.norm())));
}

template <typename VECTOR_T>
double angleBetweenVectorsDegrees(const VECTOR_T& v1, const VECTOR_T& v2){
	return angleBetweenVectorsRadians<VECTOR_T>(v1, v2) * RAD_TO_DEG;
}

Eigen::Matrix3d getRotationMatrixFromEulerAnglesRadians(double roll, double pitch, double yaw){
	Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
	Eigen::Quaternion<double> q =  yawAngle * pitchAngle * rollAngle;
	return q.matrix();
}

Eigen::Matrix3d getRotationMatrixFromEulerAnglesDegrees(double roll, double pitch, double yaw){
	return getRotationMatrixFromEulerAnglesRadians(roll * DEG_TO_RAD, pitch * DEG_TO_RAD, yaw * DEG_TO_RAD);
}

} // namespace ghost_util