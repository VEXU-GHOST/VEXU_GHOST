/*
 *   Copyright (c) 2024 Maxx Wilson
 *   All rights reserved.

 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

#pragma once

#include <cmath>
#include "eigen3/Eigen/Geometry"
#include <ghost_util/unit_conversion_utils.hpp>

namespace ghost_util
{

template<typename VECTOR_T>
double angleBetweenVectorsRadians(const VECTOR_T & v1, const VECTOR_T & v2)
{
  return std::fabs(acos((v1.dot(v2)) / (v1.norm() * v2.norm())));
}

template<typename VECTOR_T>
double angleBetweenVectorsDegrees(const VECTOR_T & v1, const VECTOR_T & v2)
{
  return angleBetweenVectorsRadians<VECTOR_T>(v1, v2) * RAD_TO_DEG;
}

Eigen::Matrix3d getRotationMatrixFromEulerAnglesRadians(double roll, double pitch, double yaw)
{
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;
  return q.matrix();
}

Eigen::Matrix3d getRotationMatrixFromEulerAnglesDegrees(double roll, double pitch, double yaw)
{
  return getRotationMatrixFromEulerAnglesRadians(
    roll * DEG_TO_RAD, pitch * DEG_TO_RAD,
    yaw * DEG_TO_RAD);
}

} // namespace ghost_util
