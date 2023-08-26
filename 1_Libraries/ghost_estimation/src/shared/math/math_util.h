// Copyright 2016 - 2018 kvedder@umass.edu, joydeepb@cs.umass.edu,
//  slane@cs.umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
//
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
// ========================================================================

#include <algorithm>
#include <array>
#include <cmath>
#include "glog/logging.h"

#ifndef SRC_MATH_MATH_UTIL_H_
#define SRC_MATH_MATH_UTIL_H_

#ifndef M_2PI
#define M_2PI (2.0 * M_PI)
#endif  // M_2PI

namespace math_util {

// Clamp value to min and max.
template <typename T>
T Clamp(const T value, const T min_value, const T max_value) {
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

// Convert angle in radians to degrees.
template <typename T>
T RadToDeg(T angle) {
  return (angle / T(M_PI) * 180.0);
}

// Convert angle in degrees to radians.
template <typename T>
constexpr T DegToRad(T angle) {
  return (angle / 180.0 * T(M_PI));
}

template <typename T>
T AngleMod(T angle) {
  angle -= T(M_2PI) * rint(angle / T(M_2PI));
  return angle;
}

template <typename T>
T AngleDiff(const T& a0, const T& a1) {
  const T s = std::sin(a0 - a1);
  const T c = std::cos(a0 - a1);
  return std::atan2(s, c);
}

template <>
inline float AngleDiff<float>(const float& a0, const float& a1) {
  static constexpr float kPiF = static_cast<float>(M_2PI);
  float angle = a0 - a1;
  angle -= kPiF * rintf(angle / kPiF);
  return angle;
}

template <>
inline double AngleDiff<double>(const double& a0, const double& a1) {
  static constexpr double kPiF = static_cast<double>(M_2PI);
  double angle = a0 - a1;
  angle -= kPiF * rint(angle / kPiF);
  return angle;
}


template <typename T>
T AngleDist(const T& a0, const T& a1) {
  return std::abs(AngleDiff(a0, a1));
}


// Check if angle is between min and max angle, moving in a counterclockwise
// direction from min_angle to max_angle.
 template <typename T>
 bool IsAngleBetween(T query,
                     T start_angle,
                     T end_angle,
                     const int rotation_sign) {
  if (rotation_sign == 0) {
    return (query == start_angle && start_angle == end_angle);
  } else if (rotation_sign == 1) {
    if (start_angle < end_angle) {
      // No wrap around.
      return (query <= end_angle) && (query >= start_angle);
    }

    return (query > start_angle) || (query < end_angle);
  } else {
    // rotation_sign == -1
    if (start_angle > end_angle) {
      return (query <= start_angle) && (query >= end_angle);
    }
    return (query > end_angle) || (query < start_angle);
  }
}

template <typename T>
T Sq(const T& x) {
  return (x * x);
}

template <typename T>
T Cube(const T& x) {
  return (x * x * x);
}

template <typename T, unsigned int n>
T Pow(const T& x) {
  T y = static_cast<T>(1);
  for (unsigned int i = 0; i < n; ++i) {
    y *= x;
  }
  return y;
}

template <typename T>
T Pow(const T& x, const unsigned int n) {
  T y = static_cast<T>(1);
  for (unsigned int i = 0; i < n; ++i) {
    y *= x;
  }
  return y;
}

// Take given base to a given power as a constexpr.
template <typename T>
constexpr T ConstexprPow(const T& base, const int exp, const T result = 1) {
  return (exp < 1) ? result
                   : ConstexprPow((base * base), (exp / 2),
                                  ((exp % 2) ? result * base : result));
}

// Return the truncated linearly interpolated value between y_min and y_max
// depending on the provided value of x, x_min, and x_max.
template <typename T>
T Ramp(const T x, const T x_min, const T x_max, const T y_min, const T y_max) {
  if (x <= x_min) {
    return y_min;
  } else if (x >= x_max) {
    return y_max;
  }
  return (y_min + (x - x_min) / (x_max - x_min) * (y_max - y_min));
}

// Solve the quadratic polynomial a * x^2 + b * x + c = 0, and return the
// two real roots as r0 and r1, where r0 is less than or equal to r1. The return
// value is the number of unique real roots found.
template <typename T>
unsigned int SolveQuadratic(const T& a, const T& b, const T& c, T* r0, T* r1) {
  const T discriminant = Sq(b) - T(4.0) * a * c;
  if (discriminant < T(0)) {
    return 0;
  } else if (discriminant == 0) {
    *r0 = *r1 = -b / (T(2.0) * a);
    return 1;
  }
  const T sqrt_discriminant = sqrt(discriminant);
  if (a >= T(0)) {
    *r0 = (-b - sqrt_discriminant) / (2.0 * a);
    *r1 = (-b + sqrt_discriminant) / (2.0 * a);
  } else {
    *r0 = (-b + sqrt_discriminant) / (2.0 * a);
    *r1 = (-b - sqrt_discriminant) / (2.0 * a);
  }
  return 2;
}

// Bound the value to the minimum and maximum specified.
template <typename T>
void Bound(const T& min_val, const T& max_val, T* x) {
  if ((*x) > max_val) *x = max_val;
  if ((*x) < min_val) *x = min_val;
}

// Bound the value to lie within +- limit.
template <typename T>
void AbsBound(const T& limit, T* x) {
  if ((*x) > limit || (*x) < limit) {
    *x = limit;
  }
}

// returns the +/- sign of a numerical value, 0 if value is 0
template <typename T>
int Sign(T val) {
  return (T(0) < val) - (val < T(0));
}

// Solve the cubic polynomial a * x^3 + b * x^2 + c * x + d = 0, and return the
// real roots as r0, r1, and r2 where r0 is the smallest real root, and r2 is
// the largest real root. The return value is the number of unique real roots
// found.
template <typename T>
unsigned int SolveCubic(const T& a, const T& b, const T& c, const T& d, T* r0,
                        T* r1, T* r2) {
  DCHECK_NE(r0, static_cast<T*>(nullptr));
  DCHECK_NE(r1, static_cast<T*>(nullptr));
  DCHECK_NE(r2, static_cast<T*>(nullptr));
  const T discriminant = T(18) * a * b * c * d - T(4) * Cube<T>(b) * d +
                         Sq<T>(b) * Sq<T>(c) - T(4) * a * Cube<T>(c) -
                         T(27) * Sq<T>(a) * Sq<T>(d);

  const T discriminant0 = Sq<T>(b) - T(3) * a * c;

  const T discriminant1 =
      T(2) * Cube<T>(b) - T(9) * a * b * c + T(27) * Sq<T>(a) * d;

  if (discriminant > T(0)) {
    // todo(slane): this case is not working
    // Three distinct real roots
    const T p = (T(3) * a * c - Sq<T>(b)) / (3 * Sq<T>(a));
    const T q = (T(2) * Cube<T>(b) - 9 * a * b * c + 27 * Sq<T>(a) * d) /
                (T(27) * Cube<T>(a));

    std::array<T, 3> roots;

    for (T k = 0; k < 3; k++) {
      roots[k] = (T(2) * sqrt(-p / T(3)) *
                      cos(T(1.0 / 3.0) *
                              acos(T(3) * q / (T(2) * p) * sqrt(T(-3) / p)) -
                          T(2.0) * k * M_PI / T(3.0)) -
                  b / (T(3) * a));
    }

    *r0 = roots[2];
    *r1 = roots[1];
    *r2 = roots[0];

    return 3;
  } else if (discriminant == T(0)) {
    if (discriminant0 == 0) {
      *r2 = -b / (T(3) * a);
      return 1;
    } else {
      const T double_root = (T(9) * a * d - b * c) / (T(2) * discriminant0);

      const T simple_root = (T(4) * a * b * c - 9 * Sq<T>(a) * d - Cube<T>(b)) /
                            (a * discriminant0);

      if (double_root > simple_root) {
        *r2 = double_root;
        *r1 = simple_root;
      } else {
        *r2 = simple_root;
        *r1 = double_root;
      }
      return 2;
    }
  } else {
    // One real root and two non-real complex conjugate roots.
    T C = cbrt(((discriminant1 + sqrt(T(-27) * Sq<T>(a) * discriminant)) / 2));

    if (C == 0) {
      C = cbrt(((discriminant1 - sqrt(T(-27) * Sq<T>(a) * discriminant)) / 2));
    }

    *r2 = -T(1) / (T(3) * a) * (b + C + discriminant0 / C);

    return 1;
  }
}

}  // namespace math_util

#endif  // SRC_MATH_MATH_UTIL_H_
