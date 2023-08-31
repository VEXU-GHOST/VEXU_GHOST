// Copyright 2019 kvedder@seas.upenn.edu
// School of Engineering and Applied Sciences,
// University of Pennsylvania
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

#ifndef SRC_MATH_STATISTICS_H_
#define SRC_MATH_STATISTICS_H_

#include <algorithm>

#include "math/math_util.h"

namespace statistics {

template <typename T>
T ProbabilityDensityGaussian(const T& sample, const T& mean, const T& stddev) {
  if (stddev == T(0)) {
    return (sample == mean) ? T(1) : T(0);
  }
  return T(1.0) / std::sqrt(T(2) * math_util::Sq(stddev) * M_PI) *
         exp(-math_util::Sq(sample - mean) / (T(2) * math_util::Sq(stddev)));
}

template <typename T>
T ProbabilityDensityExp(const T& sample, const T& lambda) {
  if (sample > T(0)) {
    return lambda * std::exp(-lambda * sample);
  } else {
    return T(0);
  }
}

template <typename T>
T ProbabilityDensityUniform(const T& sample, const T& min, const T& max) {
  if (sample >= min && sample <= max) {
    return T(1.0) / (max - min);
  } else {
    return T(0);
  }
}

template <typename Container, typename Type, typename PercentType>
Type GetPercentile(Container c, const PercentType percentile) {
  std::sort(c.begin(), c.end());
  const size_t idx =
      static_cast<size_t>(static_cast<PercentType>(c.size()) * percentile);
  const Type& val = c[idx];
  return val;
}

}  // namespace statistics

#endif  // SRC_MATH_STATISTICS_H_