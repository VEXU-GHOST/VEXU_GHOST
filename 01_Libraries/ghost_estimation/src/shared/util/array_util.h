// Copyright 2017 - 2018 kvedder@umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
// Copyright 2020 kvedder@seas.upenn.edu
// School of Engineering and Applied Sciences,
// University of Pennsylvania
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
#include <limits>
#include <vector>

#ifndef SRC_UTIL_ARRAY_UTIL_H_
#define SRC_UTIL_ARRAY_UTIL_H_

namespace array_util {

template <size_t N, class T>
std::array<T, N> MakeArray(const T& v) {
  std::array<T, N> ret;
  ret.fill(v);
  return ret;
}

template <size_t N, class T>
size_t ArgMin(const std::array<T, N>& v) {
  if (N <= 0) {
    return 0;
  }
  size_t idx = 0;
  for (size_t i = 1; i < v.size(); ++i) {
    if (v[idx] > v[i]) {
      idx = i;
    }
  }
  return idx;
}

template <size_t N, class T>
size_t ArgMax(const std::array<T, N>& v) {
  if (N <= 0) {
    return 0;
  }
  size_t idx = 0;
  for (size_t i = 1; i < v.size(); ++i) {
    if (v[idx] < v[i]) {
      idx = i;
    }
  }
  return idx;
}

template <size_t N, class T>
T SumArray(const std::array<T, N>& v) {
  T acc = 0;
  for (const auto& e : v) {
    acc += e;
  }
  return acc;
}

template <size_t N, class T>
T SelectiveSumArray(const std::array<T, N>& v, const std::array<bool, N>& b,
                    const T zero = 0) {
  T acc = zero;
  for (size_t i = 0; i < N; ++i) {
    const auto& e = v[i];
    if (b[i]) {
      acc += e;
    }
  }
  return acc;
}

template <size_t N, class T>
bool SelectiveEqual(const std::array<bool, N>& b, const std::array<T, N>& v1,
                    const std::array<T, N>& v2) {
  for (size_t i = 0; i < N; ++i) {
    if (b[i] && v1[i] != v2[i]) {
      return false;
    }
  }
  return true;
}

template <size_t N, class T>
T MinElement(const std::array<T, N>& v) {
  T acc = std::numeric_limits<T>::max();
  for (const auto& e : v) {
    acc = std::min(acc, e);
  }
  return acc;
}

template <size_t N, class T>
T SelectiveMinElement(const std::array<T, N>& v, const std::array<bool, N>& b) {
  T acc = std::numeric_limits<T>::max();
  for (size_t i = 0; i < N; ++i) {
    if (!b[i]) {
      continue;
    }
    const auto& e = v[i];
    acc = std::min(acc, e);
  }
  return acc;
}

template <size_t N, class T>
T MaxElement(const std::array<T, N>& v) {
  T acc = std::numeric_limits<T>::min();
  for (const auto& e : v) {
    acc = std::max(acc, e);
  }
  return acc;
}

template <size_t N, class T>
T SelectiveMaxElement(const std::array<T, N>& v, const std::array<bool, N>& b) {
  T acc = std::numeric_limits<T>::min();
  for (size_t i = 0; i < N; ++i) {
    if (!b[i]) {
      continue;
    }
    const auto& e = v[i];
    acc = std::max(acc, e);
  }
  return acc;
}

template <size_t N, class T>
std::array<T, N> AddToEachElement(const std::array<T, N>& a1, const T& val) {
  std::array<T, N> ret;
  for (size_t i = 0; i < N; ++i) {
    ret[i] = a1[i] + val;
  }
  return ret;
}

template <size_t N, class T>
std::array<T, N> AddArrayElements(const std::array<T, N>& a1,
                                  const std::array<T, N>& a2) {
  std::array<T, N> ret;
  for (size_t i = 0; i < N; ++i) {
    ret[i] = a1[i] + a2[i];
  }
  return ret;
}

template <size_t N, class T>
std::array<T, N> SubtractArrayElements(const std::array<T, N>& a1,
                                       const std::array<T, N>& a2) {
  std::array<T, N> ret;
  for (size_t i = 0; i < N; ++i) {
    ret[i] = a1[i] - a2[i];
  }
  return ret;
}

template <size_t N, class T>
std::array<T, N> GetIndexedElements(const std::array<std::vector<T>, N>& a,
                                    const std::array<bool, N>& needs_replans,
                                    const std::array<std::size_t, N>& indices,
                                    const T& zero) {
  std::array<T, N> ret = MakeArray<N>(zero);
  for (size_t i = 0; i < N; ++i) {
    if (needs_replans[i]) {
      const auto& v = a[i];
      const size_t& index = indices[i];
      ret[i] = v[index];
    }
  }
  return ret;
}

template <size_t N, class Container>
size_t MaxDatastructureSize(const std::array<Container, N>& v) {
  size_t max = 0;
  for (const auto& e : v) {
    max = std::max(max, e.size());
  }
  return max;
}

}  // namespace array_util

#endif  // SRC_UTIL_ARRAY_UTIL_H_
