// Copyright 2017 - 2018 kvedder@umass.edu
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
#include <limits>
#include <vector>

#ifndef SRC_UTIL_VECTOR_UTIL_H_
#define SRC_UTIL_VECTOR_UTIL_H_

namespace vector_util {

template <class T>
T SumVector(const std::vector<T>& v, const T zero = 0) {
  T acc = zero;
  for (const auto& e : v) {
    acc += e;
  }
  return acc;
}

template <class T>
std::vector<T> AddToEachElement(const std::vector<T>& a1, const T& val) {
  std::vector<T> ret(a1.size());
  for (size_t i = 0; i < a1.size(); ++i) {
    ret[i] = a1[i] + val;
  }
  return ret;
}

template <class T>
std::vector<T> MultiplyToEachElement(const std::vector<T>& a1, const T& val) {
  std::vector<T> ret(a1.size());
  for (size_t i = 0; i < a1.size(); ++i) {
    ret[i] = a1[i] * val;
  }
  return ret;
}

template <class T>
std::vector<T> AddVectorElements(const std::vector<T>& a1,
                                 const std::vector<T>& a2) {
  std::vector<T> ret(a1.size());
  for (size_t i = 0; i < a1.size(); ++i) {
    ret[i] = a1[i] + a2[i];
  }
  return ret;
}

template <class T>
T MinElement(const std::vector<T>& v, const T zero = 0) {
  T acc = zero;
  for (const auto& e : v) {
    acc = std::min(acc, e);
  }
  return acc;
}

}  // namespace vector_util

#endif  // SRC_UTIL_VECTOR_UTIL_H_
