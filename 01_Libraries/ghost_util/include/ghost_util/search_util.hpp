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

#include <vector>
#include <algorithm>
#include <stdexcept>

namespace ghost_util
{
template<typename T>
int getInsertionIndexFromSortedVector(const T & val, const std::vector<T> & vector)
{
  if (!std::is_sorted(vector.begin(), vector.end())) {
    throw std::runtime_error("[getInsertionIndexFromSortedVector] Error: vector is not sorted!");
  }
  return std::upper_bound(vector.begin(), vector.end(), val) - vector.begin();
}

template<typename T>
int insertValueIntoSortedVector(T & val, std::vector<T> & vector)
{
  int index = getInsertionIndexFromSortedVector(val, vector);
  vector.insert(vector.begin() + index, val);
  return index;
}

} // namespace ghost_util
