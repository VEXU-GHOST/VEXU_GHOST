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

#include <sstream>
#include <string>
#include <vector>

#include <iostream>

namespace ghost_util
{

template<typename T>
T convertFromString(std::string val)
{
  return (T) val;
}

template<>
int convertFromString(std::string val)
{
  return stoi(val);
}

template<>
float convertFromString(std::string val)
{
  return std::stof(val);
}

template<>
double convertFromString(std::string val)
{
  return std::stod(val);
}

template<typename T>
std::vector<T> getVectorFromString(const std::string & input, const char delim)
{
  // construct a stream from the string
  std::stringstream ss(input);
  std::vector<T> data;
  std::string s;
  while (std::getline(ss, s, delim)) {
    data.push_back(convertFromString<T>(s));
  }
  return data;
}

} // namespace ghost_util
