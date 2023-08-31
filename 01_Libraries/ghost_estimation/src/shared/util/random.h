// Copyright 2017-2019 kvedder@umass.edu, joydeepb&cs.umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
// Random number library.
//
//========================================================================
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
//========================================================================

#include <stdint.h>
#include <random>

#ifndef SRC_UTIL_RANDOM_H_
#define SRC_UTIL_RANDOM_H_

// Your one-stop shop for generating random numbers.
namespace util_random {
class Random {
 public:
  Random() : randn_(0, 1.0), randf_(0.0, 1.0) {}

  explicit Random(uint64_t seed): 
      generator_(seed),
      randn_(0, 1.0),
      randf_(0.0, 1.0) {}

  // Generate random numbers between 0 and 1, inclusive.
  double UniformRandom();

  // Generate random numbers between a and b, inclusive.
  double UniformRandom(double a, double b);

  // Generate random numbers between min and max, inclusive.
  template <typename T>
  T RandomInt(const T min, const T max) {
    std::uniform_int_distribution<T> dist(min, max);
    return dist(generator_);
  }

  // Return a random value drawn from a Normal distribution.
  double Gaussian(const double mean, const double stddev);

 private:
  std::default_random_engine generator_;
  std::normal_distribution<double> randn_;
  std::uniform_real_distribution<double> randf_;
};
}  // namespace util_random
#endif  // SRC_UTIL_RANDOM_H_
