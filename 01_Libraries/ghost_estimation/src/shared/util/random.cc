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
#include "random.h"

#include <random>


namespace util_random {

double Random::UniformRandom(double a, double b) {
  return (b - a) * UniformRandom() + a;
}

double Random::UniformRandom() {
  return randf_(generator_);
}

double Random::Gaussian(const double mean, const double stddev) {
  return mean + stddev * randn_(generator_);
}

}  // namespace util_random
