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

#include "ghost_planners/trajectory.hpp"

using Node = std::vector<double>;

namespace ghost_planners
{

Trajectory::Trajectory(
  std::vector<std::string> state_names,
  std::vector<double> init_times,
  std::vector<Node> init_nodes)
{

}

void Trajectory::addNode(double time, Node node)
{

}

const Node & Trajectory::getNode(double time) const
{

}

double Trajectory::getState(const std::string & state_name, double time) const
{

}

std::vector<double> Trajectory::getStateTrajectory(
  const std::string & component_name,
  const std::vector<double> & time_vector) const
{

}

} // namespace ghost_planners
