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


namespace ghost_planners
{

Trajectory::Trajectory(
  std::vector<std::string> state_names) : 
  m_state_names(state_names),
  m_time_vector(),
  m_state_trajectory(state_names.size(), Trajectory::Node{})
{
  // Populate state index
  for(int i = 0; i < m_state_names.size(); i++){
    m_state_index_map[m_state_names[i]] = i;
  }

}

void Trajectory::addNode(double time, Node node)
{

}

const Trajectory::Node & Trajectory::getNode(double time) const
{

}

double Trajectory::getState(const std::string & name, double time) const
{
    if(m_state_index_map.count(name) == 0){
      throw std::runtime_error(std::string("[Trajectory::getStateIndex] Error: state ") + name + " does not exist!");
    }
}

std::vector<double> Trajectory::getStateTrajectory(
  const std::string & name,
  const std::vector<double> & time_vector) const
{
      if(m_state_index_map.count(name) == 0){
      throw std::runtime_error(std::string("[Trajectory::getStateIndex] Error: state ") + name + " does not exist!");
    }

}

} // namespace ghost_planners
