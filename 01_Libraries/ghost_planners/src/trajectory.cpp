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
#include <ghost_util/search_util.hpp>
#include <ghost_util/math_util.hpp>

#include <iostream>

using ghost_util::clampedVectorInterpolate;
using ghost_util::getInsertionIndexFromSortedVector;
using ghost_util::insertValueIntoSortedVector;


namespace ghost_planners
{

Trajectory::Trajectory(
  std::vector<std::string> state_names)
: m_state_names(state_names),
  m_time_vector(),
  m_state_trajectory(),
  m_state_vector_size(state_names.size())
{
  // Populate state index
  for (int i = 0; i < m_state_names.size(); i++) {
    m_state_index_map[m_state_names[i]] = i;
  }

}

void Trajectory::clearNodes()
{
  m_state_trajectory.clear();
  m_time_vector.clear();
}

void Trajectory::reset(std::vector<std::string> state_names)
{
  m_state_trajectory.clear();
  m_time_vector.clear();

  m_state_names.clear();
  m_state_names = state_names;

  m_state_vector_size = state_names.size();

  m_state_index_map.clear();

  // Populate state index
  for (int i = 0; i < m_state_names.size(); i++) {
    m_state_index_map[m_state_names[i]] = i;
  }

}

void Trajectory::addNode(double time, Node node)
{
  if (node.size() != m_state_vector_size) {
    throw std::runtime_error(
            std::string("[Trajectory::addNode] Error: Node size ") +
            std::to_string(node.size()) + " does not match state_vector size " +
            std::to_string(m_state_vector_size));
  }

  // Check if time already exists (within tolerance)
  auto it = std::find_if(
    m_time_vector.begin(), m_time_vector.end(), [time](double t2) {
      return std::fabs(time - t2) < 1e-6;
    });

  if (it != m_time_vector.end()) {
    m_state_trajectory[it - m_time_vector.begin()] = node;
  } else {
    int insertion_index = insertValueIntoSortedVector(time, m_time_vector);
    m_state_trajectory.insert(m_state_trajectory.begin() + insertion_index, node);
  }
}

Trajectory::Node Trajectory::getNode(double time) const
{
  if (m_state_trajectory.empty()) {
    return std::vector<double>(m_state_vector_size, 0.0);
  }

  int index = getInsertionIndexFromSortedVector(time, m_time_vector);

  if (index == 0) {
    return m_state_trajectory.front();
  }
  if (index == m_time_vector.size()) {
    return m_state_trajectory.back();
  }
  return ghost_util::clampedVectorInterpolate(
    time, m_time_vector[index - 1], m_time_vector[index],
    m_state_trajectory[index - 1], m_state_trajectory[index]);
}

double Trajectory::getState(const std::string & name, double time) const
{
  if (m_state_index_map.count(name) == 0) {
    throw std::runtime_error(
            std::string(
              "[Trajectory::getStateIndex] Error: state ") + name + " does not exist!");
  }
  return getNode(time)[getStateIndex(name)];
}

std::vector<double> Trajectory::getStateTrajectory(
  const std::string & name,
  const std::vector<double> & time_vector,
  double time_offset) const
{
  if (m_state_index_map.count(name) == 0) {
    throw std::runtime_error(
            std::string(
              "[Trajectory::getStateIndex] Error: state ") + name + " does not exist!");
  }

  std::vector<double> state_trajectory;
  state_trajectory.reserve(time_vector.size());

  for (const auto & time : time_vector) {
    state_trajectory.push_back(getState(name, time + time_offset));
  }
  return state_trajectory;
}

std::vector<double> Trajectory::getFlattenedTrajectory(
  const std::vector<double> & time_vector, double time_offset) const
{
  std::vector<double> flattened_trajectory;
  flattened_trajectory.reserve(m_state_vector_size * time_vector.size());

  for (const auto & t : time_vector) {
    auto node = getNode(t + time_offset);
    flattened_trajectory.insert(flattened_trajectory.end(), node.begin(), node.end());
  }
  return flattened_trajectory;
}
} // namespace ghost_planners
