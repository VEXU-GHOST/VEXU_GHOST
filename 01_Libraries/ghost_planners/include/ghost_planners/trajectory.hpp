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

#include <vector>
#include <string>
#include <unordered_map>

namespace ghost_planners
{

class Trajectory
{
  using Node = std::vector<double>;

public:
  Trajectory(
    std::vector<std::string> state_names,
    std::vector<double> init_times = std::vector<double>{},
    std::vector<Node> init_nodes = std::vector<Node>{});

  /**
   * @brief Adds new state vector at the requested time.
   *
   * @param time double representing node time
   * @param node std::vector<double> representing the current state vector
   */
  void addNode(double time, Node node);

  /**
   * @brief Retrieves state vector at requested time via linear interpolation. Querying time outside of the trajectory's time
   * bounds will return the respective bound (i.e. the initial or final state vector).
   *
   * @param time
   * @return const Node& state vector
   */
  const Node & getNode(double time) const;

  /**
   * @brief Retrieves a state by name at requested time via linear interpolation. Querying time outside of the trajectory's
   * time bounds will return the respective bound (i.e. the initial or final state).
   *
   * @param time
   * @param state_name
   * @return double state
   */
  double getState(const std::string & state_name, double time) const;

  /**
   * @brief Returns the full trajectory for an individual state as a timeseries
   *
   * @param component_name
   * @return std::vector<double>
   */
  std::vector<double> getStateTrajectory(
    const std::string & component_name,
    const std::vector<double> & time_vector) const;

  /**
   * @brief Get vector of state names
   *
   * @return const std::vector<std::string>&
   */
  const std::vector<std::string> & getStateNames() const
  {
    return m_state_names;
  }

  /**
   * @brief Returns size of the state vector
   *
   * @return size_t
   */
  size_t getStateVectorSize() const
  {
    return m_state_vector_size;
  }

protected:
  // Trajectory Data
  std::vector<Node> m_state_trajectory;
  std::vector<double> m_time_vector;

  std::vector<std::string> m_state_names;
  std::unordered_map<std::string, int> m_state_index_map;
  size_t m_state_vector_size;

};

} // namespace ghost_planners
