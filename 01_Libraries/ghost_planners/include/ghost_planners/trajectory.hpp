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
#include <stdexcept>

namespace ghost_planners
{
/**
 * @brief This Trajectory class is comprised of a time vector, and a vector of state_vectors (where each element in the larger
 * vector is a full state_vector corresponding to the respective time). This is represented as follows:
 *
 * times: [t1, t2, ... , tn]
 *
 * state_vector at t1: [s1, ... , sn]
 * state_vector at t2: [s1, ... , sn]
 * state_vector at t3: [s1, ... , sn]
 * state_vector at t4: [s1, ... , sn]
 *
 * We can add "Nodes" to the trajectory, which represent a full state vector at that timestamp.
 * Then, we can query for Nodes at arbitrary times and the trajectory will interpolate between the two closest Nodes.
 * If queried before the earliest time, or after the last time, the trajectory returns the first/last node available.
 */
class Trajectory
{
  using Node = std::vector<double>;

public:
  Trajectory(
    std::vector<std::string> state_names);

  /**
   * @brief Returns true if there are no nodes in the trajectory
   *
   * @return true
   * @return false
   */
  bool empty()
  {
    return m_time_vector.empty();
  }

  /**
   * @brief Removes all existing nodes and times
   */
  void clearNodes();

  /**
   * @brief Resets Trajectory class to post-construction
   */
  void reset(std::vector<std::string> state_names);

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
   * @return Node state vector
   */
  Node getNode(double time) const;

  /**
   * @brief Retrieves a state by name at requested time via linear interpolation. Querying time outside of the trajectory's
   * time bounds will return the respective bound (i.e. the initial or final state).
   *
   * @param time
   * @param name
   * @return double state
   */
  double getState(const std::string & name, double time) const;

  /**
   * @brief Returns the time vector corresponding to trajectory nodes
   *
   * @return const std::vector<double>&
   */
  const std::vector<double> & getTimeVector() const
  {
    return m_time_vector;
  }

  /**
   * @brief Returns the full trajectory for an individual state as a timeseries
   *
   * @param name
   * @return std::vector<double>
   */
  std::vector<double> getStateTrajectory(
    const std::string & name,
    const std::vector<double> & time_vector) const;

  /**
   * @brief Returns Nodes sampled at each timestep flattened as a single vector.
   *
   * Example:
   *
   * time = [1.0, 2.0, 3.0]
   * x1 =   [5.0, 5.5, 6.0]
   * x2 =   [2.0, 2.5, 2.0]
   *
   * returns [5.0, 2.0, 5.5, 2.5, 6.0, 2.0]
   *
   * @param time_vector
   * @return std::vector<double>
   */
  std::vector<double> getFlattenedTrajectory(const std::vector<double> & time_vector) const;

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

  /**
   * @brief Returns the corresponding index in the state vector given a state name.
   *
   * @param name
   * @return int
   */
  int getStateIndex(const std::string & name)const
  {
    if (m_state_index_map.count(name) == 0) {
      throw std::runtime_error(
              std::string(
                "[Trajectory::getStateIndex] Error: state ") + name + " does not exist!");
    }
    return m_state_index_map.at(name);
  }

  bool operator==(const Trajectory & rhs) const
  {
    bool eq = true;
    eq &= (m_state_names == rhs.m_state_names);
    eq &= (m_state_vector_size == rhs.m_state_vector_size);
    eq &= (m_time_vector == rhs.m_time_vector);
    eq &= (m_state_trajectory == rhs.m_state_trajectory);
    eq &= (m_state_index_map.size() == rhs.m_state_index_map.size());

    for (const auto & [k, v] : m_state_index_map) {
      eq &= (rhs.m_state_index_map.count(k) != 0);
      if (eq) {
        eq &= (v == rhs.m_state_index_map.at(k));
      }
    }
    return eq;
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
