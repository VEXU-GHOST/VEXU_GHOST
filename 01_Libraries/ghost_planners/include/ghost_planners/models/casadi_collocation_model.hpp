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

#include <string>
#include <unordered_map>
#include <casadi/casadi.hpp>
#include "yaml-cpp/yaml.h"

namespace ghost_planners
{

class CasadiCollocationModel
{
public:
  CasadiCollocationModel(std::string config_file);

  void initCostFunction();
  void initConstraintVector();

  casadi::SX getIntegrationConstraints();
  casadi::SX getInitialStateConstraint();

  // Shorthand to get symbolic state variable by name
  casadi::SX getState(std::string name)
  {
    return state_vector_(state_index_map_.at(name));
  }

  // Shorthand to get symbolic parameter by name
  casadi::SX getParam(std::string name)
  {
    return param_vector_(param_index_map_.at(name));
  }

  // Shorthand to get knot string prefix from knotpoint index
  std::string getKnotPrefix(int i)
  {
    return "k" + std::to_string(i) + "_";
  }

  const std::vector<double> & getTimeVector()
  {
    return time_vector_;
  }

  const casadi::SX & getStateVector()
  {
    return state_vector_;
  }

  const casadi::SX & getParamVector()
  {
    return param_vector_;
  }

private:
  YAML::Node config_;

  std::vector<double> time_vector_;
  float time_horizon_;
  float dt_;
  int num_knots_;
  int num_opt_vars_;
  int num_states_;
  int num_params_;

  std::vector<std::string> state_names_;
  std::vector<std::string> input_names_;
  std::vector<std::string> param_names_;
  std::vector<std::pair<std::string, std::string>> integration_state_name_pairs_;

  std::unordered_map<std::string, int> state_index_map_;
  std::unordered_map<std::string, int> param_index_map_;

  casadi::SX state_vector_;
  casadi::SX param_vector_;

  casadi::SX cost_function_;
  casadi::SX constraint_vector_;

  // std::unordered_map<std::string, std::vector<double>> state_solution_map_;
};

} // namespace ghost_planners
