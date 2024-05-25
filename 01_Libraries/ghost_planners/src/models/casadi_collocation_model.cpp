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

#include "ghost_planners/models/casadi_collocation_model.hpp"

namespace ghost_planners
{

using namespace casadi;

CasadiCollocationModel::CasadiCollocationModel(std::string config_file)
{
  config_ = YAML::LoadFile(config_file);

  // Load configuration from YAML
  time_horizon_ = config_["time_horizon"].as<float>();
  dt_ = config_["dt"].as<float>();

  state_names_ = config_["base_state_names"].as<std::vector<std::string>>();
  input_names_ = config_["base_input_names"].as<std::vector<std::string>>();
  param_names_ = config_["param_names"].as<std::vector<std::string>>();

  integration_state_name_pairs_ = config_["integration_state_pairs"].as<std::vector<std::pair<std::string,
      std::string>>>();

  // Add initial state vector to param names
  for (const auto & state_name : state_names_) {
    param_names_.push_back("init_" + state_name);
  }

  num_knots_ = int(time_horizon_ / dt_) + 1;
  num_states_ = state_names_.size();
  num_opt_vars_ = num_states_ * num_knots_;
  num_params_ = param_names_.size();

  // Initialize containers for optimization variables
  state_vector_ = casadi::SX::zeros(num_opt_vars_);
  param_vector_ = casadi::SX::zeros(num_params_);

  time_vector_ = std::vector<double>(num_knots_);
  for (int i = 0; i < num_knots_; i++) {
    time_vector_[i] = i * dt_;
  }

  // Generate optimization variables and populate state_vector and state_index_map
  int state_index = 0;
  for (int i = 0; i < num_knots_; i++) {
    std::string knot_prefix = getKnotPrefix(i);
    for (auto name : state_names_) {
      state_index_map_[knot_prefix + name] = state_index;
      state_vector_(state_index) = SX::sym(knot_prefix + name);
      state_index++;
    }
  }

  // Generate optimization parameters and populate param_vector and param_index_map
  int param_index = 0;
  for (auto & param_name : param_names_) {
    param_index_map_[param_name] = param_index;
    param_vector_(param_index) = SX::sym(param_name);
    param_index++;
  }

  initConstraintVector();
  initCostFunction();
}

void CasadiCollocationModel::initConstraintVector()
{
  constraint_vector_ = vertcat(
    getIntegrationConstraints(),
    getInitialStateConstraint());
}

void CasadiCollocationModel::initCostFunction()
{
  cost_function_ = SX::zeros(1);

  // Apply Quadratic costs
  for (int k = 0; k < num_knots_ - 1; k++) {
    std::string curr_knot_prefix = getKnotPrefix(k);
    std::string next_knot_prefix = getKnotPrefix(k + 1);

    // Normalize base acceleration (essentially averaging adjacent values via trapezoidal quadrature)
    cost_function_ += 1 / dt_ *
      (pow(
        getState(curr_knot_prefix + "base_accel_x"),
        2) + pow(getState(next_knot_prefix + "base_accel_x"), 2));

    // Minimize jerk via finite difference
    cost_function_ += 1 / dt_ *
      (pow(
        getState(next_knot_prefix + "base_accel_x") -
        getState(curr_knot_prefix + "base_accel_x"), 2));
  }
}

SX CasadiCollocationModel::getIntegrationConstraints()
{
  // Integration constraints
  auto integration_constraints_vector =
    SX::zeros(integration_state_name_pairs_.size() * (num_knots_ - 1));
  int integration_constraint_index = 0;
  for (int k = 0; k < num_knots_ - 1; k++) {
    std::string curr_knot_prefix = getKnotPrefix(k);
    std::string next_knot_prefix = getKnotPrefix(k + 1);

    for (auto & pair : integration_state_name_pairs_) {
      auto x0 = curr_knot_prefix + pair.first;
      auto x1 = next_knot_prefix + pair.first;
      auto dx0 = curr_knot_prefix + pair.second;
      auto dx1 = next_knot_prefix + pair.second;

      // X1 - X0 = 1/2 * DT * (dX1 + dX0)
      integration_constraints_vector(integration_constraint_index) = 2 * (getState(x1) - getState(
          x0)) / dt_ - getState(dx1) - getState(dx0);
      integration_constraint_index++;
    }
  }
  return integration_constraints_vector;
}

SX CasadiCollocationModel::getInitialStateConstraint()
{
  return vertcat(
    getState("k0_base_pose_x") - getParam("init_base_pose_x"),
    -getState("k0_base_pose_x") + getParam("init_base_pose_x"),
    getState("k0_base_vel_x") - getParam("init_base_vel_x"),
    -getState("k0_base_vel_x") - getParam("init_base_vel_x"));
}

} // namespace ghost_planners
