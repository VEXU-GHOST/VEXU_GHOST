/*
 *   Copyright (c) 2024 Melissa Cruz
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

#include <iostream>
#include <unordered_map>
#include <memory>
#include <string>
#include "eigen3/Eigen/Geometry"
#include <casadi/casadi.hpp>
#include "matplotlibcpp.h"
#include <rclcpp/rclcpp.hpp>
#include <ghost_msgs/msg/state_sol.hpp>
#include <std_msgs/msg/string.hpp>

namespace plt = matplotlibcpp;
using namespace casadi;

std::unordered_map<std::string, std::vector<double>> STATE_SOL_MAP;

class RvizStream : public rclcpp::Node
{

public:
  RvizStream()
  : Node("rviz_stream")
  {
    state_sol_pub_ = this->create_publisher<ghost_msgs::msg::StateSol>(
      "/tank_mpc_sol",
      10);

    pipe_to_rviz(STATE_SOL_MAP);
  }

  ~RvizStream() {}

  void pipe_to_rviz(
    std::unordered_map<std::string,
    std::vector<double>> & state_sol)
  {
    ghost_msgs::msg::StateSol msg{};
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "map";
    msg.base_pose_x_traj = state_sol["base_pose_x"];
    msg.base_pose_y_traj = state_sol["base_pose_y"];
    msg.base_pose_tht_traj = state_sol["base_pose_tht"];

    state_sol_pub_->publish(msg);
  }

private:
  rclcpp::Publisher<ghost_msgs::msg::StateSol>::SharedPtr state_sol_pub_;

};


/*
   This file is a working example for how to generate optimization problems using the CasADi symbolic toolbox.

   Here, we use a 1D point mass model, with equality constraints for the initial and final states and a quadratic acceleration and jerk cost.
   The Problem Formulation is then exported to C++ code and can be called externally.
 */
int main(int argc, char * argv[])
{
  ////////////////////////////////////
  ///// User Input Configuration /////
  ////////////////////////////////////
  constexpr float TIME_HORIZON = 1.0;
  constexpr float DT = 0.01;

  // Params
  double mass = 10.0;
  double inertia = 0.14868;
  double init_pose_x = -1.0;
  double init_pose_y = 0.0;
  double init_pose_tht = 0.0;
  double init_vel_x = 0.0;
  double init_vel_y = 0.0;
  double init_vel_tht = 0.0;
  double des_vel_x = 0.75;
  double des_vel_y = 0.0;
  double des_vel_tht = 0.0;
  double final_pose_x = 1.0;

  const std::vector<std::string> STATE_NAMES = {
    "base_pose_x",
    "base_pose_y",
    "base_pose_tht",
    "base_vel_x",
    "base_vel_y",
    "base_vel_tht",
    "base_accel_x",
    "base_accel_y",
    "base_accel_tht"};
  const std::vector<std::string> PARAM_NAMES = {
    "mass",
    "inertia",
    "init_pose_x",
    "init_pose_y",
    "init_pose_tht",
    "init_vel_x",
    "init_vel_y",
    "init_vel_tht",
    "des_vel_x",
    "des_vel_y",
    "des_vel_tht",
    "final_pose_x"};

  // List pairs of base state and derivative state
  const std::vector<std::pair<std::string, std::string>> INTEGRATION_STATE_NAME_PAIRS = {
    std::pair<std::string, std::string>{"base_pose_x", "base_vel_x"},
    std::pair<std::string, std::string>{"base_pose_y", "base_vel_y"},
    std::pair<std::string, std::string>{"base_pose_tht", "base_vel_tht"},

    std::pair<std::string, std::string>{"base_vel_x", "base_accel_x"},
    std::pair<std::string, std::string>{"base_vel_y", "base_accel_y"},
    std::pair<std::string, std::string>{"base_vel_tht", "base_accel_tht"},
  };

  // Initial State Constraints
  std::vector<std::pair<std::string, std::string>> initial_state_constraint_param_pairs{
    std::pair<std::string, std::string>{"k0_base_pose_x", "init_pose_x"},
    std::pair<std::string, std::string>{"k0_base_pose_y", "init_pose_y"},
    std::pair<std::string, std::string>{"k0_base_pose_tht", "init_pose_tht"},
    std::pair<std::string, std::string>{"k0_base_vel_x", "init_vel_x"},
    std::pair<std::string, std::string>{"k0_base_vel_y", "init_vel_y"},
    std::pair<std::string, std::string>{"k0_base_vel_tht", "init_vel_tht"}
  };

  ////////////////////////////////////////////
  ///// Initialize Problem Configuration /////
  ////////////////////////////////////////////
  const int NUM_KNOTS = int(TIME_HORIZON / DT) + 1;
  const int NUM_STATES = STATE_NAMES.size();
  const int NUM_OPT_VARS = NUM_STATES * NUM_KNOTS;

  // Initialize containers for optimization variables
  std::unordered_map<std::string, int> state_index_map;
  auto state_vector = casadi::SX::zeros(NUM_OPT_VARS);

  std::unordered_map<std::string, int> param_index_map;
  auto param_vector = casadi::SX::zeros(PARAM_NAMES.size());

  ////////////////////////////
  ///// Helper Functions /////
  ////////////////////////////
  // Shorthand to get symbolic state variable by name
  auto get_state = [&state_vector, &state_index_map](std::string name) {
      return state_vector(state_index_map.at(name));
    };

  // Shorthand to get symbolic parameter by name
  auto get_param = [&param_vector, &param_index_map](std::string name) {
      return param_vector(param_index_map.at(name));
    };

  // Shorthand to get knot string prefix from knotpoint index
  auto get_knot_prefix = [](int i) {
      return "k" + std::to_string(i) + "_";
    };

  //////////////////////////////////////////////
  ///// Initialize Time, State, and Inputs /////
  //////////////////////////////////////////////
  // Populate time vector
  std::vector<double> time_vector(NUM_KNOTS);
  for (int i = 0; i < NUM_KNOTS; i++) {
    time_vector[i] = i * DT;
  }

  // Generate optimization variables and populate state_vector and state_index_map
  int state_index = 0;
  for (int i = 0; i < NUM_KNOTS; i++) {
    std::string knot_prefix = get_knot_prefix(i);
    for (auto name : STATE_NAMES) {
      state_index_map[knot_prefix + name] = state_index;
      state_vector(state_index) = SX::sym(knot_prefix + name);
      state_index++;
    }
  }

  // Generate optimization parameters and populate param_vector and param_index_map
  int param_index = 0;
  for (auto & param_name : PARAM_NAMES) {
    param_index_map[param_name] = param_index;
    param_vector(param_index) = SX::sym(param_name);
    param_index++;
  }

  /////////////////////////////////
  ///// Formulate Constraints /////
  /////////////////////////////////
  // Integration constraints
  auto integration_constraints_vector =
    SX::zeros(INTEGRATION_STATE_NAME_PAIRS.size() * (NUM_KNOTS - 1));
  int integration_constraint_index = 0;
  for (int k = 0; k < NUM_KNOTS - 1; k++) {
    std::string curr_knot_prefix = get_knot_prefix(k);
    std::string next_knot_prefix = get_knot_prefix(k + 1);

    for (auto & pair : INTEGRATION_STATE_NAME_PAIRS) {
      auto x0 = curr_knot_prefix + pair.first;
      auto x1 = next_knot_prefix + pair.first;
      auto dx0 = curr_knot_prefix + pair.second;
      auto dx1 = next_knot_prefix + pair.second;

      // X1 - X0 = 1/2 * DT * (dX1 + dX0)
      integration_constraints_vector(integration_constraint_index) = 2 *
        (get_state(x1) - get_state(x0)) / DT - get_state(dx1) - get_state(dx0);
      integration_constraint_index++;
    }
  }

  // Initial and Terminal Constraints
  auto initial_state_constraint_vector = casadi::Matrix<casadi::SXElem>();

  for (const auto & pair : initial_state_constraint_param_pairs) {
    initial_state_constraint_vector = vertcat(
      initial_state_constraint_vector, get_state(
        pair.first) - get_param(pair.second));
  }

  std::string final_knot_prefix = get_knot_prefix(NUM_KNOTS - 1);
  auto final_state_constraint_vector = vertcat(
    get_state(final_knot_prefix + "base_pose_x") - get_param("final_pose_x"),
    -get_state(final_knot_prefix + "base_pose_x") + get_param("final_pose_x"),
    get_state(final_knot_prefix + "base_vel_x"),
    -get_state(final_knot_prefix + "base_vel_x"));

  // Combine all constraints into single vector
  auto constraints = vertcat(
    integration_constraints_vector,
    initial_state_constraint_vector,
    final_state_constraint_vector);

  ///////////////////////////
  ///// Formulate Costs /////
  ///////////////////////////
  // Initialize empty cost function
  auto f = SX::zeros(1);

  // Apply Quadratic costs
  for (int k = 0; k < NUM_KNOTS - 1; k++) {
    std::string curr_knot_prefix = "k" + std::to_string(k) + "_";
    std::string next_knot_prefix = "k" + std::to_string(k + 1) + "_";

    // Regularize base acceleration (essentially averaging adjacent values via trapezoidal quadrature)
    f += 0.000001 / DT *
      (pow(
        get_state(curr_knot_prefix + "base_accel_x"),
        2) + pow(get_state(next_knot_prefix + "base_accel_x"), 2));
    f += 0.000001 / DT *
      (pow(
        get_state(curr_knot_prefix + "base_accel_y"),
        2) + pow(get_state(next_knot_prefix + "base_accel_y"), 2));
    f += 0.000001 / DT *
      (pow(
        get_state(curr_knot_prefix + "base_accel_tht"),
        2) + pow(get_state(next_knot_prefix + "base_accel_tht"), 2));

    // Regularize Base Jerk
    f += 0.01 / DT *
      (pow(
        get_state(curr_knot_prefix + "base_accel_x") -
        get_state(next_knot_prefix + "base_accel_x"), 2));
    f += 0.01 / DT *
      (pow(
        get_state(curr_knot_prefix + "base_accel_y") -
        get_state(next_knot_prefix + "base_accel_y"), 2));
    f += 0.01 / DT *
      (pow(
        get_state(curr_knot_prefix + "base_accel_tht") -
        get_state(next_knot_prefix + "base_accel_tht"), 2));

    // Penalize Velocity Deviation
    f += 10000.0 * 1 / DT *
      (pow(
        get_param("des_vel_x") - get_state(curr_knot_prefix + "base_vel_x"),
        2) + pow(get_param("des_vel_x") - get_state(next_knot_prefix + "base_vel_x"), 2));
    f += 10000.0 * 1 / DT *
      (pow(
        get_param("des_vel_y") - get_state(curr_knot_prefix + "base_vel_y"),
        2) + pow(get_param("des_vel_y") - get_state(next_knot_prefix + "base_vel_y"), 2));
    f += 10000.0 * 1 / DT *
      (pow(
        get_param("des_vel_tht") - get_state(curr_knot_prefix + "base_vel_tht"),
        2) + pow(get_param("des_vel_tht") - get_state(next_knot_prefix + "base_vel_tht"), 2));
  }

  /////////////////////////
  ///// Create Solver /////
  /////////////////////////
  SXDict nlp_config{
    {"x", state_vector},
    {"f", f},
    {"g", constraints},
    {"p", param_vector}};
  Dict solver_config{
    // {"verbose", true},
    // {"ipopt.print_level", 0}
  };

  auto solver = nlpsol("casadi_tank_model", "ipopt", nlp_config, solver_config);

  //////////////////////
  ///// Test Solve /////
  //////////////////////
  std::map<std::string, DM> solver_args, res;
  solver_args["lbx"] = DM::ones(NUM_OPT_VARS) * -DM::inf();
  solver_args["ubx"] = DM::ones(NUM_OPT_VARS) * DM::inf();
  solver_args["lbg"] = 0;
  solver_args["ubg"] = 0;
  solver_args["x0"] = DM::ones(NUM_OPT_VARS);

  solver_args["p"] = {mass,
    inertia,
    init_pose_x,
    init_pose_y,
    init_pose_tht,
    init_vel_x,
    init_vel_y,
    init_vel_tht,
    des_vel_x,
    des_vel_y,
    des_vel_tht,
    final_pose_x
  };
  res = solver(solver_args);

  /////////////////////////////
  ///// Evaluate Solution /////
  /////////////////////////////
  // Print the solution
  std::cout << "-----" << std::endl;
  std::cout << "Optimal solution for p = " << solver_args.at("p") << ":" << std::endl;
  std::cout << "Objective: " << res.at("f") << std::endl;


  // Unpack solution into individual time series
  std::unordered_map<std::string, std::vector<double>> state_solution_map;
  auto raw_solution_vector = std::vector<double>(res.at("x"));
  for (auto & name : STATE_NAMES) {
    // Allocate timeseries vector for each state/input variable
    state_solution_map[name] = std::vector<double>(NUM_KNOTS);

    // Iterate through timeseries and add final state values to solution vector
    for (int k = 0; k < NUM_KNOTS; k++) {
      std::string knot_prefix = get_knot_prefix(k);
      state_solution_map[name][k] = raw_solution_vector[state_index_map.at(knot_prefix + name)];
    }
  }

  // std::cout << time_vector.size() << std::endl;
  // std::cout << state_solution_map["base_pose_x"].size() << std::endl;
  // std::cout << state_solution_map["base_vel_x"].size() << std::endl;
  // std::cout << state_solution_map["base_accel_x"].size() << std::endl;

  STATE_SOL_MAP = state_solution_map;
  // plt::figure();
  // plt::subplot(3, 1, 1);
  // plt::plot(time_vector, state_solution_map["base_pose_x"]);
  // plt::subplot(3, 1, 2);
  // plt::plot(time_vector, state_solution_map["base_vel_x"]);
  // plt::subplot(3, 1, 3);
  // plt::plot(time_vector, state_solution_map["base_accel_x"]);
  // plt::show();

  ////////////////////////////////
  ///// Generate Source Code /////
  ////////////////////////////////
  // Dict nlp_codegen_conf{
  //     {"cpp", true},
  //     {"with_header", true}};

  // std::string filename = solver.generate_dependencies("example_trajectory_optimization.cpp", nlp_codegen_conf);
  // std::cout << "Saved to " << filename << std::endl;
  ////////////////////////////////////
  /////// Instantiate ROS Node ///////
  ////////////////////////////////////
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RvizStream>());
  rclcpp::shutdown();
  return 0;
  // return 0;
}
