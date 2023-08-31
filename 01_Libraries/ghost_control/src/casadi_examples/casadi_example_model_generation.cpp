#include <iostream>
#include <unordered_map>
#include "eigen3/Eigen/Geometry"
#include <casadi/casadi.hpp>
#include "matplotlibcpp.h"
#include "yaml-cpp/yaml.h"

#include "ghost_control/models/casadi_collocation_model.hpp"

namespace plt = matplotlibcpp;
using namespace casadi;

/*
   This file is a working example for how to generate optimization problems using the CasADi symbolic toolbox.

   Here, we use a 1D point mass model, with equality constraints for the initial and final states and a quadratic acceleration and jerk cost.
   The Problem Formulation is then exported to C++ code and can be called externally.
 */
int main(int argc, char *argv[]){
	auto yaml_file = std::string(getenv("HOME")) + "/VEXU_GHOST/ghost_control/config/casadi_example_model_generation.yaml";
	auto model = ghost_control::CasadiCollocationModel(yaml_file);

	// /////////////////////////
	// ///// Create Solver /////
	// /////////////////////////
	// SXDict nlp_config{
	//     {"x", state_vector},
	//     {"f", f},
	//     {"g", constraints},
	//     {"p", param_vector}};

	// Dict solver_config{
	//     // {"verbose", true},
	//     // {"ipopt.print_level", 7},
	//     // {"ipopt.timing_statistics", "yes"}
	// };

	// auto solver = nlpsol("example_trajectory_optimization", "ipopt", nlp_config, solver_config);

	// //////////////////////
	// ///// Test Solve /////
	// //////////////////////

	// // Set parameters
	// auto param_config_vector = DM::zeros(NUM_PARAMS);
	// param_config_vector(param_index_map["final_base_pose_x"]) = 5.0;
	// param_config_vector(param_index_map["final_base_vel_x"]) = 0.0;
	// std::map<std::string, DM> solver_args, res;

	// // Config solver args
	// solver_args["lbx"] = DM::ones(NUM_OPT_VARS) * -DM::inf();
	// solver_args["ubx"] = DM::ones(NUM_OPT_VARS) * DM::inf();
	// solver_args["lbg"] = 0;
	// solver_args["ubg"] = 0;
	// solver_args["x0"] = DM::ones(NUM_OPT_VARS);
	// solver_args["p"] = param_config_vector;
	// res = solver(solver_args);

	// /////////////////////////////
	// ///// Evaluate Solution /////
	// /////////////////////////////
	// // Print the solution
	// std::cout << "-----" << std::endl;
	// std::cout << "Optimal solution for p = " << solver_args.at("p") << ":" << std::endl;
	// std::cout << "Objective: " << res.at("f") << std::endl;

	// // Unpack solution into individual time series
	// std::unordered_map<std::string, std::vector<double>> state_solution_map;
	// auto raw_solution_vector = std::vector<double>(res.at("x"));
	// for (auto &name : STATE_NAMES)
	// {
	//     // Allocate timeseries vector for each state/input variable
	//     state_solution_map[name] = std::vector<double>(NUM_KNOTS);

	//     // Iterate through timeseries and add final state values to solution vector
	//     for (int k = 0; k < NUM_KNOTS; k++)
	//     {
	//         std::string knot_prefix = get_knot_prefix(k);
	//         state_solution_map[name][k] = raw_solution_vector[state_index_map.at(knot_prefix + name)];
	//     }
	// }

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

	return 0;
}