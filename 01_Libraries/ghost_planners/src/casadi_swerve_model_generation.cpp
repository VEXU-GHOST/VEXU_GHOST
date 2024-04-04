#include <iostream>
#include <unordered_map>
#include "eigen3/Eigen/Geometry"
#include <casadi/casadi.hpp>

#include <ghost_util/unit_conversion_utils.hpp>

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;
using namespace casadi;

int main(int argc, char *argv[]){
	////////////////////////////////////
	///// User Input Configuration /////
	////////////////////////////////////
	const float TIME_HORIZON = 1.0;
	const float DT = 0.05;
	const int NUM_SWERVE_MODULES = 0;

	// 2D Point Mass model
	const std::vector<std::string> STATE_NAMES = {
		"base_pose_x",
		"base_pose_y",
		"base_pose_theta",
		"base_vel_x",
		"base_vel_y",
		"base_vel_theta",
		"base_accel_x",
		"base_accel_y",
		"base_accel_theta"};

	std::vector<std::string> JOINT_STATE_NAMES = {
		// "steering_angle",
		// "steering_vel",
		// "steering_accel",
		// "wheel_vel",
		// "wheel_accel",
		// "voltage_1",
		// "voltage_2",
	};
	const std::vector<std::string> PARAM_NAMES = {
		"init_pose_x",
		"init_pose_y",
		"init_pose_theta",
		"init_vel_x",
		"init_vel_y",
		"init_vel_theta",
		"des_vel_x",
		"des_vel_y",
		"des_vel_theta",
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
	auto get_state = [&state_vector, &state_index_map](std::string name){
						 return state_vector(state_index_map.at(name));
					 };

	// Shorthand to get symbolic parameter by name
	auto get_param = [&param_vector, &param_index_map](std::string name){
						 return param_vector(param_index_map.at(name));
					 };

	// Shorthand to get knot string prefix from knotpoint index
	auto get_knot_prefix = [](int i){
							   return "k" + std::to_string(i) + "_";
						   };

	//////////////////////////////////////////////
	///// Initialize Time, State, and Inputs /////
	//////////////////////////////////////////////
	// Populate time vector
	std::vector<double> time_vector(NUM_KNOTS);
	for(int i = 0; i < NUM_KNOTS; i++){
		time_vector[i] = i * DT;
	}

	// Generate optimization variables and populate state_vector and state_index_map
	int state_index = 0;
	for(int i = 0; i < NUM_KNOTS; i++){
		std::string knot_prefix = get_knot_prefix(i);
		for(auto name : STATE_NAMES){
			state_index_map[knot_prefix + name] = state_index;
			state_vector(state_index) = SX::sym(knot_prefix + name);
			state_index++;
		}

		for(int m = 1; m < NUM_SWERVE_MODULES + 1; m++){
			std::string module_prefix = knot_prefix + "m" + std::to_string(m) + "_";
			for(auto name : JOINT_STATE_NAMES){
				state_index_map[module_prefix + name] = state_index;
				state_vector(state_index) = SX::sym(module_prefix + name);
				state_index++;
			}
		}
	}

	// Generate optimization parameters and populate param_vector and param_index_map
	int param_index = 0;
	for(auto &param_name : PARAM_NAMES){
		param_index_map[param_name] = param_index;
		param_vector(param_index) = SX::sym(param_name);
		param_index++;
	}

	/////////////////////////////////
	///// Formulate Constraints /////
	/////////////////////////////////

	// EULER INTEGRATION CONSTRAINTS
	// List pairs of base state and derivative state
	std::vector<std::pair<std::string, std::string> > euler_integration_state_names = {
		std::pair<std::string, std::string>{"base_pose_x", "base_vel_x"},
		std::pair<std::string, std::string>{"base_pose_y", "base_vel_y"},
		std::pair<std::string, std::string>{"base_pose_theta", "base_vel_theta"},
		std::pair<std::string, std::string>{"base_vel_x", "base_accel_x"},
		std::pair<std::string, std::string>{"base_vel_y", "base_accel_y"},
		std::pair<std::string, std::string>{"base_vel_theta", "base_accel_theta"},
	};

	// Add joint states for each swerve module to the integration states pairs
	for(int m = 1; m < NUM_SWERVE_MODULES + 1; m++){
		std::string module_prefix = "m" + std::to_string(m) + "_";
		euler_integration_state_names.push_back(std::pair<std::string, std::string>{module_prefix + "steering_vel", module_prefix + "steering_accel"});
		euler_integration_state_names.push_back(std::pair<std::string, std::string>{module_prefix + "wheel_vel", module_prefix + "wheel_accel"});
	}

	// Populate euler integration constraints for state vector
	auto integration_constraints_vector = SX::zeros(euler_integration_state_names.size() * (NUM_KNOTS - 1));
	int integration_constraint_index = 0;
	for(int k = 0; k < NUM_KNOTS - 1; k++){
		std::string curr_knot_prefix = get_knot_prefix(k);
		std::string next_knot_prefix = get_knot_prefix(k + 1);

		for(auto &pair : euler_integration_state_names){
			auto x0 = curr_knot_prefix + pair.first;
			auto x1 = next_knot_prefix + pair.first;
			auto dx0 = curr_knot_prefix + pair.second;
			auto dx1 = next_knot_prefix + pair.second;

			// X1 - X0 = 1/2 * DT * (dX1 + dX0)
			integration_constraints_vector(integration_constraint_index) = 2 * (get_state(x1) - get_state(x0)) / DT - get_state(dx1) - get_state(dx0);
			integration_constraint_index++;
		}
	}

	// Initial State Constraints
	std::vector<std::pair<std::string, std::string> > initial_state_constraint_param_pairs{
		std::pair<std::string, std::string>{"k0_base_pose_x", "init_pose_x"},
		std::pair<std::string, std::string>{"k0_base_pose_y", "init_pose_y"},
		std::pair<std::string, std::string>{"k0_base_pose_theta", "init_pose_theta"},
		std::pair<std::string, std::string>{"k0_base_vel_x", "init_vel_x"},
		std::pair<std::string, std::string>{"k0_base_vel_y", "init_vel_y"},
		std::pair<std::string, std::string>{"k0_base_vel_theta", "init_vel_theta"}
	};
	auto initial_state_constraint_vector = casadi::Matrix<casadi::SXElem>();

	for(const auto& pair : initial_state_constraint_param_pairs){
		initial_state_constraint_vector = vertcat(initial_state_constraint_vector, get_state(pair.first) - get_param(pair.second));
	}
	// Combine all constraints into single vector
	auto constraints = vertcat(
		integration_constraints_vector,
		initial_state_constraint_vector);

	// Optimization Variables Limits
	auto lbx = DM::ones(NUM_OPT_VARS) * -DM::inf();
	auto ubx = DM::ones(NUM_OPT_VARS) * DM::inf();

	for(int k = 0; k < NUM_KNOTS; k++){
		std::string curr_knot_prefix = get_knot_prefix(k);
		lbx(state_index_map[curr_knot_prefix + "base_vel_x"]) = -1.389;
		ubx(state_index_map[curr_knot_prefix + "base_vel_x"]) = 1.389;
		lbx(state_index_map[curr_knot_prefix + "base_vel_y"]) = -1.389;
		ubx(state_index_map[curr_knot_prefix + "base_vel_y"]) = 1.389;
		lbx(state_index_map[curr_knot_prefix + "base_vel_theta"]) = -7.0305;
		ubx(state_index_map[curr_knot_prefix + "base_vel_theta"]) = 7.0305;

		lbx(state_index_map[curr_knot_prefix + "base_accel_x"]) = -5.5755;
		ubx(state_index_map[curr_knot_prefix + "base_accel_x"]) = 5.5755;
		lbx(state_index_map[curr_knot_prefix + "base_accel_y"]) = -5.5755;
		ubx(state_index_map[curr_knot_prefix + "base_accel_y"]) = 5.5755;
		lbx(state_index_map[curr_knot_prefix + "base_accel_theta"]) = -45.97811;
		ubx(state_index_map[curr_knot_prefix + "base_accel_theta"]) = 45.97811;
	}

	///////////////////////////
	///// Formulate Costs /////
	///////////////////////////
	// Initialize empty cost function
	auto f = SX::zeros(1);

	// Apply Quadratic costs
	for(int k = 0; k < NUM_KNOTS - 1; k++){
		std::string curr_knot_prefix = "k" + std::to_string(k) + "_";
		std::string next_knot_prefix = "k" + std::to_string(k + 1) + "_";

		// Normalize base acceleration (essentially averaging adjacent values via trapezoidal quadrature)
		f += 1 / DT * (pow(get_state(curr_knot_prefix + "base_accel_x"), 2) + pow(get_state(next_knot_prefix + "base_accel_x"), 2));
		f += 1 / DT * (pow(get_state(curr_knot_prefix + "base_accel_y"), 2) + pow(get_state(next_knot_prefix + "base_accel_y"), 2));
		f += 1 / DT * (pow(get_state(curr_knot_prefix + "base_accel_theta"), 2) + pow(get_state(next_knot_prefix + "base_accel_theta"), 2));

		// Penalize Velocity Deviation
		f += 10 * 1 / DT * (pow(get_param("des_vel_x") - get_state(curr_knot_prefix + "base_vel_x"), 2) + pow(get_param("des_vel_x") - get_state(next_knot_prefix + "base_vel_x"), 2));
		f += 10 * 1 / DT * (pow(get_param("des_vel_y") - get_state(curr_knot_prefix + "base_vel_y"), 2) + pow(get_param("des_vel_y") - get_state(next_knot_prefix + "base_vel_y"), 2));
		f += 10 * 1 / DT * (pow(get_param("des_vel_theta") - get_state(curr_knot_prefix + "base_vel_theta"), 2) + pow(get_param("des_vel_theta") - get_state(next_knot_prefix + "base_vel_theta"), 2));

		// Minimize jerk via finite difference
		f += 1 / DT * (pow(get_state(next_knot_prefix + "base_accel_x") - get_state(curr_knot_prefix + "base_accel_x"), 2));
		f += 1 / DT * (pow(get_state(next_knot_prefix + "base_accel_y") - get_state(curr_knot_prefix + "base_accel_y"), 2));
		f += 1 / DT * (pow(get_state(next_knot_prefix + "base_accel_theta") - get_state(curr_knot_prefix + "base_accel_theta"), 2));
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
	auto solver = nlpsol("example_trajectory_optimization", "ipopt", nlp_config, solver_config);

	//////////////////////
	///// Test Solve /////
	//////////////////////
	std::map<std::string, DM> solver_args, res;
	solver_args["lbx"] = lbx;
	solver_args["ubx"] = ubx;
	solver_args["lbg"] = 0;
	solver_args["ubg"] = 0;
	solver_args["x0"] = DM::zeros(NUM_OPT_VARS);
	solver_args["p"] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.5, 5.0};
	res = solver(solver_args);

	/////////////////////////////
	///// Evaluate Solution /////
	/////////////////////////////
	// Print the solution
	std::cout << "-----" << std::endl;
	std::cout << "Optimal solution for p = " << solver_args.at("p") << ":" << std::endl;
	std::cout << "Objective: " << res.at("f") << std::endl;

	// Unpack solution into individual time series
	std::unordered_map<std::string, std::vector<double> > state_solution_map;
	auto raw_solution_vector = std::vector<double>(res.at("x"));
	for(auto &name : STATE_NAMES){
		// Allocate timeseries vector for each state/input variable
		state_solution_map[name] = std::vector<double>(NUM_KNOTS);

		// Iterate through timeseries and add final state values to solution vector
		for(int k = 0; k < NUM_KNOTS; k++){
			std::string knot_prefix = get_knot_prefix(k);
			state_solution_map[name][k] = raw_solution_vector[state_index_map.at(knot_prefix + name)];
		}
	}

	std::cout << time_vector.size() << std::endl;
	std::cout << state_solution_map["base_pose_x"].size() << std::endl;
	std::cout << state_solution_map["base_vel_x"].size() << std::endl;
	std::cout << state_solution_map["base_accel_x"].size() << std::endl;

	std::vector<double> x_angle_components;
	std::vector<double> y_angle_components;

	x_angle_components.reserve(state_solution_map["base_pose_theta"].size());
	y_angle_components.reserve(state_solution_map["base_pose_theta"].size());

	for(const auto & angle : state_solution_map["base_pose_theta"]){
		x_angle_components.push_back(cos(angle));
		y_angle_components.push_back(sin(angle));
	}

	plt::figure();
	plt::quiver(state_solution_map["base_pose_x"], state_solution_map["base_pose_y"], x_angle_components, y_angle_components);

	plt::figure();
	plt::suptitle("X");
	plt::subplot(3, 1, 1);
	plt::plot(time_vector, state_solution_map["base_pose_x"]);
	plt::subplot(3, 1, 2);
	plt::plot(time_vector, state_solution_map["base_vel_x"]);
	plt::subplot(3, 1, 3);
	plt::plot(time_vector, state_solution_map["base_accel_x"]);

	plt::figure();
	plt::suptitle("Y");
	plt::subplot(3, 1, 1);
	plt::plot(time_vector, state_solution_map["base_pose_y"]);
	plt::subplot(3, 1, 2);
	plt::plot(time_vector, state_solution_map["base_vel_y"]);
	plt::subplot(3, 1, 3);
	plt::plot(time_vector, state_solution_map["base_accel_y"]);

	plt::figure();
	plt::suptitle("Theta");
	plt::subplot(3, 1, 1);
	plt::plot(time_vector, state_solution_map["base_pose_theta"]);
	plt::subplot(3, 1, 2);
	plt::plot(time_vector, state_solution_map["base_vel_theta"]);
	plt::subplot(3, 1, 3);
	plt::plot(time_vector, state_solution_map["base_accel_theta"]);
	plt::show();

	return 0;
}