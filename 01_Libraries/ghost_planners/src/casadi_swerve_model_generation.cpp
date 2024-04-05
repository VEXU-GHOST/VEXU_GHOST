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
	const float DT = 0.01;
	const int NUM_SWERVE_MODULES = 1;

	// 2D Point Mass model
	std::vector<std::string> STATE_NAMES = {
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
		"steering_angle",
		"steering_vel",
		"steering_accel",
		"wheel_force",
		"lateral_force"
		// "steering_vel",
		// "steering_accel",
		// "wheel_vel",
		// "wheel_accel",
		// "voltage_1",
		// "voltage_2",
	};

	// Add joint states to state name vector
	for(int m = 1; m < NUM_SWERVE_MODULES + 1; m++){
		std::string module_prefix = "m" + std::to_string(m) + "_";
		for(auto name : JOINT_STATE_NAMES){
			STATE_NAMES.push_back(module_prefix + name);
		}
	}
	const std::vector<std::string> PARAM_NAMES = {
		"mass",
		"init_pose_x",
		"init_pose_y",
		"init_pose_theta",
		"init_vel_x",
		"init_vel_y",
		"init_vel_theta",
		"des_vel_x",
		"des_vel_y",
		"des_vel_theta",
		"init_m1_steering_angle",
		"init_m1_steering_vel"
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
						 if(state_index_map.count(name) != 0){
							 return state_vector(state_index_map.at(name));
						 }
						 else{
							 throw std::runtime_error("[get_state] No state with name " + name);
						 }
					 };

	// Shorthand to get symbolic parameter by name
	auto get_param = [&param_vector, &param_index_map](std::string name){
						 if(param_index_map.count(name) != 0){
							 return param_vector(param_index_map.at(name));
						 }
						 else{
							 throw std::runtime_error("[get_param] No param with name " + name);
						 }
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
		std::pair<std::string, std::string>{"m1_steering_angle", "m1_steering_vel"},
		std::pair<std::string, std::string>{"m1_steering_vel", "m1_steering_accel"},
	};

	// Add joint states for each swerve module to the integration states pairs
	for(int m = 1; m < NUM_SWERVE_MODULES + 1; m++){
		std::string module_prefix = "m" + std::to_string(m) + "_";
		euler_integration_state_names.push_back(std::pair<std::string, std::string>{module_prefix + "steering_vel", module_prefix + "steering_accel"});
		// euler_integration_state_names.push_back(std::pair<std::string, std::string>{module_prefix + "wheel_vel", module_prefix + "wheel_accel"});
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
		std::pair<std::string, std::string>{"k0_base_vel_theta", "init_vel_theta"},
		std::pair<std::string, std::string>{"k0_m1_steering_angle", "init_m1_steering_angle"},
		std::pair<std::string, std::string>{"k0_m1_steering_vel", "init_m1_steering_vel"}
	};
	auto initial_state_constraint_vector = casadi::Matrix<casadi::SXElem>();
	for(const auto& pair : initial_state_constraint_param_pairs){
		initial_state_constraint_vector = vertcat(initial_state_constraint_vector, get_state(pair.first) - get_param(pair.second));
	}

	// acceleration dynamics constraints
	auto acceleration_dynamics_constraint_vector = casadi::Matrix<casadi::SXElem>();
	for(int k = 0; k < NUM_KNOTS; k++){
		std::string curr_knot_prefix = get_knot_prefix(k);
		auto accel_x = get_state(curr_knot_prefix + "base_accel_x");
		auto accel_y = get_state(curr_knot_prefix + "base_accel_y");
		auto wheel_force = get_state(curr_knot_prefix + "m1_wheel_force");
		auto lateral_force = get_state(curr_knot_prefix + "m1_lateral_force");
		auto steering_angle = get_state(curr_knot_prefix + "m1_steering_angle");
		auto x_accel_constraint = get_param("mass") * get_state(curr_knot_prefix + "base_accel_x") - cos(steering_angle) * wheel_force + sin(steering_angle) * lateral_force;
		auto y_accel_constraint = get_param("mass") * get_state(curr_knot_prefix + "base_accel_y") - sin(steering_angle) * wheel_force - cos(steering_angle) * lateral_force;
		acceleration_dynamics_constraint_vector = vertcat(acceleration_dynamics_constraint_vector, x_accel_constraint);
		acceleration_dynamics_constraint_vector = vertcat(acceleration_dynamics_constraint_vector, y_accel_constraint);
	}

	// Zero Lateral wheel velocity constraint
	auto zero_lateral_wheel_vel_constraint = casadi::Matrix<casadi::SXElem>();
	for(int k = 0; k < NUM_KNOTS; k++){
		std::string curr_knot_prefix = get_knot_prefix(k);
		auto vel_x = get_state(curr_knot_prefix + "base_vel_x");
		auto vel_y = get_state(curr_knot_prefix + "base_vel_y");
		auto steering_angle = get_state(curr_knot_prefix + "m1_steering_angle");
		auto vel_constraint = -vel_x*sin(steering_angle) + vel_y * cos(steering_angle);
		zero_lateral_wheel_vel_constraint = vertcat(zero_lateral_wheel_vel_constraint, vel_constraint);
	}

	// Combine all constraints into single vector
	auto constraints = vertcat(
		integration_constraints_vector,
		initial_state_constraint_vector,
		acceleration_dynamics_constraint_vector,
		zero_lateral_wheel_vel_constraint);

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

		lbx(state_index_map[curr_knot_prefix + "m1_steering_accel"]) = -5000.0;
		ubx(state_index_map[curr_knot_prefix + "m1_steering_accel"]) = 5000.0;

		lbx(state_index_map[curr_knot_prefix + "m1_wheel_force"]) = -20.0;
		ubx(state_index_map[curr_knot_prefix + "m1_wheel_force"]) = 20.0;
	}

	auto lbg = DM::zeros(constraints.size1());
	auto ubg = DM::zeros(constraints.size1());

	for(int i = integration_constraints_vector.size1() + initial_state_constraint_vector.size1() +
	            acceleration_dynamics_constraint_vector.size1(); i < integration_constraints_vector.size1() +
	    initial_state_constraint_vector.size1() +
	    acceleration_dynamics_constraint_vector.size1() + zero_lateral_wheel_vel_constraint.size1(); i++){
		lbg(i) = -0.001;
		ubg(i) = 0.001;
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
		f += 0.01 * 1 / DT * (pow(get_state(curr_knot_prefix + "m1_steering_accel"), 2) + pow(get_state(next_knot_prefix + "m1_steering_accel"), 2));

		// Penalize Velocity Deviation
		f += 500.0 * 1 / DT * (pow(get_param("des_vel_x") - get_state(curr_knot_prefix + "base_vel_x"), 2) + pow(get_param("des_vel_x") - get_state(next_knot_prefix + "base_vel_x"), 2));
		f += 500.0 * 1 / DT * (pow(get_param("des_vel_y") - get_state(curr_knot_prefix + "base_vel_y"), 2) + pow(get_param("des_vel_y") - get_state(next_knot_prefix + "base_vel_y"), 2));
		f += 500.0 * 1 / DT * (pow(get_param("des_vel_theta") - get_state(curr_knot_prefix + "base_vel_theta"), 2) + pow(get_param("des_vel_theta") - get_state(next_knot_prefix + "base_vel_theta"), 2));

		// Minimize jerk via finite difference
		f += 1 / DT * (pow(get_state(next_knot_prefix + "base_accel_x") - get_state(curr_knot_prefix + "base_accel_x"), 2));
		f += 1 / DT * (pow(get_state(next_knot_prefix + "base_accel_y") - get_state(curr_knot_prefix + "base_accel_y"), 2));
		f += 1 / DT * (pow(get_state(next_knot_prefix + "base_accel_theta") - get_state(curr_knot_prefix + "base_accel_theta"), 2));
		f += 0.01 * 1 / DT * (pow(get_state(next_knot_prefix + "m1_steering_accel") - get_state(curr_knot_prefix + "m1_steering_accel"), 2));
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
		// {"ipopt.max_iter", 5}
	};
	auto solver = nlpsol("example_trajectory_optimization", "ipopt", nlp_config, solver_config);

	//////////////////////
	///// Test Solve /////
	//////////////////////
	std::map<std::string, DM> solver_args, res;
	solver_args["lbx"] = lbx;
	solver_args["ubx"] = ubx;
	solver_args["lbg"] = lbg;
	solver_args["ubg"] = ubg;
	solver_args["x0"] = DM::zeros(NUM_OPT_VARS);
	solver_args["p"] = {10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5, 1.5, 0.1, 3.14159 / 4.0, 0.0};
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

	//////////////////////////////
	///// Animate Trajectory /////
	//////////////////////////////

	std::vector<double> x_pos;
	std::vector<double> y_pos;
	std::vector<double> x_vel;
	std::vector<double> y_vel;
	std::vector<double> x_angle_components;
	std::vector<double> y_angle_components;
	std::vector<double> wheel_force_x_components;
	std::vector<double> wheel_force_y_components;
	std::vector<double> lateral_force_x_components;
	std::vector<double> lateral_force_y_components;
	int resolution = 1;

	std::cout << NUM_KNOTS << std::endl;

	x_pos.reserve(NUM_KNOTS / resolution);
	y_pos.reserve(NUM_KNOTS / resolution);
	x_vel.reserve(NUM_KNOTS / resolution);
	y_vel.reserve(NUM_KNOTS / resolution);
	x_angle_components.reserve(NUM_KNOTS / resolution);
	y_angle_components.reserve(NUM_KNOTS / resolution);
	wheel_force_x_components.reserve(NUM_KNOTS / resolution);
	wheel_force_y_components.reserve(NUM_KNOTS / resolution);
	lateral_force_x_components.reserve(NUM_KNOTS / resolution);
	lateral_force_y_components.reserve(NUM_KNOTS / resolution);

	for(int k = 0; k < NUM_KNOTS / resolution; k++){
		auto index = resolution * k;
		x_pos.push_back(state_solution_map["base_pose_x"][index]);
		y_pos.push_back(state_solution_map["base_pose_y"][index]);
		x_vel.push_back(state_solution_map["base_vel_x"][index]);
		y_vel.push_back(state_solution_map["base_vel_y"][index]);

		x_angle_components.push_back(cos(state_solution_map["base_pose_theta"][index]));
		y_angle_components.push_back(sin(state_solution_map["base_pose_theta"][index]));

		auto angle = state_solution_map["m1_steering_angle"][index];
		auto wheel_force = state_solution_map["m1_wheel_force"][index];
		auto lateral_force = state_solution_map["m1_lateral_force"][index];
		wheel_force_x_components.push_back(wheel_force * cos(angle));
		wheel_force_y_components.push_back(wheel_force * sin(angle));
		lateral_force_x_components.push_back(-lateral_force * sin(angle));
		lateral_force_y_components.push_back(lateral_force * cos(angle));
	}

	const double wheel_base_width = 16.5 * ghost_util::INCHES_TO_METERS;
	const double drivetrain_dimension = 12.5 * ghost_util::INCHES_TO_METERS;
	const double wheel_diam = 2.75 * ghost_util::INCHES_TO_METERS;
	const double wheel_width = 1 * ghost_util::INCHES_TO_METERS;

	// Allocate points for chassis
	Eigen::Vector2d chassis_top_left;
	Eigen::Vector2d chassis_top_right;
	Eigen::Vector2d chassis_bottom_right;
	Eigen::Vector2d chassis_bottom_left;

	// Allocate points for wheel
	Eigen::Vector2d m1_top_left;
	Eigen::Vector2d m1_top_right;
	Eigen::Vector2d m1_bottom_right;
	Eigen::Vector2d m1_bottom_left;
	Eigen::Vector2d m1_steering_center;

	plt::figure();
	for(int k = 0; k < NUM_KNOTS; k++){
		plt::clf();
		double x = state_solution_map["base_pose_x"][k];
		double y = state_solution_map["base_pose_y"][k];
		double theta = state_solution_map["base_pose_theta"][k];
		double x_vel = state_solution_map["base_vel_x"][k];
		double y_vel = state_solution_map["base_vel_y"][k];
		double m1_steering_angle = state_solution_map["m1_steering_angle"][k];

		// Plot Chassis
		auto rotate_theta = Eigen::Rotation2D<double>(theta).toRotationMatrix();

		chassis_top_left = rotate_theta * Eigen::Vector2d(-wheel_base_width / 2,wheel_base_width / 2);
		chassis_top_right = rotate_theta * Eigen::Vector2d(wheel_base_width / 2,wheel_base_width / 2);
		chassis_bottom_right = rotate_theta * Eigen::Vector2d(wheel_base_width / 2, -wheel_base_width / 2);
		chassis_bottom_left = rotate_theta * Eigen::Vector2d(-wheel_base_width / 2, -wheel_base_width / 2);

		std::vector<double> x_points{
			x + chassis_top_left.x(),
			x + chassis_top_right.x(),
			x + chassis_bottom_right.x(),
			x + chassis_bottom_left.x(),
			x + chassis_top_left.x()
		};
		std::vector<double> y_points{
			y + chassis_top_left.y(),
			y + chassis_top_right.y(),
			y + chassis_bottom_right.y(),
			y + chassis_bottom_left.y(),
			y + chassis_top_left.y()
		};

		plt::axis("scaled");
		plt::xlim(-48.0 * ghost_util::INCHES_TO_METERS, 48.0 * ghost_util::INCHES_TO_METERS);
		plt::ylim(-48.0 * ghost_util::INCHES_TO_METERS, 48.0 * ghost_util::INCHES_TO_METERS);

		// Plot Chassis
		plt::plot(
			x_points,
			y_points,
			std::map<std::string, std::string>{{"color", "black"}});

		// Plot Base Linear Velocity
		plt::plot(
			std::vector<double>{x, x + x_vel / 100.0},
			std::vector<double>{y, y + y_vel / 100.0},
			std::map<std::string, std::string>{{"color", "red"}});

		// Plot Base Link Center Point
		plt::scatter(
			std::vector<double>{x},
			std::vector<double>{y});

		// Plot Wheel
		auto rotate_steering = Eigen::Rotation2D<double>(m1_steering_angle).toRotationMatrix();
		m1_steering_center = Eigen::Vector2d(x, y) + rotate_theta * Eigen::Vector2d(drivetrain_dimension / 2,drivetrain_dimension / 2);
		m1_top_left = m1_steering_center + rotate_steering * Eigen::Vector2d(-wheel_diam / 2,wheel_width / 2);
		m1_top_right = m1_steering_center + rotate_steering * Eigen::Vector2d(wheel_diam / 2,wheel_width / 2);
		m1_bottom_right = m1_steering_center + rotate_steering * Eigen::Vector2d(wheel_diam / 2, -wheel_width / 2);
		m1_bottom_left = m1_steering_center + rotate_steering * Eigen::Vector2d(-wheel_diam / 2, -wheel_width / 2);

		std::vector<double> m1_x_points{
			m1_top_left.x(),
			m1_top_right.x(),
			m1_bottom_right.x(),
			m1_bottom_left.x(),
			m1_top_left.x()
		};
		std::vector<double> m1_y_points{
			m1_top_left.y(),
			m1_top_right.y(),
			m1_bottom_right.y(),
			m1_bottom_left.y(),
			m1_top_left.y()
		};

		// Plot Module 1
		plt::plot(
			m1_x_points,
			m1_y_points,
			std::map<std::string, std::string>{{"color", "black"}});

		plt::plot(
			std::vector<double>{m1_steering_center.x(), m1_steering_center.x() + wheel_force_x_components[k] / 100.0},
			std::vector<double>{m1_steering_center.y(), m1_steering_center.y() + wheel_force_y_components[k] / 100.0},
			std::map<std::string, std::string>{{"color", "red"}});

		plt::plot(
			std::vector<double>{m1_steering_center.x(), m1_steering_center.x() + lateral_force_x_components[k] / 100.0},
			std::vector<double>{m1_steering_center.y(), m1_steering_center.y() + lateral_force_y_components[k] / 100.0},
			std::map<std::string, std::string>{{"color", "blue"}});

		plt::pause(DT);
	}

	plt::figure();
	plt::quiver(x_pos, y_pos, x_vel, y_vel, std::map<std::string, std::string>{{"color", "black"}});
	plt::quiver(x_pos, y_pos, wheel_force_x_components, wheel_force_y_components, std::map<std::string, std::string>{{"color", "red"}});
	plt::quiver(x_pos, y_pos, lateral_force_x_components, lateral_force_y_components, std::map<std::string, std::string>{{"color", "blue"}});
	plt::xlim(-1.0, 1.0);
	plt::ylim(-1.0, 1.0);

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

	plt::figure();
	plt::suptitle("Steering");
	plt::subplot(3, 1, 1);
	plt::plot(time_vector, state_solution_map["m1_steering_angle"]);
	plt::subplot(3, 1, 2);
	plt::plot(time_vector, state_solution_map["m1_steering_vel"]);
	plt::subplot(3, 1, 3);
	plt::plot(time_vector, state_solution_map["m1_steering_accel"]);

	plt::figure();
	plt::suptitle("Forces");
	plt::subplot(2, 1, 1);
	plt::plot(time_vector, state_solution_map["m1_wheel_force"]);
	plt::subplot(2, 1, 2);
	plt::plot(time_vector, state_solution_map["m1_lateral_force"]);

	plt::show();

	return 0;
}