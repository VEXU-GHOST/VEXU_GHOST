
#include <iostream>
#include <unordered_map>
#include "eigen3/Eigen/Geometry"
#include <casadi/casadi.hpp>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
using namespace casadi;

/*
This file is a working example for how to generate optimization problems using the CasADi symbolic toolbox.

Here, we use a 1D point mass model, with equality constraints for the initial and final states and a quadratic acceleration cost.
The Problem Formulation is then exported to C++ code and can be called externally.
*/
int main(int argc, char *argv[])
{
    // Input Configuration (defined when code is generated offline)
    float TIME_HORIZON = 1.0;
    float DT = 0.01;

    // 2D Point Mass model
    std::vector<std::string> state_names = {
        "base_pose_x",
        "base_vel_x",
        "base_accel_x"};

    std::vector<std::string> param_names = {
        "final_pose_x"};

    int num_knots = int(TIME_HORIZON / DT) + 1;
    int num_states = state_names.size();
    int num_opt_vars = num_states * num_knots;

    // Initialize containers for optimization variables
    std::unordered_map<std::string, int> state_index_map;
    auto state_vector = casadi::SX::zeros(num_opt_vars);

    std::unordered_map<std::string, int> param_index_map;
    auto param_vector = casadi::SX::zeros(param_names.size());

    // Shorthand function to get symbolic state variable by name
    auto get_state = [&state_vector, &state_index_map](std::string name)
    {
        return state_vector(state_index_map.at(name));
    };

    auto get_param = [&param_vector, &param_index_map](std::string name)
    {
        return param_vector(param_index_map.at(name));
    };

    auto get_knot_prefix = [](int i)
    {
        return "k" + std::to_string(i) + "_";
    };

    //////////////////////////////////////////////
    ///// Initialize Time, State, and Inputs /////
    //////////////////////////////////////////////
    // Populate time vector
    std::vector<double> time_vector(num_knots);
    for (int i = 0; i < num_knots; i++)
    {
        time_vector[i] = i * DT;
    }

    // Populate state and input variables
    int state_index = 0;
    for (int i = 0; i < num_knots; i++)
    {
        std::string knot_prefix = get_knot_prefix(i);
        for (auto name : state_names)
        {
            state_index_map[knot_prefix + name] = state_index;
            state_vector(state_index) = SX::sym(knot_prefix + name);
            state_index++;
        }
    }

    int param_index = 0;
    for (auto &param_name : param_names)
    {
        param_index_map[param_name] = param_index;
        param_vector(param_index) = SX::sym(param_name);
        param_index++;
    }

    /////////////////////////////////
    ///// Formulate Constraints /////
    /////////////////////////////////
    // List pairs of base state and derivative state
    std::vector<std::pair<std::string, std::string>> euler_integration_state_names = {
        std::pair<std::string, std::string>{"base_pose_x", "base_vel_x"},
        std::pair<std::string, std::string>{"base_vel_x", "base_accel_x"},
    };

    // Populate trapezoidal integration constraints for state vector
    auto integration_constraints_vector = SX::zeros(euler_integration_state_names.size() * (num_knots - 1));
    int integration_constraint_index = 0;
    for (int k = 0; k < num_knots - 1; k++)
    {
        std::string curr_knot_prefix = get_knot_prefix(k);
        std::string next_knot_prefix = get_knot_prefix(k + 1);

        for (auto &pair : euler_integration_state_names)
        {
            auto x0 = curr_knot_prefix + pair.first;
            auto x1 = next_knot_prefix + pair.first;
            auto dx0 = curr_knot_prefix + pair.second;
            auto dx1 = next_knot_prefix + pair.second;

            // X1 - X0 = 1/2 * DT * (dX1 + dX0)
            integration_constraints_vector(integration_constraint_index) = 2*(get_state(x1) - get_state(x0)) / DT - get_state(dx1) - get_state(dx0);
            integration_constraint_index++;
        }
    }

    // Add equality constraints for initial and final states
    auto initial_state_constraint_vector = vertcat(
        get_state("k0_base_pose_x") - 1,
        -get_state("k0_base_pose_x") + 1,
        get_state("k0_base_vel_x"),
        -get_state("k0_base_vel_x"));

    std::string final_knot_prefix = get_knot_prefix(num_knots - 1);
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

    ///////////////////////////////////
    ///// Formulate Cost Function /////
    ///////////////////////////////////
    // Quadratic cost on all accelerations
    auto f = SX::zeros(1);

    // Apply Quadratic costs
    for (int k = 0; k < num_knots-1; k++)
    {
        std::string curr_knot_prefix = "k" + std::to_string(k) + "_";
        std::string next_knot_prefix = "k" + std::to_string(k+1) + "_";

        // Normalize base acceleration (essentially averaging adjacent values via trapezoidal quadrature)
        f += 1/DT * (pow(get_state(curr_knot_prefix + "base_accel_x"), 2) + pow(get_state(next_knot_prefix + "base_accel_x"), 2));

        // Minimize jerk via finite difference
        f += 1/DT * (pow(get_state(next_knot_prefix + "base_accel_x") - get_state(curr_knot_prefix + "base_accel_x"), 2));
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
    solver_args["lbx"] = DM::ones(num_opt_vars) * -DM::inf();
    solver_args["ubx"] = DM::ones(num_opt_vars) * DM::inf();
    solver_args["lbg"] = 0;
    solver_args["ubg"] = 0;
    solver_args["x0"] = DM::ones(num_opt_vars);
    solver_args["p"] = {5.0};
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
    for (auto &name : state_names)
    {
        state_solution_map[name] = std::vector<double>(num_knots);
        for (int k = 0; k < num_knots; k++)
        {
            std::string knot_prefix = get_knot_prefix(k);
            state_solution_map[name][k] = raw_solution_vector[state_index_map.at(knot_prefix + name)];
        }
    }

    plt::figure();
    plt::subplot(3, 1, 1);
    plt::plot(time_vector, state_solution_map["base_pose_x"]);
    plt::subplot(3, 1, 2);
    plt::plot(time_vector, state_solution_map["base_vel_x"]);
    plt::subplot(3, 1, 3);
    plt::plot(time_vector, state_solution_map["base_accel_x"]);
    plt::show();

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