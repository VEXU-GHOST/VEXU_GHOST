
#include <iostream>
#include <unordered_map>
#include "eigen3/Eigen/Geometry"
#include <casadi/casadi.hpp>

using namespace casadi;

int main(int argc, char *argv[])
{
    // Input Configuration
    float TIME_HORIZON = 1.0;
    float DT = 0.5;
    int NUM_SWERVE_MODULES = 4;

    std::vector<std::string> base_state_name_vector = {
        "base_pose_x",
        "base_pose_y",
        "base_pose_theta",
        "base_vel_x",
        "base_vel_y",
        "base_vel_theta",
        "base_accel_x",
        "base_accel_y",
        "base_accel_theta"};

    std::vector<std::string> joint_state_name_vector = {
        "steering_angle_i",
        "steering_angle_j",
        "steering_vel",
        "steering_accel",
        "wheel_vel",
        "wheel_accel",
        "voltage_1",
        "voltage_2",
    };

    int num_knots = int(TIME_HORIZON / DT) + 1;
    int num_states = (9 + NUM_SWERVE_MODULES * 8);
    int num_opt_vars = num_states * num_knots;

    // Populate time vector
    Eigen::VectorXf time_vector(num_knots);
    for (int i = 0; i < num_knots; i++)
    {
        time_vector[i] = i * DT;
    }

    // Initialize map to hold opt variable indexes
    std::unordered_map<std::string, int> opt_var_index_map;

    // Populate empty vector to hold optimization variables
    auto opt_var_vector = casadi::SX::zeros(num_opt_vars);

    // Populate state and input variables
    int state_index = 0;
    for (int i = 0; i < num_knots; i++)
    {
        std::string knot_prefix = "k" + std::to_string(i) + "_";
        for (auto name : base_state_name_vector)
        {
            opt_var_index_map[knot_prefix + name] = state_index;
            opt_var_vector(state_index) = SX::sym(knot_prefix + name);
            state_index++;
        }

        for (int m = 1; m < NUM_SWERVE_MODULES + 1; m++)
        {

            std::string module_prefix = knot_prefix + "m" + std::to_string(m) + "_";
            for (auto name : joint_state_name_vector)
            {
                opt_var_index_map[module_prefix + name] = state_index;
                opt_var_vector(state_index) = SX::sym(module_prefix + name);
                state_index++;
            }
        }
    }

    // Shorthand function to get symbolic state variable by name
    auto get_state_by_name = [&opt_var_vector, &opt_var_index_map](std::string name){
        return opt_var_vector(opt_var_index_map.at(name));
    };

    // EULER INTEGRATION CONSTRAINTS
    // List pairs of base state and derivative state
    std::vector<std::pair<std::string, std::string>> euler_integration_state_names = {
        std::pair<std::string, std::string>{"base_pose_x", "base_vel_x"},
        std::pair<std::string, std::string>{"base_pose_y", "base_vel_y"},
        std::pair<std::string, std::string>{"base_vel_x", "base_accel_x"},
        std::pair<std::string, std::string>{"base_vel_y", "base_accel_y"},
        std::pair<std::string, std::string>{"base_vel_theta", "base_accel_theta"},
    };

    // Add joint states for each swerve module to the integration states pairs
    for (int m = 1; m < NUM_SWERVE_MODULES + 1; m++)
    {
        std::string module_prefix = "m" + std::to_string(m) + "_";
        euler_integration_state_names.push_back(std::pair<std::string, std::string>{module_prefix + "steering_vel", module_prefix + "steering_accel"});
        euler_integration_state_names.push_back(std::pair<std::string, std::string>{module_prefix + "wheel_vel", module_prefix + "wheel_accel"});
    }

    // Populate euler integration constraints for state vector
    auto euler_integration_constraints_vector = SX::zeros(euler_integration_state_names.size() * (num_knots - 1));
    int integration_constraint_index = 0;
    for (int k = 0; k < num_knots - 1; k++)
    {
        std::string curr_knot_prefix = "k" + std::to_string(k) + "_";
        std::string next_knot_prefix = "k" + std::to_string(k+1) + "_";

        for(auto & pair : euler_integration_state_names){
            auto state_1 = curr_knot_prefix + pair.first;
            auto state_2 = next_knot_prefix + pair.first;
            auto deriv_1 = curr_knot_prefix + pair.second;
            euler_integration_constraints_vector(integration_constraint_index) = (get_state_by_name(state_2) - get_state_by_name(state_1)) / DT - get_state_by_name(deriv_1);
            integration_constraint_index++;
        }
    }

    for(int i = 0; i < euler_integration_constraints_vector.size1(); i++){
        std::cout << euler_integration_constraints_vector(i) << std::endl;
    }

    return 0;
}