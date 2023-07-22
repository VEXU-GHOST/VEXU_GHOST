#pragma once

#include <string>
#include <unordered_map>
#include "yaml-cpp/yaml.h"
#include <casadi/casadi.hpp>

namespace ghost_control
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
        };

        // Shorthand to get symbolic parameter by name
        casadi::SX getParam(std::string name)
        {
            return param_vector_(param_index_map_.at(name));
        };

        // Shorthand to get knot string prefix from knotpoint index
        std::string getKnotPrefix(int i)
        {
            return "k" + std::to_string(i) + "_";
        };

        const std::vector<double>& getTimeVector(){
            return time_vector_;
        }

    private:
        YAML::Node config_;

        std::vector<double> time_vector_;
        float time_horizon_;
        float dt_;

        int num_knots_;
        int num_opt_vars_;

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

} // namespace ghost_control