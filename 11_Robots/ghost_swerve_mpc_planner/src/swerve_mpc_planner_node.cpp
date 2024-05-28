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

#include <iostream>
#include <unordered_map>
#include <atomic>
#include <mutex>
#include <thread>
#include <csignal>

#include "eigen3/Eigen/Geometry"
#include <casadi/casadi.hpp>

#include <ghost_util/unit_conversion_utils.hpp>

#include <ghost_msgs/msg/labeled_vector.hpp>
#include <ghost_msgs/msg/labeled_vector_map.hpp>

#include <ghost_ros_interfaces/msg_helpers/msg_helpers.hpp>
#include <rclcpp/rclcpp.hpp>

#include "matplotlibcpp.h"
#include "yaml.h"

#include "ghost_planners/ipopt_iteration_callback.hpp"

using namespace std::chrono_literals;
using namespace casadi;
namespace plt = matplotlibcpp;
using ghost_planners::IterationCallback;
using ghost_planners::Trajectory;

std::shared_ptr<std::atomic_bool> EXIT_GLOBAL_PTR = std::make_shared<std::atomic_bool>(false);

void siginthandler(int param)
{
  std::cout << "Ctr-C Received, Aborting..." << std::endl;
  *EXIT_GLOBAL_PTR = true;
}

class SwerveMPCPlanner : public rclcpp::Node
{
public:
  struct Config
  {
    double dt;
    double time_horizon;
    int num_swerve_modules;
    double chassis_width;
    double wheel_base_width;
    double wheel_width;
    double wheel_radius;
    double robot_mass;
    double steering_inertia;
    double wheel_constraint_tolerance;
  };

  SwerveMPCPlanner()
  : rclcpp::Node("swerve_mpc_planner_node")
  {
    // Load and validate config params
    loadConfig();
    validateConfig();

    // Set state and parameter names
    generateStateNames();
    generateParameterNames();

    // create empty vectors for states, parameters, cost, constraints, and problem bounds
    populateContainers();

    // Set costs, constraints, and state bounds for swerve drive optimization
    addCosts();
    addConstraints();
    setStateBounds();

    // Init ROS interfaces
    initROS();

    initSolver();
  }

  void loadConfig()
  {
    declare_parameter("time_horizon", 0.0);
    config_.time_horizon = get_parameter("time_horizon").as_double();

    declare_parameter("dt", 0.0);
    config_.dt = get_parameter("dt").as_double();

    declare_parameter("num_swerve_modules", 0);
    config_.num_swerve_modules = get_parameter("num_swerve_modules").as_int();

    declare_parameter("chassis_width", 0.0);
    config_.chassis_width = get_parameter("chassis_width").as_double() * I2M;

    declare_parameter("wheel_base_width", 0.0);
    config_.wheel_base_width = get_parameter("wheel_base_width").as_double() * I2M;

    declare_parameter("wheel_width", 0.0);
    config_.wheel_width = get_parameter("wheel_width").as_double() * I2M;

    declare_parameter("wheel_diam", 0.0);
    config_.wheel_radius = get_parameter("wheel_diam").as_double() * I2M / 2.0;

    declare_parameter("robot_mass", 0.0);
    config_.robot_mass = get_parameter("robot_mass").as_double();

    declare_parameter("steering_inertia", 0.0);
    config_.steering_inertia = get_parameter("steering_inertia").as_double();

    declare_parameter("wheel_constraint_tolerance", 0.0);
    config_.wheel_constraint_tolerance = get_parameter("wheel_constraint_tolerance").as_double();

    declare_parameter("default_state_weights", std::vector<double>{});
    auto default_state_weights = get_parameter("default_state_weights").as_double_array();

    declare_parameter("default_joint_weights", std::vector<double>{});
    auto default_joint_weights = get_parameter("default_joint_weights").as_double_array();

    // Weights
    weights_ = std::vector<double>{};
    weights_.insert(weights_.end(), default_state_weights.begin(), default_state_weights.end());
    weights_.insert(weights_.end(), default_joint_weights.begin(), default_joint_weights.end());
    weights_.insert(weights_.end(), default_joint_weights.begin(), default_joint_weights.end());
    weights_.insert(weights_.end(), default_joint_weights.begin(), default_joint_weights.end());
    weights_.insert(weights_.end(), default_joint_weights.begin(), default_joint_weights.end());

    // Module Positions
    module_positions_ = std::vector<Eigen::Vector2d>{
      Eigen::Vector2d(config_.wheel_base_width / 2, config_.wheel_base_width / 2),
      Eigen::Vector2d(-config_.wheel_base_width / 2, config_.wheel_base_width / 2),
      Eigen::Vector2d(-config_.wheel_base_width / 2, -config_.wheel_base_width / 2),
      Eigen::Vector2d(config_.wheel_base_width / 2, -config_.wheel_base_width / 2)
    };

    num_knots_ = int(config_.time_horizon / config_.dt) + 1;
  }

  void validateConfig()
  {
    std::unordered_map<std::string, double> double_params{
      {"time_horizon", config_.time_horizon},
      {"dt", config_.dt},
      {"chassis_width", config_.chassis_width},
      {"wheel_base_width", config_.wheel_base_width},
      {"wheel_width", config_.wheel_width},
      {"wheel_diam", config_.wheel_radius * 2.0},
      {"robot_mass", config_.robot_mass},
      {"steering_inertia", config_.steering_inertia},
      {"wheel_constraint_tolerance", config_.wheel_constraint_tolerance},
    };

    for (const auto & [key, val] : double_params) {
      if (val <= 0) {
        std::string err_string =
          std::string("[SwerveMPCPlanner::validateConfig] Error: ") + key +
          " must be non-zero and positive!";
        throw std::runtime_error(err_string);
      }
    }

    std::unordered_map<std::string, int> int_params{
      {"num_swerve_modules", config_.num_swerve_modules}};

    for (const auto & [key, val] : int_params) {
      if (val <= 0) {
        std::string err_string =
          std::string("[SwerveMPCPlanner::validateConfig] Error: ") + key +
          " must be non-zero and positive!";
        throw std::runtime_error(err_string);
      }
    }
  }

  void initROS()
  {
    trajectory_publisher_ = create_publisher<ghost_msgs::msg::LabeledVectorMap>(
      "/trajectory/swerve_mpc_trajectory", 10);
  }

  void generateStateNames()
  {
    // Populate state vector
    state_names_ = std::vector<std::string> {
      "base_pose_x",
      "base_pose_y",
      "base_pose_theta",
      "base_vel_x",
      "base_vel_y",
      "base_vel_theta",
      "base_accel_x",
      "base_accel_y",
      "base_accel_theta"};

    std::vector<std::string> joint_state_names = {
      "steering_angle",
      "steering_vel",
      "steering_accel",
      "wheel_vel",
      "wheel_torque",
      "lateral_force"
      // "voltage_1",
      // "voltage_2",
    };

    // Add joint states to state name vector
    for (int m = 1; m < config_.num_swerve_modules + 1; m++) {
      std::string module_prefix = "m" + std::to_string(m) + "_";
      for (auto name : joint_state_names) {
        state_names_.push_back(module_prefix + name);
      }
    }

    // Set total number of optimization variables
    num_opt_vars_ = state_names_.size() * num_knots_;

    // Error Checking
    if (weights_.size() != state_names_.size()) {
      throw std::runtime_error(
              "[SwerveMPCPlanner::populateContainers] Error: weights_ must be the same length as the state vector!");
    }
  }

  void generateParameterNames()
  {
    // Add initial state params
    param_names_ = {
      "init_pose_x",
      "init_pose_y",
      "init_pose_theta",
      "init_vel_x",
      "init_vel_y",
      "init_vel_theta",
    };

    for (int m = 1; m < config_.num_swerve_modules + 1; m++) {
      param_names_.push_back(std::string("init_m") + std::to_string(m) + "_steering_angle");
      param_names_.push_back(std::string("init_m") + std::to_string(m) + "_steering_vel");
      param_names_.push_back(std::string("init_m") + std::to_string(m) + "_wheel_vel");
    }
  }

  void populateContainers()
  {
    cost_ = SX::zeros(1);
    lbx_ = DM::ones(num_opt_vars_) * -DM::inf();
    ubx_ = DM::ones(num_opt_vars_) * DM::inf();
    constraints_ = casadi::Matrix<casadi::SXElem>();
    lbg_ = DM::zeros(0);
    ubg_ = DM::zeros(0);

    // Populate time vector
    time_vector_ = std::vector<double>(num_knots_);
    for (int i = 0; i < num_knots_; i++) {
      time_vector_[i] = i * config_.dt;
    }

    // Generate optimization variables and populate state_vector and state_index_map
    state_vector_ = casadi::SX::zeros(num_opt_vars_);
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
    param_vector_ = casadi::SX::zeros(param_names_.size());
    int param_index = 0;
    for (auto & param_name : param_names_) {
      param_index_map_[param_name] = param_index;
      param_vector_(param_index) = SX::sym(param_name);
      param_index++;
    }

    int weight_index = 0;
    for (auto & name : state_names_) {
      weight_index_map_[name] = weight_index;
      weight_index++;
    }

    // Instantiate reference trajectory
    std::vector<std::string> reference_names;
    for (const auto & name : state_names_) {
      reference_names.push_back(name + "_ref");
    }
    reference_trajectory_ptr_ = std::make_shared<Trajectory>(reference_names);
  }

  double getWeight(std::string name)
  {
    if (weight_index_map_.count(name) != 0) {
      return weights_[weight_index_map_.at(name)];
    } else {
      throw std::runtime_error("[getWeight] No state with name " + name);
    }
  }

  // Shorthand to get symbolic state variable by name
  casadi::Matrix<casadi::SXElem> getState(std::string name, int k)
  {
    auto state_name = "k" + std::to_string(k) + "_" + name;
    if (state_index_map_.count(state_name) != 0) {
      return state_vector_(state_index_map_.at(state_name));
    } else {
      throw std::runtime_error("[getState] No state with name " + state_name);
    }
  }

  // Shorthand to get symbolic state variable by name
  casadi::Matrix<casadi::SXElem> getState(std::string name)
  {
    if (state_index_map_.count(name) != 0) {
      return state_vector_(state_index_map_.at(name));
    } else {
      throw std::runtime_error("[getState] No state with name " + name);
    }
  }

  casadi::Matrix<casadi::SXElem> getParam(std::string name)
  {
    if (param_index_map_.count(name) != 0) {
      return param_vector_(param_index_map_.at(name));
    } else {
      throw std::runtime_error("[getParam] No param with name " + name);
    }
  }

  static std::string getKnotPrefix(int i)
  {
    return "k" + std::to_string(i) + "_";
  }

  casadi::Matrix<casadi::SXElem> getQuadraticTrackingCost(std::string state, int k)
  {
    std::string kt1_ = "k" + std::to_string(k) + "_";
    std::string kt2_ = "k" + std::to_string(k + 1) + "_";

    auto s1 = getState(kt1_ + state);
    auto s2 = getState(kt2_ + state);

    double r1 = reference_trajectory_ptr_->getState(state + "_ref", config_.dt * k);
    double r2 = reference_trajectory_ptr_->getState(state + "_ref", config_.dt * (k + 1));
    double weight = getWeight(state);

    return weight * 0.5 * config_.dt * (pow(s1 - r1, 2) + pow(s2 - r2, 2));
  }

  casadi::Matrix<casadi::SXElem> getJerkCost(std::string state, int k, double cost)
  {
    std::string kt1_ = "k" + std::to_string(k + 1) + "_";
    std::string kt2_ = "k" + std::to_string(k) + "_";

    auto s1 = getState(kt1_ + state);
    auto s2 = getState(kt2_ + state);
    auto jerk = (s1 - s2) / config_.dt;

    return cost * pow(jerk, 2);
  }

  void rotateVectorSymbolic(
    const Matrix<SXElem> & angle,
    const Matrix<SXElem> & x_in,
    const Matrix<SXElem> & y_in,
    Matrix<SXElem> & x_out,
    Matrix<SXElem> & y_out)
  {
    x_out = x_in * cos(angle) - y_in * sin(angle);
    y_out = x_in * sin(angle) + y_in * cos(angle);
  }

  void rotateVectorDouble(

    const Matrix<SXElem> & angle,
    const double & x_in,
    const double & y_in,
    Matrix<SXElem> & x_out,
    Matrix<SXElem> & y_out)
  {
    x_out = x_in * cos(angle) - y_in * sin(angle);
    y_out = x_in * sin(angle) + y_in * cos(angle);
  }

  std::unordered_map<std::string, std::vector<double>> generateTrajectoryMap(
    const std::vector<double> & solution_vector)
  {
    std::unordered_map<std::string, std::vector<double>> solution_map;

    for (auto & name : state_names_) {
      // Allocate timeseries vector for each state/input variable
      solution_map[name] = std::vector<double>(num_knots_);

      // Iterate through timeseries and add final state values to solution vector
      for (int k = 0; k < num_knots_; k++) {
        std::string knot_prefix = getKnotPrefix(k);
        solution_map[name][k] = solution_vector[state_index_map_.at(knot_prefix + name)];
      }
    }

    // Allocate timeseries vector for each state/input variable
    solution_map["time"] = std::vector<double>(num_knots_);

    // Iterate through timeseries and add final state values to solution vector
    for (int k = 0; k < num_knots_; k++) {
      solution_map["time"][k] = config_.dt * ((double) k);
    }
    return solution_map;
  }

  void addIntegrationConstraints()
  {
    // List pairs of base state and derivative state
    std::vector<std::pair<std::string, std::string>> euler_integration_state_names = {
      std::pair<std::string, std::string>{"base_pose_x", "base_vel_x"},
      std::pair<std::string, std::string>{"base_pose_y", "base_vel_y"},
      std::pair<std::string, std::string>{"base_pose_theta", "base_vel_theta"},
      std::pair<std::string, std::string>{"base_vel_x", "base_accel_x"},
      std::pair<std::string, std::string>{"base_vel_y", "base_accel_y"},
      std::pair<std::string, std::string>{"base_vel_theta", "base_accel_theta"},
    };

    // Add joint states for each swerve module to the integration states pairs
    for (int m = 1; m < config_.num_swerve_modules + 1; m++) {
      std::string module_prefix = "m" + std::to_string(m) + "_";
      euler_integration_state_names.push_back(
        std::pair<std::string,
        std::string>{module_prefix + "steering_angle",
          module_prefix + "steering_vel"});
      euler_integration_state_names.push_back(
        std::pair<std::string,
        std::string>{module_prefix + "steering_vel",
          module_prefix + "steering_accel"});
    }

    // Populate euler integration constraints for state vector
    for (int k = 0; k < num_knots_ - 1; k++) {
      std::string curr_knot_prefix = getKnotPrefix(k);
      std::string next_knot_prefix = getKnotPrefix(k + 1);

      for (auto & pair : euler_integration_state_names) {
        auto x0 = curr_knot_prefix + pair.first;
        auto x1 = next_knot_prefix + pair.first;
        auto dx0 = curr_knot_prefix + pair.second;
        auto dx1 = next_knot_prefix + pair.second;

        // X1 - X0 = 1/2 * DT * (dX1 + dX0)
        auto c = 2 * (getState(x1) - getState(x0)) / config_.dt - getState(dx1) - getState(dx0);
        constraints_ = vertcat(constraints_, c);
        lbg_ = vertcat(lbg_, DM(0));
        ubg_ = vertcat(ubg_, DM(0));

      }
    }
  }

  void addInitialStateConstraints()
  {
    // Initial State Constraints
    std::vector<std::pair<std::string, std::string>> initial_state_constraint_param_pairs{
      std::pair<std::string, std::string>{"k0_base_pose_x", "init_pose_x"},
      std::pair<std::string, std::string>{"k0_base_pose_y", "init_pose_y"},
      std::pair<std::string, std::string>{"k0_base_pose_theta", "init_pose_theta"},
      std::pair<std::string, std::string>{"k0_base_vel_x", "init_vel_x"},
      std::pair<std::string, std::string>{"k0_base_vel_y", "init_vel_y"},
      std::pair<std::string, std::string>{"k0_base_vel_theta", "init_vel_theta"}
    };

    for (int m = 1; m < config_.num_swerve_modules + 1; m++) {
      initial_state_constraint_param_pairs.push_back(
        std::pair<std::string, std::string>{
        std::string("k0_m") + std::to_string(m) + "_steering_angle",
        std::string("init_m") + std::to_string(m) + "_steering_angle",
      });
      initial_state_constraint_param_pairs.push_back(
        std::pair<std::string, std::string>{
        std::string("k0_m") + std::to_string(m) + "_steering_vel",
        std::string("init_m") + std::to_string(m) + "_steering_vel",
      });
      initial_state_constraint_param_pairs.push_back(
        std::pair<std::string, std::string>{
        std::string("k0_m") + std::to_string(m) + "_wheel_vel",
        std::string("init_m") + std::to_string(m) + "_wheel_vel",
      });
    }

    for (const auto & pair : initial_state_constraint_param_pairs) {
      auto c = getState(pair.first) - getParam(pair.second);
      constraints_ = vertcat(constraints_, c);
      lbg_ = vertcat(lbg_, DM(0));
      ubg_ = vertcat(ubg_, DM(0));
    }
  }

  void addAccelerationDynamicsConstraints()
  {
    // acceleration dynamics constraints
    for (int k = 0; k < num_knots_; k++) {
      std::string kt1_ = getKnotPrefix(k);
      auto x_accel_constraint = config_.robot_mass * getState(kt1_ + "base_accel_x");
      auto y_accel_constraint = config_.robot_mass * getState(kt1_ + "base_accel_y");
      auto theta_accel_constraint = config_.steering_inertia * getState(kt1_ + "base_accel_theta");
      auto theta = getState(kt1_ + "base_pose_theta");

      for (int m = 1; m < config_.num_swerve_modules + 1; m++) {
        std::string kt1_mN_ = kt1_ + "m" + std::to_string(m) + "_";
        auto wheel_force = getState(kt1_mN_ + "wheel_torque") / config_.wheel_radius;
        auto lateral_force = getState(kt1_mN_ + "lateral_force");
        auto world_steering_angle = theta + getState(kt1_mN_ + "steering_angle");

        auto x_force = casadi::Matrix<casadi::SXElem>();
        auto y_force = casadi::Matrix<casadi::SXElem>();
        rotateVectorSymbolic(world_steering_angle, wheel_force, lateral_force, x_force, y_force);
        x_accel_constraint -= x_force;
        y_accel_constraint -= y_force;

        auto mod_offset_x = casadi::Matrix<casadi::SXElem>();
        auto mod_offset_y = casadi::Matrix<casadi::SXElem>();
        rotateVectorDouble(
          theta, module_positions_[m - 1].x(),
          module_positions_[m - 1].y(), mod_offset_x, mod_offset_y);

        auto base_torque = y_force * (mod_offset_x) - x_force * (mod_offset_y);
        theta_accel_constraint -= base_torque;
      }

      constraints_ = vertcat(constraints_, x_accel_constraint);
      constraints_ = vertcat(constraints_, y_accel_constraint);
      constraints_ = vertcat(constraints_, theta_accel_constraint);

      lbg_ = vertcat(lbg_, DM::zeros(3));
      ubg_ = vertcat(ubg_, DM::zeros(3));
    }
  }

  void addNoWheelSlipConstraints()
  {
    // acceleration dynamics constraints
    for (int k = 0; k < num_knots_; k++) {
      std::string kt1_ = getKnotPrefix(k);
      auto theta = getState(kt1_ + "base_pose_theta");
      auto vel_x = getState(kt1_ + "base_vel_x");
      auto vel_y = getState(kt1_ + "base_vel_y");
      auto vel_theta = getState(kt1_ + "base_vel_theta");

      for (int m = 1; m < config_.num_swerve_modules + 1; m++) {
        std::string kt1_mN_ = kt1_ + "m" + std::to_string(m) + "_";
        auto world_steering_angle = theta + getState(kt1_mN_ + "steering_angle");

        auto mod_offset_x = casadi::Matrix<casadi::SXElem>();
        auto mod_offset_y = casadi::Matrix<casadi::SXElem>();
        rotateVectorDouble(
          theta, module_positions_[m - 1].x(),
          module_positions_[m - 1].y(), mod_offset_x, mod_offset_y);

        // Lateral Wheel Velocity Constraint
        auto base_steering_angle = getState(kt1_mN_ + "steering_angle");
        auto tan_vel = vel_theta * sqrt((pow(mod_offset_x, 2) + pow(mod_offset_y, 2)));
        auto r_angle = atan2(module_positions_[m - 1].y(), module_positions_[m - 1].x());                 // base_link
        auto tan_vel_x = tan_vel * -sin(r_angle);
        auto tan_vel_y = tan_vel * cos(r_angle);

        auto world_vel_x = casadi::Matrix<casadi::SXElem>();
        auto world_vel_y = casadi::Matrix<casadi::SXElem>();
        rotateVectorSymbolic(theta, tan_vel_x, tan_vel_y, world_vel_x, world_vel_y);
        world_vel_x += vel_x;
        world_vel_y += vel_y;

        auto forward_velocity = casadi::Matrix<casadi::SXElem>();
        auto lateral_velocity = casadi::Matrix<casadi::SXElem>();
        rotateVectorSymbolic(
          -world_steering_angle, world_vel_x, world_vel_y, forward_velocity,
          lateral_velocity);


        auto wheel_vel = getState(kt1_mN_ + "wheel_vel");
        constraints_ = vertcat(constraints_, lateral_velocity);
        constraints_ = vertcat(constraints_, forward_velocity - wheel_vel * config_.wheel_radius);

        lbg_ = vertcat(lbg_, -DM::ones(2) * config_.wheel_constraint_tolerance);
        ubg_ = vertcat(ubg_, DM::ones(2) * config_.wheel_constraint_tolerance);
      }
    }
  }

  void addConstraints()
  {
    addIntegrationConstraints();
    addInitialStateConstraints();
    addAccelerationDynamicsConstraints();
    addNoWheelSlipConstraints();

    if (lbg_.size1() != ubg_.size1()) {
      throw std::runtime_error(
              "[SwerveMPCPlanner::addConstraints] Error: lbg and ubg are not the same size!");
    }

    if (lbg_.size1() != constraints_.size1()) {
      throw std::runtime_error(
              "[SwerveMPCPlanner::addConstraints] Error: constraint bounds and constraint vector are not the same size!");
    }
  }

  void setStateBounds()
  {
    // Optimization Variables Limits
    lbx_ = DM::ones(num_opt_vars_) * -DM::inf();
    ubx_ = DM::ones(num_opt_vars_) * DM::inf();
    double translation_speed_limit = 3.0;     // 1.389
    double translation_accel_limit = 15.0;     // 5.5755

    for (int k = 0; k < num_knots_; k++) {
      std::string curr_knot_prefix = getKnotPrefix(k);
      lbx_(state_index_map_[curr_knot_prefix + "base_vel_x"]) = -translation_speed_limit;
      ubx_(state_index_map_[curr_knot_prefix + "base_vel_x"]) = translation_speed_limit;
      lbx_(state_index_map_[curr_knot_prefix + "base_vel_y"]) = -translation_speed_limit;
      ubx_(state_index_map_[curr_knot_prefix + "base_vel_y"]) = translation_speed_limit;
      lbx_(state_index_map_[curr_knot_prefix + "base_vel_theta"]) = -7.0305;
      ubx_(state_index_map_[curr_knot_prefix + "base_vel_theta"]) = 7.0305;

      lbx_(state_index_map_[curr_knot_prefix + "base_accel_x"]) = -translation_accel_limit;
      ubx_(state_index_map_[curr_knot_prefix + "base_accel_x"]) = translation_accel_limit;
      lbx_(state_index_map_[curr_knot_prefix + "base_accel_y"]) = -translation_accel_limit;
      ubx_(state_index_map_[curr_knot_prefix + "base_accel_y"]) = translation_accel_limit;
      lbx_(state_index_map_[curr_knot_prefix + "base_accel_theta"]) = -45.97811;
      ubx_(state_index_map_[curr_knot_prefix + "base_accel_theta"]) = 45.97811;

      for (int m = 1; m < config_.num_swerve_modules + 1; m++) {
        std::string module_prefix = curr_knot_prefix + "m" + std::to_string(m) + "_";

        // TODO(maxxwilson): Add steering speed limits, and wheel velocity limits

        lbx_(state_index_map_[module_prefix + "steering_accel"]) = -5000.0;
        ubx_(state_index_map_[module_prefix + "steering_accel"]) = 5000.0;

        lbx_(state_index_map_[module_prefix + "wheel_torque"]) = -50.0 * config_.wheel_radius;
        ubx_(state_index_map_[module_prefix + "wheel_torque"]) = 50.0 * config_.wheel_radius;

        lbx_(state_index_map_[module_prefix + "lateral_force"]) = -config_.robot_mass * 9.81 * 1.25;
        ubx_(state_index_map_[module_prefix + "lateral_force"]) = config_.robot_mass * 9.81 * 1.25;
      }
    }
  }

  void addCosts()
  {
    // Apply Quadratic costs
    for (int k = 0; k < num_knots_ - 1; k++) {
      std::string kt1_ = "k" + std::to_string(k) + "_";
      std::string kt2_ = "k" + std::to_string(k + 1) + "_";

      // Regularize base acceleration (essentially averaging adjacent values via trapezoidal quadrature)
      cost_ += getQuadraticTrackingCost("base_accel_x", k);
      cost_ += getQuadraticTrackingCost("base_accel_y", k);
      cost_ += getQuadraticTrackingCost("base_accel_theta", k);

      // Regularize Base Jerk
      cost_ += getJerkCost("base_accel_x", k, 0.01);
      cost_ += getJerkCost("base_accel_y", k, 0.01);
      cost_ += getJerkCost("base_accel_theta", k, 0.01);

      // Penalize Velocity Deviation
      cost_ + getQuadraticTrackingCost("base_vel_x", k);
      cost_ + getQuadraticTrackingCost("base_vel_y", k);
      cost_ + getQuadraticTrackingCost("base_vel_theta", k);

      for (int m = 1; m < config_.num_swerve_modules + 1; m++) {
        std::string mN_ = "m" + std::to_string(m) + "_";

        // Small Regularization
        cost_ += getQuadraticTrackingCost(mN_ + "steering_accel", k);
        cost_ += getQuadraticTrackingCost(mN_ + "steering_vel", k);
        cost_ += getQuadraticTrackingCost(mN_ + "wheel_vel", k);
        cost_ += getQuadraticTrackingCost(mN_ + "wheel_torque", k);
        cost_ += getQuadraticTrackingCost(mN_ + "lateral_force", k);

        cost_ += getJerkCost(mN_ + "lateral_force", k, 0.001);
        cost_ += getJerkCost(mN_ + "wheel_torque", k, 0.01);
      }
    }
  }

  void initSolver(std::shared_ptr<std::atomic_bool> exit_flag_ptr = nullptr)
  {
    callback_data_buffer_ = std::make_shared<std::deque<IterationCallback::IPOPTOutput>>();
    callback_data_mutex_ = std::make_shared<std::mutex>();

    if (!exit_flag_ptr) {
      exit_flag_ptr = std::make_shared<std::atomic_bool>(false);
    }

    iteration_callback_ = std::make_shared<IterationCallback>(
      num_opt_vars_,
      constraints_.size1(),
      callback_data_buffer_,
      callback_data_mutex_,
      exit_flag_ptr);

    Dict solver_config{
      {"verbose", false},
      {"ipopt.print_level", 5},
      {"iteration_callback", *iteration_callback_},
      {"ipopt.max_iter", 750}
    };

    SXDict nlp_config{
      {"x", getStateVector()},
      {"f", getCost()},
      {"g", getConstraints()},
      {"p", getParamVector()}};

    solver_ = nlpsol("swerve_mpc_planner", "ipopt", nlp_config, solver_config);
  }

  std::map<std::string, DM> runSolver()
  {

    double init_pose_x = -1.0;
    double init_pose_y = 0.0;
    double init_pose_theta = 0.0;
    double init_vel_x = 0.0;
    double init_vel_y = 0.0;
    double init_vel_theta = 0.0;


    double init_m1_steering_angle = 3.14159 / 4;     // -3.14159 / 4.0;
    double init_m1_steering_vel = 0.0;
    double init_m1_wheel_vel = 0.0;

    double init_m2_steering_angle = 3.14159 / 4;     // 3.14159 / 4.0;
    double init_m2_steering_vel = 0.0;
    double init_m2_wheel_vel = 0.0;

    double init_m3_steering_angle = 3.14159 / 4;     // -3.14159 / 4.0;
    double init_m3_steering_vel = 0.0;
    double init_m3_wheel_vel = 0.0;

    double init_m4_steering_angle = -3.14159 / 4;     // 3.14159 / 4.0;
    double init_m4_steering_vel = 0.0;
    double init_m4_wheel_vel = 0.0;

    std::map<std::string, DM> solver_args, res;
    solver_args["lbx"] = getStateLowerBounds();
    solver_args["ubx"] = getStateUpperBounds();
    solver_args["lbg"] = getConstraintLowerBounds();
    solver_args["ubg"] = getConstraintUpperBounds();
    solver_args["x0"] = DM::zeros(getNumOptVars());

    solver_args["p"] = {
      init_pose_x,
      init_pose_y,
      init_pose_theta,
      init_vel_x,
      init_vel_y,
      init_vel_theta,
      init_m1_steering_angle,
      init_m1_steering_vel,                           // , 3.14159 / 4.0, 0.0};
      init_m1_wheel_vel,
      init_m2_steering_angle,
      init_m2_steering_vel,                           // , 3.14159 / 4.0, 0.0};
      init_m2_wheel_vel,
      init_m3_steering_angle,                           // , 3.14159 / 4.0, 0.0};
      init_m3_steering_vel,
      init_m3_wheel_vel,
      init_m4_steering_angle,                           // , 3.14159 / 4.0, 0.0};
      init_m4_steering_vel,
      init_m4_wheel_vel,
    };     // , 3.14159 / 4.0, 0.0};

    std::cout << "Starting Solve" << std::endl;
    res = solver_(solver_args);
    // solving = false;

    return res;
  }

  void publishMPCTrajectory(const ghost_msgs::msg::LabeledVectorMap & msg)
  {
    trajectory_publisher_->publish(msg);
  }

  const Eigen::Vector2d & getModulePosition(int m)
  {
    if (m <= config_.num_swerve_modules) {
      return module_positions_[m - 1];
    } else {
      throw std::runtime_error(
              "[SwerveMPCPlanner::getModulePosition] Module index cannot exceed the number of swerve modules!");
    }
  }

  const casadi::Matrix<casadi::SXElem> & getCost()
  {
    return cost_;
  }

  const casadi::Matrix<casadi::SXElem> & getConstraints()
  {
    return constraints_;
  }

  const casadi::DM & getStateLowerBounds()
  {
    return lbx_;
  }

  const casadi::DM & getStateUpperBounds()
  {
    return ubx_;
  }
  const casadi::DM & getConstraintLowerBounds()
  {
    return lbg_;
  }

  const casadi::DM & getConstraintUpperBounds()
  {
    return ubg_;
  }

  int getNumKnots()
  {
    return num_knots_;
  }

  int getNumOptVars()
  {
    return num_opt_vars_;
  }

  const Config & getConfig()
  {
    return config_;
  }

  const casadi::Matrix<casadi::SXElem> & getStateVector()
  {
    return state_vector_;
  }


  const casadi::Matrix<casadi::SXElem> & getParamVector()
  {
    return param_vector_;
  }

  const std::vector<double> & getTimeVector()
  {
    return time_vector_;
  }

  std::shared_ptr<std::deque<IterationCallback::IPOPTOutput>> getCallbackDataBuffer()
  {
    return callback_data_buffer_;
  }
  std::shared_ptr<std::mutex> getCallbackDataMutex()
  {
    return callback_data_mutex_;
  }

private:
  const double I2M = ghost_util::INCHES_TO_METERS;

  Config config_;
  int num_knots_;
  int num_opt_vars_;

  rclcpp::Publisher<ghost_msgs::msg::LabeledVectorMap>::SharedPtr trajectory_publisher_;
  std::vector<Eigen::Vector2d> module_positions_;
  std::vector<double> time_vector_;

  // Initialize containers for optimization variables
  std::unordered_map<std::string, int> state_index_map_;
  casadi::Matrix<casadi::SXElem> state_vector_;
  std::unordered_map<std::string, int> param_index_map_;
  casadi::Matrix<casadi::SXElem> param_vector_;
  std::unordered_map<std::string, int> weight_index_map_;
  std::vector<double> weights_;

  casadi::Matrix<casadi::SXElem> cost_;
  casadi::DM lbx_;
  casadi::DM ubx_;

  casadi::Matrix<casadi::SXElem> constraints_;
  casadi::DM lbg_;
  casadi::DM ubg_;

  std::vector<std::string> state_names_;
  std::vector<std::string> param_names_;

  std::shared_ptr<ghost_planners::Trajectory> reference_trajectory_ptr_;

  casadi::Function solver_;

  std::shared_ptr<std::deque<IterationCallback::IPOPTOutput>> callback_data_buffer_;
  std::shared_ptr<std::mutex> callback_data_mutex_;
  std::shared_ptr<IterationCallback> iteration_callback_;


};

int main(int argc, char * argv[])
{
  signal(SIGINT, siginthandler);

  bool plot = true;
  ////////////////////////////////////
  /////// Instantiate ROS Node ///////
  ////////////////////////////////////
  rclcpp::init(argc, argv);

  auto node_ptr = std::make_shared<SwerveMPCPlanner>();
  std::thread node_thread([&]() {
      rclcpp::spin(node_ptr);
    });

  plt::figure();
  plt::plot(std::vector<double>{}, std::vector<double>{});
  std::vector<IterationCallback::IPOPTOutput> solver_iteration_data;
  std::vector<double> iteration_vector{};
  std::vector<double> cost_vector{};
  std::atomic<bool> solving(true);

  // plt::ion();
  // plt::show();

  // while (solving) {
  //   std::unique_lock<std::mutex> lock(*callback_data_mutex_);
  //   if (!callback_data_buffer_->empty()) {
  //     // Retrieve data from queue
  //     IterationCallback::IPOPTOutput data(callback_data_buffer_->back());
  //     callback_data_buffer_->pop_back();

  //     // Matplotlib
  //     solver_iteration_data.push_back(data);
  //     iteration_vector.push_back(data.iteration);
  //     cost_vector.push_back(data.f);

  //     plt::clf();
  //     plt::plot(iteration_vector, cost_vector);

  //     // Publish to RVIZ
  //     ghost_msgs::msg::LabeledVectorMap msg{};
  //     ghost_ros_interfaces::msg_helpers::toROSMsg(generateTrajectoryMap(data.x), msg);
  //     publishMPCTrajectory(msg);

  //   }
  //   lock.unlock();

  //   if (true) {   // plot
  //     plt::pause(0.01);
  //   } else {
  //     std::this_thread::sleep_for(10ms);
  //   }
  // }

  auto res = node_ptr->runSolver();

  auto raw_solution_vector = std::vector<double>(res.at("x"));
  ghost_msgs::msg::LabeledVectorMap msg{};
  ghost_ros_interfaces::msg_helpers::toROSMsg(
    node_ptr->generateTrajectoryMap(
      raw_solution_vector), msg);
  node_ptr->publishMPCTrajectory(msg);

  /////////////////////////////
  ///// Evaluate Solution /////
  /////////////////////////////
  // Print the solution
  std::cout << "-----" << std::endl;
  std::cout << "Objective: " << res.at("f") << std::endl;

  // Unpack solution into individual time series
  std::unordered_map<std::string,
    std::vector<double>> state_solution_map = node_ptr->generateTrajectoryMap(
    raw_solution_vector);

  if (!plot) {
    return 0;
  }

  //////////////////////////////
  ///// Animate Trajectory /////
  //////////////////////////////

  // Get Data
  const int NUM_KNOTS = node_ptr->getNumKnots();
  const double DT = node_ptr->getConfig().dt;
  const double CHASSIS_WIDTH = node_ptr->getConfig().chassis_width;
  const double WHEEL_WIDTH = node_ptr->getConfig().wheel_width;
  const double WHEEL_RADIUS = node_ptr->getConfig().wheel_radius;
  auto time_vector = node_ptr->getTimeVector();

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

  x_pos.reserve(NUM_KNOTS);
  y_pos.reserve(NUM_KNOTS);
  x_vel.reserve(NUM_KNOTS);
  y_vel.reserve(NUM_KNOTS);
  x_angle_components.reserve(NUM_KNOTS);
  y_angle_components.reserve(NUM_KNOTS);
  wheel_force_x_components.reserve(NUM_KNOTS);
  wheel_force_y_components.reserve(NUM_KNOTS);
  lateral_force_x_components.reserve(NUM_KNOTS);
  lateral_force_y_components.reserve(NUM_KNOTS);

  for (int k = 0; k < NUM_KNOTS; k++) {
    x_pos.push_back(state_solution_map["base_pose_x"][k]);
    y_pos.push_back(state_solution_map["base_pose_y"][k]);
    x_vel.push_back(state_solution_map["base_vel_x"][k]);
    y_vel.push_back(state_solution_map["base_vel_y"][k]);

    x_angle_components.push_back(cos(state_solution_map["base_pose_theta"][k]));
    y_angle_components.push_back(sin(state_solution_map["base_pose_theta"][k]));

    auto angle = state_solution_map["m1_steering_angle"][k];
    auto wheel_force = state_solution_map["m1_wheel_torque"][k] / WHEEL_RADIUS;
    auto lateral_force = state_solution_map["m1_lateral_force"][k];
    wheel_force_x_components.push_back(wheel_force * cos(angle));
    wheel_force_y_components.push_back(wheel_force * sin(angle));
    lateral_force_x_components.push_back(-lateral_force * sin(angle));
    lateral_force_y_components.push_back(lateral_force * cos(angle));
  }
  auto plotRectanglePoints =
    [&](double x, double y, double width, double height, double angle,
      std::vector<double> & x_points, std::vector<double> & y_points) {
      auto rotation_matrix = Eigen::Rotation2D<double>(angle).toRotationMatrix();

      Eigen::Vector2d top_left = rotation_matrix * Eigen::Vector2d(-width / 2, height / 2);
      Eigen::Vector2d top_right = rotation_matrix * Eigen::Vector2d(width / 2, height / 2);
      Eigen::Vector2d bottom_right = rotation_matrix * Eigen::Vector2d(width / 2, -height / 2);
      Eigen::Vector2d bottom_left = rotation_matrix * Eigen::Vector2d(-width / 2, -height / 2);

      x_points.clear();
      x_points.reserve(5);
      x_points.push_back(x + top_left.x());
      x_points.push_back(x + top_right.x());
      x_points.push_back(x + bottom_right.x());
      x_points.push_back(x + bottom_left.x());
      x_points.push_back(x + top_left.x());

      y_points.clear();
      y_points.reserve(5);
      y_points.push_back(y + top_left.y());
      y_points.push_back(y + top_right.y());
      y_points.push_back(y + bottom_right.y());
      y_points.push_back(y + bottom_left.y());
      y_points.push_back(y + top_left.y());

      return;
    };

  plt::figure();
  for (int k = 0; k < NUM_KNOTS; k++) {
    break;
    plt::clf();
    double x = state_solution_map["base_pose_x"][k];
    double y = state_solution_map["base_pose_y"][k];
    double theta = state_solution_map["base_pose_theta"][k];

    // Plot Chassis
    std::vector<double> chassis_x_points;
    std::vector<double> chassis_y_points;
    plotRectanglePoints(
      x, y, CHASSIS_WIDTH, CHASSIS_WIDTH, theta, chassis_x_points,
      chassis_y_points);

    plt::axis("scaled");
    plt::xlim(-24.0 * 3 * ghost_util::INCHES_TO_METERS, 3 * 24.0 * ghost_util::INCHES_TO_METERS);
    plt::ylim(-24.0 * 3 * ghost_util::INCHES_TO_METERS, 3 * 24.0 * ghost_util::INCHES_TO_METERS);

    plt::plot(
      chassis_x_points,
      chassis_y_points,
      std::map<std::string, std::string>{{"color", "black"}});

    // Plot Base Linear Velocity
    plt::plot(
      std::vector<double>{x, x + state_solution_map["base_vel_x"][k] / 10.0},
      std::vector<double>{y, y + state_solution_map["base_vel_y"][k] / 10.0},
      std::map<std::string, std::string>{{"color", "green"}});

    plt::plot(
      std::vector<double>{x, x + state_solution_map["base_accel_x"][k] / 10.0},
      std::vector<double>{y, y + state_solution_map["base_accel_y"][k] / 10.0},
      std::map<std::string, std::string>{{"color", "red"}});

    // Plot Base Link Center Point
    plt::scatter(
      std::vector<double>{x},
      std::vector<double>{y});

    // Plot Modules
    auto rotate_theta = Eigen::Rotation2D<double>(theta).toRotationMatrix();
    auto plot_wheel_module =
      [&](const Eigen::Vector2d & steering_chassis_offset, std::string module_prefix) {
        std::vector<double> m_x_points;
        std::vector<double> m_y_points;
        Eigen::Vector2d steering_center =
          Eigen::Vector2d(x, y) + rotate_theta * steering_chassis_offset;
        double steering_angle = state_solution_map[module_prefix + "steering_angle"][k];
        // Plot Wheel
        plotRectanglePoints(
          steering_center.x(),
          steering_center.y(), WHEEL_RADIUS * 2, WHEEL_WIDTH, theta + steering_angle, m_x_points,
          m_y_points);
        plt::plot(
          m_x_points,
          m_y_points,
          std::map<std::string, std::string>{{"color", "black"}});

        // Extract and plot forces
        auto angle = state_solution_map[module_prefix + "steering_angle"][k] + theta;
        auto wheel_force = state_solution_map[module_prefix + "wheel_torque"][k] / WHEEL_RADIUS;
        auto lateral_force = state_solution_map[module_prefix + "lateral_force"][k];

        plt::plot(
          std::vector<double>{steering_center.x(), steering_center.x() + wheel_force * cos(
              angle) / 100.0},
          std::vector<double>{steering_center.y(), steering_center.y() + wheel_force * sin(
              angle) / 100.0},
          std::map<std::string, std::string>{{"color", "red"}});

        plt::plot(
          std::vector<double>{steering_center.x(), steering_center.x() - lateral_force * sin(
              angle) / 100.0},
          std::vector<double>{steering_center.y(), steering_center.y() + lateral_force * cos(
              angle) / 100.0},
          std::map<std::string, std::string>{{"color", "blue"}});
      };

    plot_wheel_module(node_ptr->getModulePosition(1), "m1_");
    plot_wheel_module(node_ptr->getModulePosition(2), "m2_");
    plot_wheel_module(node_ptr->getModulePosition(3), "m3_");
    plot_wheel_module(node_ptr->getModulePosition(4), "m4_");

    if (*EXIT_GLOBAL_PTR) {
      *EXIT_GLOBAL_PTR = false;
      break;
    }
  }


  plt::figure();
  plt::quiver(x_pos, y_pos, x_vel, y_vel, std::map<std::string, std::string>{{"color", "black"}});
  plt::quiver(
    x_pos, y_pos, wheel_force_x_components, wheel_force_y_components,
    std::map<std::string, std::string>{{"color", "red"}});
  plt::quiver(
    x_pos, y_pos, lateral_force_x_components, lateral_force_y_components,
    std::map<std::string, std::string>{{"color", "blue"}});
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
  plt::suptitle("Wheel Steering");
  plt::subplot(2, 2, 1);
  plt::plot(time_vector, state_solution_map["m1_steering_angle"]);
  plt::subplot(2, 2, 2);
  plt::plot(time_vector, state_solution_map["m2_steering_angle"]);
  plt::subplot(2, 2, 3);
  plt::plot(time_vector, state_solution_map["m3_steering_angle"]);
  plt::subplot(2, 2, 4);
  plt::plot(time_vector, state_solution_map["m4_steering_angle"]);

  plt::figure();
  plt::suptitle("m1_forces");
  plt::subplot(2, 1, 1);
  plt::plot(time_vector, state_solution_map["m1_wheel_torque"]);
  plt::subplot(2, 1, 2);
  plt::plot(time_vector, state_solution_map["m1_lateral_force"]);

  plt::figure();
  plt::suptitle("m2_forces");
  plt::subplot(2, 1, 1);
  plt::plot(time_vector, state_solution_map["m2_wheel_torque"]);
  plt::subplot(2, 1, 2);
  plt::plot(time_vector, state_solution_map["m2_lateral_force"]);

  plt::figure();
  plt::suptitle("m3_forces");
  plt::subplot(2, 1, 1);
  plt::plot(time_vector, state_solution_map["m3_wheel_torque"]);
  plt::subplot(2, 1, 2);
  plt::plot(time_vector, state_solution_map["m3_lateral_force"]);

  plt::figure();
  plt::suptitle("m4_forces");
  plt::subplot(2, 1, 1);
  plt::plot(time_vector, state_solution_map["m4_wheel_torque"]);
  plt::subplot(2, 1, 2);
  plt::plot(time_vector, state_solution_map["m4_lateral_force"]);

  plt::figure();
  plt::suptitle("Wheel Velocities");
  plt::subplot(2, 2, 1);
  plt::plot(time_vector, state_solution_map["m1_wheel_vel"]);
  plt::subplot(2, 2, 2);
  plt::plot(time_vector, state_solution_map["m2_wheel_vel"]);
  plt::subplot(2, 2, 3);
  plt::plot(time_vector, state_solution_map["m3_wheel_vel"]);
  plt::subplot(2, 2, 4);
  plt::plot(time_vector, state_solution_map["m4_wheel_vel"]);

  plt::ion();
  plt::pause(0.1);
  plt::show();

  while (!(*EXIT_GLOBAL_PTR)) {
    std::this_thread::sleep_for(50ms);
  }

  rclcpp::shutdown();

  return 0;
}
