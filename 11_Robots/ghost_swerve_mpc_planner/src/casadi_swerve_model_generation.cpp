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
#include "eigen3/Eigen/Geometry"
#include <casadi/casadi.hpp>

#include <ghost_util/unit_conversion_utils.hpp>

#include <ghost_msgs/msg/labeled_vector.hpp>
#include <ghost_msgs/msg/labeled_vector_map.hpp>

#include <ghost_ros_interfaces/msg_helpers/msg_helpers.hpp>

#include <atomic>
#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>

#include <signal.h>
#include "matplotlibcpp.h"

#include <atomic>

using namespace std::chrono_literals;
namespace plt = matplotlibcpp;
using namespace casadi;

std::atomic_bool EXIT_GLOBAL = false;

void siginthandler(int param)
{
  std::cout << "Ctr-C Received, Aborting..." << std::endl;
  EXIT_GLOBAL = true;
}
class IterationCallback : public casadi::Callback
{
public:
  struct IPOPTOutput
  {
    IPOPTOutput()
    {
    }
    IPOPTOutput(const IPOPTOutput & rhs)
    {
      x = rhs.x;
      g = rhs.g;
      f = rhs.f;
      lam_x = rhs.lam_x;
      lam_g = rhs.lam_g;
      iteration = rhs.iteration;
    }

    int iteration;
    std::vector<double> x;
    std::vector<double> g;
    double f = 0.0;
    std::vector<double> lam_x;
    std::vector<double> lam_g;
  };

  IterationCallback(
    int nx, int ng, std::shared_ptr<std::deque<IPOPTOutput>> data_buffer,
    std::shared_ptr<std::mutex> data_mutex)
  : nx_(nx),
    ng_(ng),
    data_buffer_(data_buffer),
    data_mutex_(data_mutex),
    casadi::Callback()
  {
    construct("iteration_callback");
  }

  bool has_eval_buffer() const override
  {
    return true;
  }

  void init() override
  {
  }

  casadi_int get_n_in() override
  {
    return nlpsol_n_out();
  }

  casadi_int get_n_out() override
  {
    return 1;
  }

  std::string get_name_in(casadi_int i) override
  {
    return nlpsol_out(i);
  }

  Sparsity get_sparsity_in(casadi_int i) override
  {
    auto n = nlpsol_out(i);
    if (n == "f") {
      return Sparsity::scalar();
    } else if ((n == "x") || (n == "lam_x") ) {
      return Sparsity::dense(nx_);
    } else if ((n == "g") || (n == "lam_g") ) {
      return Sparsity::dense(ng_);
    } else {
      return Sparsity(0, 0);
    }
  }

  std::string get_name_out(casadi_int i) override
  {
    return "ret";
  }

  std::vector<DM> eval(const std::vector<DM> & arg) const override
  {
    return {0};
  }

  int eval_buffer(
    const double ** arg, const std::vector<casadi_int> & sizes_arg,
    double ** res, const std::vector<casadi_int> & sizes_res) const override
  {
    auto output_map = nlpsol_out();
    IPOPTOutput data{};

    data.x.resize(nx_);
    data.g.resize(ng_);
    data.lam_x.resize(nx_);
    data.lam_g.resize(ng_);

    for (int i = 0; i < sizes_arg[0]; i++) {
      data.x[i] = arg[0][i];
    }
    data.f = arg[1][0];

    for (int i = 0; i < sizes_arg[2]; i++) {
      data.g[i] = arg[2][i];
    }

    for (int i = 0; i < sizes_arg[3]; i++) {
      data.lam_x[i] = arg[3][i];
    }

    for (int i = 0; i < sizes_arg[4]; i++) {
      data.lam_g[i] = arg[4][i];
    }

    data.iteration = iteration_count_;

    std::unique_lock<std::mutex> lock(*data_mutex_);
    data_buffer_->push_front(data);
    iteration_count_++;

    if (EXIT_GLOBAL) {
      *(res[0]) = 1.0;
      EXIT_GLOBAL = false;
    }
    return {0};
  }

  int nx_;
  int ng_;
  std::shared_ptr<std::deque<IPOPTOutput>> data_buffer_;
  std::shared_ptr<std::mutex> data_mutex_;
  mutable int iteration_count_ = 1;
};

int main(int argc, char * argv[])
{
  signal(SIGINT, siginthandler);

  bool plot = true;
  ////////////////////////////////////
  /////// Instantiate ROS Node ///////
  ////////////////////////////////////
  rclcpp::init(argc, argv);

  auto node_ptr = std::make_shared<rclcpp::Node>("swerve_mpc_node");
  auto mpc_trajectory_publisher = node_ptr->create_publisher<ghost_msgs::msg::LabeledVectorMap>(
    "/trajectory/swerve_mpc_trajectory", 10);

  std::thread node_thread([&]() {
      rclcpp::spin(node_ptr);
    });

  ////////////////////////////////////
  ///// User Input Configuration /////
  ////////////////////////////////////
  const float TIME_HORIZON = 2.0;
  const float DT = 0.01;
  const int NUM_SWERVE_MODULES = 4;

  // Swerve Config
  const double CHASSIS_WIDTH = 16.5 * ghost_util::INCHES_TO_METERS;
  const double WHEEL_BASE_WIDTH = 12.5 * ghost_util::INCHES_TO_METERS;
  const double WHEEL_DIAM = 2.75 * ghost_util::INCHES_TO_METERS;
  const double WHEEL_WIDTH = 1 * ghost_util::INCHES_TO_METERS;
  const double M1_X = WHEEL_BASE_WIDTH / 2;
  const double M1_Y = WHEEL_BASE_WIDTH / 2;
  const double M2_X = -WHEEL_BASE_WIDTH / 2;
  const double M2_Y = WHEEL_BASE_WIDTH / 2;
  const double M3_X = -WHEEL_BASE_WIDTH / 2;
  const double M3_Y = -WHEEL_BASE_WIDTH / 2;
  const double M4_X = WHEEL_BASE_WIDTH / 2;
  const double M4_Y = -WHEEL_BASE_WIDTH / 2;

  // Params
  double mass = 10.0;
  double inertia = 0.14868;
  double init_pose_x = -1.0;
  double init_pose_y = 0.0;
  double init_pose_theta = 0.0;
  double init_vel_x = 0.0;
  double init_vel_y = 0.0;
  double init_vel_theta = 0.0;
  double des_vel_x = 0.75;
  double des_vel_y = 0.0;
  double des_vel_theta = 6.5;
  double init_m1_steering_angle = 0.0;       // -3.14159 / 4.0;
  double init_m1_steering_vel = 0.0;
  double init_m2_steering_angle = 0.0;       // 3.14159 / 4.0;
  double init_m2_steering_vel = 0.0;
  double init_m3_steering_angle = 0.0;       // -3.14159 / 4.0;
  double init_m3_steering_vel = 0.0;
  double init_m4_steering_angle = 0.0;       // 3.14159 / 4.0;
  double init_m4_steering_vel = 0.0;

  std::vector<Eigen::Vector2d> module_positions{
    Eigen::Vector2d(M1_X, M1_Y),
    Eigen::Vector2d(M2_X, M2_Y),
    Eigen::Vector2d(M3_X, M3_Y),
    Eigen::Vector2d(M4_X, M4_Y)
  };

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
    // "wheel_vel",
    // "wheel_accel",
    // "voltage_1",
    // "voltage_2",
  };

  // Add joint states to state name vector
  for (int m = 1; m < NUM_SWERVE_MODULES + 1; m++) {
    std::string module_prefix = "m" + std::to_string(m) + "_";
    for (auto name : JOINT_STATE_NAMES) {
      STATE_NAMES.push_back(module_prefix + name);
    }
  }
  std::vector<std::string> PARAM_NAMES = {
    "mass",
    "inertia",
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

  for (int m = 1; m < NUM_SWERVE_MODULES + 1; m++) {
    PARAM_NAMES.push_back(std::string("init_m") + std::to_string(m) + "_steering_angle");
    PARAM_NAMES.push_back(std::string("init_m") + std::to_string(m) + "_steering_vel");
  }

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
      if (state_index_map.count(name) != 0) {
        return state_vector(state_index_map.at(name));
      } else {
        throw std::runtime_error("[get_state] No state with name " + name);
      }
    };

  // Shorthand to get symbolic parameter by name
  auto get_param = [&param_vector, &param_index_map](std::string name) {
      if (param_index_map.count(name) != 0) {
        return param_vector(param_index_map.at(name));
      } else {
        throw std::runtime_error("[get_param] No param with name " + name);
      }
    };

  // Shorthand to get knot string prefix from knotpoint index
  auto get_knot_prefix = [](int i) {
      return "k" + std::to_string(i) + "_";
    };
  auto generate_trajectory_map = [&](const std::vector<double> & solution_vector) {
      std::unordered_map<std::string, std::vector<double>> solution_map;

      for (auto & name : STATE_NAMES) {
        // Allocate timeseries vector for each state/input variable
        solution_map[name] = std::vector<double>(NUM_KNOTS);

        // Iterate through timeseries and add final state values to solution vector
        for (int k = 0; k < NUM_KNOTS; k++) {
          std::string knot_prefix = get_knot_prefix(k);
          solution_map[name][k] = solution_vector[state_index_map.at(knot_prefix + name)];
        }
      }

      // Allocate timeseries vector for each state/input variable
      solution_map["time"] = std::vector<double>(NUM_KNOTS);

      // Iterate through timeseries and add final state values to solution vector
      for (int k = 0; k < NUM_KNOTS; k++) {
        solution_map["time"][k] = DT * ((double) k);
      }

      return solution_map;
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

  // EULER INTEGRATION CONSTRAINTS
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
  for (int m = 1; m < NUM_SWERVE_MODULES + 1; m++) {
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
  auto integration_constraints_vector =
    SX::zeros(euler_integration_state_names.size() * (NUM_KNOTS - 1));
  int integration_constraint_index = 0;
  for (int k = 0; k < NUM_KNOTS - 1; k++) {
    std::string curr_knot_prefix = get_knot_prefix(k);
    std::string next_knot_prefix = get_knot_prefix(k + 1);

    for (auto & pair : euler_integration_state_names) {
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

  // Initial State Constraints
  std::vector<std::pair<std::string, std::string>> initial_state_constraint_param_pairs{
    std::pair<std::string, std::string>{"k0_base_pose_x", "init_pose_x"},
    std::pair<std::string, std::string>{"k0_base_pose_y", "init_pose_y"},
    std::pair<std::string, std::string>{"k0_base_pose_theta", "init_pose_theta"},
    std::pair<std::string, std::string>{"k0_base_vel_x", "init_vel_x"},
    std::pair<std::string, std::string>{"k0_base_vel_y", "init_vel_y"},
    std::pair<std::string, std::string>{"k0_base_vel_theta", "init_vel_theta"}
  };

  for (int m = 1; m < NUM_SWERVE_MODULES + 1; m++) {
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
  }

  auto initial_state_constraint_vector = casadi::Matrix<casadi::SXElem>();
  for (const auto & pair : initial_state_constraint_param_pairs) {
    initial_state_constraint_vector = vertcat(
      initial_state_constraint_vector, get_state(
        pair.first) - get_param(pair.second));
  }

  // acceleration dynamics constraints
  auto acceleration_dynamics_constraint_vector = casadi::Matrix<casadi::SXElem>();
  for (int k = 0; k < NUM_KNOTS; k++) {
    std::string curr_knot_prefix = get_knot_prefix(k);
    auto x_accel_constraint = get_param("mass") * get_state(curr_knot_prefix + "base_accel_x");
    auto y_accel_constraint = get_param("mass") * get_state(curr_knot_prefix + "base_accel_y");
    auto theta_accel_constraint = get_param("inertia") * get_state(
      curr_knot_prefix + "base_accel_theta");
    auto theta = get_state(curr_knot_prefix + "base_pose_theta");
    auto x_pos = get_state(curr_knot_prefix + "base_pose_x");
    auto y_pos = get_state(curr_knot_prefix + "base_pose_y");

    for (int m = 1; m < NUM_SWERVE_MODULES + 1; m++) {
      std::string module_prefix = curr_knot_prefix + "m" + std::to_string(m) + "_";
      auto wheel_force = get_state(module_prefix + "wheel_force");
      auto lateral_force = get_state(module_prefix + "lateral_force");
      auto world_steering_angle = theta + get_state(module_prefix + "steering_angle");
      auto x_force = cos(world_steering_angle) * wheel_force - sin(world_steering_angle) *
        lateral_force;
      auto y_force = sin(world_steering_angle) * wheel_force + cos(world_steering_angle) *
        lateral_force;
      x_accel_constraint -= x_force;
      y_accel_constraint -= y_force;

      auto module_position_world_x = x_pos + cos(theta) * module_positions[m - 1].x() - sin(theta) *
        module_positions[m - 1].y();
      auto module_position_world_y = y_pos + sin(theta) * module_positions[m - 1].x() + cos(theta) *
        module_positions[m - 1].y();
      auto base_torque = y_force * (module_position_world_x - x_pos) - x_force *
        (module_position_world_y - y_pos);
      theta_accel_constraint -= base_torque;
    }

    acceleration_dynamics_constraint_vector = vertcat(
      acceleration_dynamics_constraint_vector,
      x_accel_constraint);
    acceleration_dynamics_constraint_vector = vertcat(
      acceleration_dynamics_constraint_vector,
      y_accel_constraint);
    acceleration_dynamics_constraint_vector = vertcat(
      acceleration_dynamics_constraint_vector,
      theta_accel_constraint);
  }

  // // Zero Lateral wheel velocity constraint
  auto zero_lateral_wheel_vel_constraint = casadi::Matrix<casadi::SXElem>();

  // // Combine all constraints into single vector
  // auto constraints = vertcat(
  //    integration_constraints_vector,
  //    initial_state_constraint_vector,
  //    acceleration_dynamics_constraint_vector,
  //    zero_lateral_wheel_vel_constraint
  //    );

  // Optimization Variables Limits
  auto lbx = DM::ones(NUM_OPT_VARS) * -DM::inf();
  auto ubx = DM::ones(NUM_OPT_VARS) * DM::inf();
  double translation_speed_limit = 3.0;       // 1.389
  double translation_accel_limit = 15.0;       // 5.5755

  for (int k = 0; k < NUM_KNOTS; k++) {
    std::string curr_knot_prefix = get_knot_prefix(k);
    lbx(state_index_map[curr_knot_prefix + "base_vel_x"]) = -translation_speed_limit;
    ubx(state_index_map[curr_knot_prefix + "base_vel_x"]) = translation_speed_limit;
    lbx(state_index_map[curr_knot_prefix + "base_vel_y"]) = -translation_speed_limit;
    ubx(state_index_map[curr_knot_prefix + "base_vel_y"]) = translation_speed_limit;
    lbx(state_index_map[curr_knot_prefix + "base_vel_theta"]) = -7.0305;
    ubx(state_index_map[curr_knot_prefix + "base_vel_theta"]) = 7.0305;

    lbx(state_index_map[curr_knot_prefix + "base_accel_x"]) = -translation_accel_limit;
    ubx(state_index_map[curr_knot_prefix + "base_accel_x"]) = translation_accel_limit;
    lbx(state_index_map[curr_knot_prefix + "base_accel_y"]) = -translation_accel_limit;
    ubx(state_index_map[curr_knot_prefix + "base_accel_y"]) = translation_accel_limit;
    lbx(state_index_map[curr_knot_prefix + "base_accel_theta"]) = -45.97811;
    ubx(state_index_map[curr_knot_prefix + "base_accel_theta"]) = 45.97811;

    for (int m = 1; m < NUM_SWERVE_MODULES + 1; m++) {
      std::string module_prefix = curr_knot_prefix + "m" + std::to_string(m) + "_";
      lbx(state_index_map[module_prefix + "steering_accel"]) = -5000.0;
      ubx(state_index_map[module_prefix + "steering_accel"]) = 5000.0;

      lbx(state_index_map[module_prefix + "wheel_force"]) = -50.0;
      ubx(state_index_map[module_prefix + "wheel_force"]) = 50.0;

      lbx(state_index_map[module_prefix + "lateral_force"]) = -mass * 9.81 * 1.25;
      ubx(state_index_map[module_prefix + "lateral_force"]) = mass * 9.81 * 1.25;
    }
  }

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
        get_state(curr_knot_prefix + "base_accel_theta"),
        2) + pow(get_state(next_knot_prefix + "base_accel_theta"), 2));

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
        get_state(curr_knot_prefix + "base_accel_theta") -
        get_state(next_knot_prefix + "base_accel_theta"), 2));

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
        get_param("des_vel_theta") - get_state(curr_knot_prefix + "base_vel_theta"),
        2) + pow(get_param("des_vel_theta") - get_state(next_knot_prefix + "base_vel_theta"), 2));

    auto vel_x = get_state(curr_knot_prefix + "base_vel_x");
    auto vel_y = get_state(curr_knot_prefix + "base_vel_y");
    auto vel_theta = get_state(curr_knot_prefix + "base_vel_theta");             // world frame
    auto theta = get_state(curr_knot_prefix + "base_pose_theta");
    auto x_pos = get_state(curr_knot_prefix + "base_pose_x");
    auto y_pos = get_state(curr_knot_prefix + "base_pose_y");

    for (int m = 1; m < NUM_SWERVE_MODULES + 1; m++) {
      std::string curr_knot_module_prefix = curr_knot_prefix + "m" + std::to_string(m) + "_";
      std::string next_knot_module_prefix = next_knot_prefix + "m" + std::to_string(m) + "_";
      f += 0.00001 * 1 / DT *
        (pow(
          get_state(curr_knot_module_prefix + "steering_accel"),
          2) + pow(get_state(next_knot_module_prefix + "steering_accel"), 2));

      // Small Regularization
      f += 0.00001 * 1 / DT *
        (pow(
          get_state(curr_knot_module_prefix + "steering_vel"),
          2) + pow(get_state(next_knot_module_prefix + "steering_vel"), 2));
      f += 0.00001 * 1 / DT *
        (pow(
          get_state(curr_knot_module_prefix + "lateral_force"),
          2) + pow(get_state(next_knot_module_prefix + "lateral_force"), 2));
      f += 0.001 * 1 / DT *
        (pow(
          get_state(curr_knot_module_prefix + "lateral_force") -
          get_state(next_knot_module_prefix + "lateral_force"), 2));
      f += 0.00001 * 1 / DT *
        (pow(
          get_state(curr_knot_module_prefix + "wheel_force"),
          2) + pow(get_state(next_knot_module_prefix + "wheel_force"), 2));
      f += 0.01 * 1 / DT *
        (pow(
          get_state(curr_knot_module_prefix + "wheel_force") -
          get_state(next_knot_module_prefix + "wheel_force"), 2));

      auto world_steering_angle = theta + get_state(curr_knot_module_prefix + "steering_angle");
      auto base_steering_angle = get_state(curr_knot_module_prefix + "steering_angle");
      auto module_position_world_x = cos(theta) * module_positions[m - 1].x() - sin(theta) *
        module_positions[m - 1].y();
      auto module_position_world_y = sin(theta) * module_positions[m - 1].x() + cos(theta) *
        module_positions[m - 1].y();
      auto tan_vel = vel_theta *
        sqrt((pow(module_position_world_x, 2) + pow(module_position_world_y, 2)));
      auto r_angle = atan2(module_positions[m - 1].y(), module_positions[m - 1].x());                   // base_link
      auto tan_vel_x = tan_vel * -sin(r_angle);
      auto tan_vel_y = tan_vel * cos(r_angle);
      auto world_tan_vel_x = tan_vel_x * cos(theta) - tan_vel_y * sin(theta);
      auto world_tan_vel_y = tan_vel_x * sin(theta) + tan_vel_y * cos(theta);
      auto forward_velocity = (world_tan_vel_y + vel_y) * cos(-world_steering_angle) -
        (world_tan_vel_x + vel_x) * sin(-world_steering_angle);
      auto lateral_velocity = (world_tan_vel_x + vel_x) * sin(-world_steering_angle) +
        (world_tan_vel_y + vel_y) * cos(-world_steering_angle);
      zero_lateral_wheel_vel_constraint = vertcat(
        zero_lateral_wheel_vel_constraint,
        lateral_velocity);
      // auto module_offset_world_x = cos(theta) * module_positions[m - 1].x() - sin(theta) * module_positions[m - 1].y();
      // auto module_offset_world_y = sin(theta) * module_positions[m - 1].x() + cos(theta) * module_positions[m - 1].y();
      // auto module_x_velocity_world = vel_x - vel_theta * module_offset_world_y;
      // auto module_y_velocity_world = vel_y + vel_theta * module_offset_world_x;
      // auto module_x_velocity_local = cos(world_steering_angle) * module_x_velocity_world + sin(world_steering_angle) * module_y_velocity_world;
      // auto module_y_velocity_local = -sin(world_steering_angle) * module_x_velocity_world + cos(world_steering_angle) * module_y_velocity_world;
      // zero_lateral_wheel_vel_constraint = vertcat(zero_lateral_wheel_vel_constraint, module_y_velocity_local);
      f += 0.000001 * pow(lateral_velocity, 2);
    }
  }

  // Combine all constraints into single vector
  auto constraints = vertcat(
    integration_constraints_vector,
    initial_state_constraint_vector,
    acceleration_dynamics_constraint_vector,
    zero_lateral_wheel_vel_constraint
  );
  auto lbg = DM::zeros(constraints.size1());
  auto ubg = DM::zeros(constraints.size1());

  for (int i = integration_constraints_vector.size1() + initial_state_constraint_vector.size1() +
    acceleration_dynamics_constraint_vector.size1(); i < integration_constraints_vector.size1() +
    initial_state_constraint_vector.size1() +
    acceleration_dynamics_constraint_vector.size1() + zero_lateral_wheel_vel_constraint.size1();
    i++)
  {
    lbg(i) = -0.001;
    ubg(i) = 0.001;
  }

  /////////////////////////
  ///// Create Solver /////
  /////////////////////////
  auto data_buffer = std::make_shared<std::deque<IterationCallback::IPOPTOutput>>();
  auto data_mutex = std::make_shared<std::mutex>();
  auto iteration_callback = IterationCallback(
    NUM_OPT_VARS,
    constraints.size1(), data_buffer, data_mutex);
  SXDict nlp_config{
    {"x", state_vector},
    {"f", f},
    {"g", constraints},
    {"p", param_vector}};
  Dict solver_config{
    {"verbose", false},
    {"ipopt.print_level", 0},
    {"iteration_callback", iteration_callback},
    {"ipopt.max_iter", 750}
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

  solver_args["p"] = {mass,
    inertia,
    init_pose_x,
    init_pose_y,
    init_pose_theta,
    init_vel_x,
    init_vel_y,
    init_vel_theta,
    des_vel_x,
    des_vel_y,
    des_vel_theta,
    init_m1_steering_angle,
    init_m1_steering_vel,                             // , 3.14159 / 4.0, 0.0};
    init_m2_steering_angle,
    init_m2_steering_vel,                             // , 3.14159 / 4.0, 0.0};
    init_m3_steering_angle,                             // , 3.14159 / 4.0, 0.0};
    init_m3_steering_vel,
    init_m4_steering_angle,                             // , 3.14159 / 4.0, 0.0};
    init_m4_steering_vel
  };       // , 3.14159 / 4.0, 0.0};


  if (plot) {
    plt::figure();
    plt::plot(std::vector<double>{}, std::vector<double>{});
  }
  std::vector<IterationCallback::IPOPTOutput> solver_iteration_data;
  std::vector<double> iteration_vector{};
  std::vector<double> cost_vector{};
  std::atomic<bool> solving(true);

  std::thread solver_thread([&]() {
      std::cout << "Starting Solve" << std::endl;
      res = solver(solver_args);
      solving = false;
    });
  plt::ion();
  // plt::show();

  while (solving) {
    std::unique_lock<std::mutex> lock(*data_mutex);
    if (!data_buffer->empty()) {
      // Retrieve data from queue
      IterationCallback::IPOPTOutput data(data_buffer->back());
      data_buffer->pop_back();

      // Matplotlib
      solver_iteration_data.push_back(data);
      iteration_vector.push_back(data.iteration);
      cost_vector.push_back(data.f);

      if (plot) {
        plt::clf();
        plt::plot(iteration_vector, cost_vector);
      }

      // Publish to RVIZ
      ghost_msgs::msg::LabeledVectorMap msg{};
      ghost_ros_interfaces::msg_helpers::toROSMsg(generate_trajectory_map(data.x), msg);
      mpc_trajectory_publisher->publish(msg);
    }
    lock.unlock();

    if (plot) {
      plt::pause(0.01);
    } else {
      std::this_thread::sleep_for(10ms);
    }
  }


  auto raw_solution_vector = std::vector<double>(res.at("x"));
  ghost_msgs::msg::LabeledVectorMap msg{};
  ghost_ros_interfaces::msg_helpers::toROSMsg(generate_trajectory_map(raw_solution_vector), msg);
  mpc_trajectory_publisher->publish(msg);

  /////////////////////////////
  ///// Evaluate Solution /////
  /////////////////////////////
  // Print the solution
  std::cout << "-----" << std::endl;
  std::cout << "Optimal solution for p = " << solver_args.at("p") << ":" << std::endl;
  std::cout << "Objective: " << res.at("f") << std::endl;

  // Unpack solution into individual time series
  std::unordered_map<std::string, std::vector<double>> state_solution_map = generate_trajectory_map(
    raw_solution_vector);

  if (!plot) {
    return 0;
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
    auto wheel_force = state_solution_map["m1_wheel_force"][k];
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
    plt::ylim(-24.0 * 1 * ghost_util::INCHES_TO_METERS, 1 * 24.0 * ghost_util::INCHES_TO_METERS);


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
          steering_center.y(), WHEEL_DIAM, WHEEL_WIDTH, theta + steering_angle, m_x_points,
          m_y_points);
        plt::plot(
          m_x_points,
          m_y_points,
          std::map<std::string, std::string>{{"color", "black"}});

        // Extract and plot forces
        auto angle = state_solution_map[module_prefix + "steering_angle"][k] + theta;
        auto wheel_force = state_solution_map[module_prefix + "wheel_force"][k];
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

    plot_wheel_module(Eigen::Vector2d(M1_X, M1_Y), "m1_");
    plot_wheel_module(Eigen::Vector2d(M2_X, M2_Y), "m2_");
    plot_wheel_module(Eigen::Vector2d(M3_X, M3_Y), "m3_");
    plot_wheel_module(Eigen::Vector2d(M4_X, M4_Y), "m4_");
    plt::pause(DT);

    if (EXIT_GLOBAL) {
      EXIT_GLOBAL = false;
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
  plt::suptitle("Module 1 Steering");
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

  plt::figure();
  plt::suptitle("Forces");
  plt::subplot(2, 1, 1);
  plt::plot(time_vector, state_solution_map["m2_wheel_force"]);
  plt::subplot(2, 1, 2);
  plt::plot(time_vector, state_solution_map["m2_lateral_force"]);

  plt::pause(0.1);
  plt::show();

  while (!EXIT_GLOBAL) {
    std::this_thread::sleep_for(50ms);
  }

  rclcpp::shutdown();

  return 0;
}
