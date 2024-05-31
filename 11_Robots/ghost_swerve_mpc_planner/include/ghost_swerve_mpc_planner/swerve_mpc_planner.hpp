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
#pragma once

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

#include <ghost_planners/trajectory.hpp>

#include "ghost_planners/ipopt_iteration_callback.hpp"

namespace ghost_swerve_mpc_planner
{

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
    double robot_inertia;
    double wheel_constraint_tolerance;
  };

  SwerveMPCPlanner();

  void loadConfig();

  void validateConfig();

  void initROS();

  void generateStateNames();

  void generateParameterNames();

  void populateContainers();

  double getWeight(std::string name);

  // Shorthand to get symbolic state variable by name
  casadi::Matrix<casadi::SXElem> getState(std::string name, int k);

  // Shorthand to get symbolic state variable by name
  casadi::Matrix<casadi::SXElem> getState(std::string name);

  casadi::Matrix<casadi::SXElem> getParam(std::string name);

  static std::string getKnotPrefix(int i)
  {
    return "k" + std::to_string(i) + "_";
  }

  static casadi::DM convertVectorToDM(std::vector<double> vector);

  casadi::Matrix<casadi::SXElem> getQuadraticTrackingCost(std::string state, int k);

  casadi::Matrix<casadi::SXElem> getJerkCost(std::string state, int k, double cost);

  void rotateVectorSymbolic(
    const Matrix<SXElem> & angle,
    const Matrix<SXElem> & x_in,
    const Matrix<SXElem> & y_in,
    Matrix<SXElem> & x_out,
    Matrix<SXElem> & y_out);

  void rotateVectorDouble(

    const Matrix<SXElem> & angle,
    const double & x_in,
    const double & y_in,
    Matrix<SXElem> & x_out,
    Matrix<SXElem> & y_out);

  std::unordered_map<std::string, std::vector<double>> generateTrajectoryMap(
    const std::vector<double> & solution_vector);

  void addIntegrationConstraints();

  void addInitialStateConstraints();

  void addAccelerationDynamicsConstraints();

  void addNoWheelSlipConstraints();

  void addConstraints();

  void setStateBounds();

  void addCosts();

  void initSolver();

  void runSolver(
    double current_time,
    const std::vector<double> & current_state,
    const ghost_planners::Trajectory & x0,
    const ghost_planners::Trajectory & reference_trajectory,
    bool pose_tracking = false);

  const Eigen::Vector2d & getModulePosition(int m)
  {
    if (m <= config_.num_swerve_modules) {
      return module_positions_[m - 1];
    } else {
      throw std::runtime_error(
              "[SwerveMPCPlanner::getModulePosition] Module index cannot exceed the number of swerve modules!");
    }
  }

  void exitEarly()
  {
    *exit_flag_ptr_ = true;
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

  std::shared_ptr<std::deque<ghost_planners::IterationCallback::IPOPTOutput>> getCallbackDataBuffer()
  {
    return callback_data_buffer_;
  }
  std::shared_ptr<std::mutex> getCallbackDataMutex()
  {
    return callback_data_mutex_;
  }

  const std::vector<std::string> & getStateNames() const
  {
    return state_names_;
  }

  const std::vector<std::string> & getParamNames() const
  {
    return param_names_;
  }

  ghost_planners::Trajectory getLatestSolution()
  {
    std::unique_lock<std::mutex> lock(latest_solution_mutex_);
    return *latest_solution_ptr_;
  }

private:
  void startSolverThread();

  // Constants
  const double I2M = ghost_util::INCHES_TO_METERS;
  std::vector<Eigen::Vector2d> module_positions_;
  std::vector<std::string> state_names_;
  std::vector<std::string> param_names_;

  // Config
  Config config_;
  int num_knots_;
  int num_opt_vars_;

  // ROS Publishers
  rclcpp::Publisher<ghost_msgs::msg::LabeledVectorMap>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<ghost_msgs::msg::LabeledVectorMap>::SharedPtr intermediate_trajectory_publisher_;
  rclcpp::Publisher<ghost_msgs::msg::IPOPTOutput>::SharedPtr ipopt_output_publisher_;
  bool publish_intermediate_solutions_ = false;

  // Index Helpers
  std::unordered_map<std::string, int> state_index_map_;
  std::unordered_map<std::string, int> param_index_map_;
  std::unordered_map<std::string, int> weight_index_map_;

  // Containers
  casadi::Matrix<casadi::SXElem> state_vector_;
  casadi::Matrix<casadi::SXElem> param_vector_;
  std::vector<double> weights_;

  casadi::Matrix<casadi::SXElem> cost_;
  casadi::DM lbx_;
  casadi::DM ubx_;

  casadi::Matrix<casadi::SXElem> constraints_;
  casadi::DM lbg_;
  casadi::DM ubg_;

  std::vector<double> time_vector_;

  // Solver
  casadi::Function solver_;
  std::shared_ptr<std::atomic_bool> exit_flag_ptr_;
  std::shared_ptr<ghost_planners::Trajectory> latest_solution_ptr_;
  std::vector<double> latest_solution_vector_;
  std::mutex latest_solution_mutex_;
  std::atomic_bool solver_active_;
  std::map<std::string, DM> solver_args;

  // Callback
  std::shared_ptr<std::deque<ghost_planners::IterationCallback::IPOPTOutput>> callback_data_buffer_;
  std::shared_ptr<std::mutex> callback_data_mutex_;
  std::shared_ptr<ghost_planners::IterationCallback> iteration_callback_;
  std::thread callback_thread_;
};

} // namespace ghost_swerve_mpc_planner
