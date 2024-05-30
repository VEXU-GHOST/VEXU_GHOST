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

#include "ghost_swerve_mpc_planner/swerve_mpc_planner.hpp"

using namespace std::chrono_literals;
using namespace casadi;
namespace plt = matplotlibcpp;
using ghost_planners::IterationCallback;
using ghost_planners::Trajectory;
using ghost_swerve_mpc_planner::SwerveMPCPlanner;

std::shared_ptr<std::atomic_bool> EXIT_GLOBAL_PTR = std::make_shared<std::atomic_bool>(false);

void siginthandler(int param)
{
  std::cout << "Ctr-C Received, Aborting..." << std::endl;
  *EXIT_GLOBAL_PTR = true;
}


int main(int argc, char * argv[])
{
  signal(SIGINT, siginthandler);

  bool plot = true;
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

  auto state_names = node_ptr->getStateNames();
  ghost_planners::Trajectory init_trajectory(state_names);
  init_trajectory.addNode(0.0, std::vector<double>(state_names.size(), 0.0));

  ghost_planners::Trajectory reference_trajectory(node_ptr->getStateNames());

  std::vector<double> reference(state_names.size(), 0.0);
  reference[reference_trajectory.getStateIndex("base_pose_x")] = -1.0;
  reference[reference_trajectory.getStateIndex("base_pose_y")] = 1.0;
  reference_trajectory.addNode(1.25, reference);

  reference[reference_trajectory.getStateIndex("base_pose_x")] = -1.0;
  reference[reference_trajectory.getStateIndex("base_pose_y")] = -1.0;
  reference_trajectory.addNode(2.5, reference);

  reference[reference_trajectory.getStateIndex("base_pose_x")] = 1.0;
  reference[reference_trajectory.getStateIndex("base_pose_y")] = -1.0;
  reference_trajectory.addNode(3.75, reference);

  reference[reference_trajectory.getStateIndex("base_pose_x")] = 1.0;
  reference[reference_trajectory.getStateIndex("base_pose_y")] = 1.0;
  reference_trajectory.addNode(5.0, reference);

  double current_time = 0.0;
  std::vector<double> current_state{
    1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 3.14159 / 4, 0.0, 0.0, 3.14159 / 4, 0.0, 0.0, 3.14159 / 4, 0.0,
    0.0, 3.14159 / 4, 0.0, 0.0};

  bool track_pose = true;

  node_ptr->runSolver(
    current_time, current_state, init_trajectory, reference_trajectory,
    track_pose);

  auto solution = node_ptr->getLatestSolution();
  auto raw_solution_vector = solution.getFlattenedTrajectory(node_ptr->getTimeVector());
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
  plt::ion();
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

  while (!(*EXIT_GLOBAL_PTR)) {
    std::this_thread::sleep_for(50ms);
  }

  rclcpp::shutdown();

  return 0;
}
