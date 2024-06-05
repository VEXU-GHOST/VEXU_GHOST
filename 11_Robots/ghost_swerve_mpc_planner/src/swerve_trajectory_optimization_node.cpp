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

  bool plot = false;
  rclcpp::init(argc, argv);

  auto node_ptr = std::make_shared<SwerveMPCPlanner>();
  std::thread node_thread([&]() {
      rclcpp::spin(node_ptr);
    });

  std::thread exit_thread([&]() {
      while (!(*EXIT_GLOBAL_PTR)) {
        std::this_thread::sleep_for(50ms);
      }
      node_ptr->exitEarly();
    });

  auto state_names = node_ptr->getStateNames();
  ghost_planners::Trajectory init_trajectory(state_names);
  auto init_node = std::vector<double>(state_names.size(), 0.0);
  init_node[init_trajectory.getStateIndex("base_pose_x")] = 1.0;
  init_node[init_trajectory.getStateIndex("base_pose_y")] = 1.0;
  init_trajectory.addNode(0.0, init_node);

  ghost_planners::Trajectory reference_trajectory(state_names);

  std::vector<double> reference_node(state_names.size(), 0.0);

  // reference_node[reference_trajectory.getStateIndex("base_pose_x")] = 0.0;
  // reference_node[reference_trajectory.getStateIndex("base_pose_y")] = 1.0;
  // reference_node[reference_trajectory.getStateIndex("base_pose_theta")] = -3.14159;
  // reference_node[reference_trajectory.getStateIndex("base_vel_x")] = -2 * 2.0 / 6.0;
  // reference_node[reference_trajectory.getStateIndex("base_vel_y")] = 0.0;
  // reference_node[reference_trajectory.getStateIndex("base_vel_theta")] = -2 * 2 * 3.14159 / 6.0;
  // reference_trajectory.addNode(3.0, reference_node);
  // init_trajectory.addNode(3.0, reference_node);

  // reference_node[reference_trajectory.getStateIndex("base_pose_x")] = -1.0;
  // reference_node[reference_trajectory.getStateIndex("base_pose_y")] = 1.0;
  // reference_node[reference_trajectory.getStateIndex("base_pose_theta")] = -2 * 3.14159;
  // reference_node[reference_trajectory.getStateIndex("base_vel_x")] = 0.0;
  // reference_node[reference_trajectory.getStateIndex("base_vel_y")] = 0.0;
  // reference_node[reference_trajectory.getStateIndex("base_vel_theta")] = 0.0;
  // reference_trajectory.addNode(6.0, reference_node);
  // init_trajectory.addNode(6.0, reference_node);

  reference_node[reference_trajectory.getStateIndex("base_pose_x")] = 0.0;
  reference_node[reference_trajectory.getStateIndex("base_pose_y")] = 1.0;
  reference_node[reference_trajectory.getStateIndex("base_pose_theta")] = 3.14159 / 8;
  reference_node[reference_trajectory.getStateIndex("base_vel_x")] = -2 * 2.0 / 1.5;
  reference_node[reference_trajectory.getStateIndex("base_vel_y")] = 0.0;
  reference_node[reference_trajectory.getStateIndex("base_vel_theta")] = 2 * 3.14159 / 4 / 1.5;
  reference_trajectory.addNode(0.75, reference_node);
  init_trajectory.addNode(0.75, reference_node);

  reference_node[reference_trajectory.getStateIndex("base_pose_x")] = -1.0;
  reference_node[reference_trajectory.getStateIndex("base_pose_y")] = 1.0;
  reference_node[reference_trajectory.getStateIndex("base_pose_theta")] = 3.14159 / 4;
  reference_node[reference_trajectory.getStateIndex("base_vel_x")] = 0.0;
  reference_node[reference_trajectory.getStateIndex("base_vel_y")] = 0.0;
  reference_node[reference_trajectory.getStateIndex("base_vel_theta")] = 0.0;
  reference_trajectory.addNode(1.5, reference_node);
  init_trajectory.addNode(1.5, reference_node);

  reference_node[reference_trajectory.getStateIndex("base_pose_x")] = -1.0;
  reference_node[reference_trajectory.getStateIndex("base_pose_y")] = 0.0;
  reference_node[reference_trajectory.getStateIndex("base_pose_theta")] = 3 * 3.14159 / 8;
  reference_node[reference_trajectory.getStateIndex("base_vel_x")] = 0.0;
  reference_node[reference_trajectory.getStateIndex("base_vel_y")] = -2 * 2.0 / 1.5;
  reference_node[reference_trajectory.getStateIndex("base_vel_theta")] = 2 * 3.14159 / 4 / 1.5;
  reference_trajectory.addNode(2.25, reference_node);
  init_trajectory.addNode(2.25, reference_node);

  reference_node[reference_trajectory.getStateIndex("base_pose_x")] = -1.0;
  reference_node[reference_trajectory.getStateIndex("base_pose_y")] = -1.0;
  reference_node[reference_trajectory.getStateIndex("base_pose_theta")] = 3.14159 / 2;
  reference_node[reference_trajectory.getStateIndex("base_vel_x")] = 0.0;
  reference_node[reference_trajectory.getStateIndex("base_vel_y")] = 0.0;
  reference_node[reference_trajectory.getStateIndex("base_vel_theta")] = 0.0;
  reference_trajectory.addNode(3.0, reference_node);
  init_trajectory.addNode(3.0, reference_node);

  reference_node[reference_trajectory.getStateIndex("base_pose_x")] = 0.0;
  reference_node[reference_trajectory.getStateIndex("base_pose_y")] = -1.0;
  reference_node[reference_trajectory.getStateIndex("base_pose_theta")] = 5 * 3.14159 / 8;
  reference_node[reference_trajectory.getStateIndex("base_vel_x")] = 2 * 2.0 / 1.5;
  reference_node[reference_trajectory.getStateIndex("base_vel_y")] = 0.0;
  reference_node[reference_trajectory.getStateIndex("base_vel_theta")] = 2 * 3.14159 / 4 / 1.5;
  reference_trajectory.addNode(3.75, reference_node);
  init_trajectory.addNode(3.75, reference_node);

  reference_node[reference_trajectory.getStateIndex("base_pose_x")] = 1.0;
  reference_node[reference_trajectory.getStateIndex("base_pose_y")] = -1.0;
  reference_node[reference_trajectory.getStateIndex("base_pose_theta")] = 3 * 3.14159 / 4;
  reference_node[reference_trajectory.getStateIndex("base_vel_x")] = 0.0;
  reference_node[reference_trajectory.getStateIndex("base_vel_y")] = 0.0;
  reference_node[reference_trajectory.getStateIndex("base_vel_theta")] = 0.0;
  reference_trajectory.addNode(4.5, reference_node);
  init_trajectory.addNode(4.5, reference_node);

  reference_node[reference_trajectory.getStateIndex("base_pose_x")] = 1.0;
  reference_node[reference_trajectory.getStateIndex("base_pose_y")] = 0.0;
  reference_node[reference_trajectory.getStateIndex("base_pose_theta")] = 5 * 3.14159 / 8;
  reference_node[reference_trajectory.getStateIndex("base_vel_x")] = 0.0;
  reference_node[reference_trajectory.getStateIndex("base_vel_y")] = 2 * 2.0 / 1.5;
  reference_node[reference_trajectory.getStateIndex("base_vel_theta")] = -2 * 3.14159 / 4 / 1.5;
  reference_trajectory.addNode(5.25, reference_node);
  init_trajectory.addNode(5.25, reference_node);

  reference_node[reference_trajectory.getStateIndex("base_pose_x")] = 1.0;
  reference_node[reference_trajectory.getStateIndex("base_pose_y")] = 1.0;
  reference_node[reference_trajectory.getStateIndex("base_pose_theta")] = 3.14159 / 2.0;
  reference_node[reference_trajectory.getStateIndex("base_vel_x")] = 0.0;
  reference_node[reference_trajectory.getStateIndex("base_vel_y")] = 0.0;
  reference_node[reference_trajectory.getStateIndex("base_vel_theta")] = 0.0;
  reference_trajectory.addNode(6.0, reference_node);
  init_trajectory.addNode(6.0, reference_node);

  double current_time = 0.0;
  std::vector<double> current_state{
    1.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    3.14159 / 4, 0.0, 0.0,
    3.14159 / 4, 0.0, 0.0,
    3.14159 / 4, 0.0, 0.0,
    3.14159 / 4, 0.0, 0.0};

  bool track_pose = true;
  bool generate_dense_trajectory = false;

  node_ptr->runSolver(
    current_time, current_state, init_trajectory, reference_trajectory,
    track_pose, generate_dense_trajectory);

  auto solution = node_ptr->getLatestSolution();
  auto raw_solution_vector = solution.getFlattenedTrajectory(node_ptr->getTimeVector());

  /////////////////////////////
  ///// Evaluate Solution /////
  /////////////////////////////
  // Unpack solution into individual time series
  std::unordered_map<std::string,
    std::vector<double>> state_solution_map = node_ptr->generateTrajectoryMap(
    raw_solution_vector);

  while (!(*EXIT_GLOBAL_PTR)) {
    std::this_thread::sleep_for(50ms);
  }

  rclcpp::shutdown();

  return 0;
}
