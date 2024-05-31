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

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <unordered_map>
#include <atomic>
#include <mutex>
#include <thread>
#include <csignal>

#include <ghost_util/unit_conversion_utils.hpp>

#include <ghost_msgs/msg/labeled_vector.hpp>
#include <ghost_msgs/msg/labeled_vector_map.hpp>

#include "eigen3/Eigen/Geometry"

#include <ghost_ros_interfaces/msg_helpers/msg_helpers.hpp>
#include <rclcpp/rclcpp.hpp>

#include "matplotlibcpp.h"
#include "yaml.h"

#include <ghost_planners/trajectory.hpp>

using namespace std::chrono_literals;
using ghost_ros_interfaces::msg_helpers::fromROSMsg;
namespace plt = matplotlibcpp;

std::shared_ptr<std::atomic_bool> EXIT_GLOBAL_PTR = std::make_shared<std::atomic_bool>(false);

void siginthandler(int param)
{
  std::cout << "Ctr-C Received, Aborting..." << std::endl;
  *EXIT_GLOBAL_PTR = true;
}

int main(int argc, char * argv[])
{
  signal(SIGINT, siginthandler);
  rclcpp::init(argc, argv);

  std::shared_ptr<std::deque<ghost_msgs::msg::LabeledVectorMap>> traj_map_buffer_ptr =
    std::make_shared<std::deque<ghost_msgs::msg::LabeledVectorMap>>();

  std::shared_ptr<std::mutex> callback_mutex_ptr = std::make_shared<std::mutex>();

  auto node_ptr = std::make_shared<rclcpp::Node>("swerve_animator_node");

  auto traj_sub = node_ptr->create_subscription<ghost_msgs::msg::LabeledVectorMap>(
    "/trajectory/swerve_mpc_trajectory_intermediates", 10,
    [&](const ghost_msgs::msg::LabeledVectorMap::SharedPtr msg) {
      std::unique_lock lock(*callback_mutex_ptr);
      traj_map_buffer_ptr->push_front(*msg);
    });

  node_ptr->declare_parameter("skip_factor", 10);
  int skip_factor = node_ptr->get_parameter("skip_factor").as_int();

  node_ptr->declare_parameter("speedup_factor", 1.0);
  double speedup_factor = node_ptr->get_parameter("speedup_factor").as_double();

  std::thread node_thread([&]() {
      rclcpp::spin(node_ptr);
    });

  plt::ion();
  plt::figure();
  plt::plot();

  int curr_iteration = 0;
  int iteration_counter = 0;

  while (!(*EXIT_GLOBAL_PTR)) {
    if (!traj_map_buffer_ptr->empty()) {
      // Retrieve data from queue
      std::unique_lock<std::mutex> lock(*callback_mutex_ptr);
      auto latest_msg = traj_map_buffer_ptr->back();
      traj_map_buffer_ptr->pop_back();
      iteration_counter++;
      for (int i = 0; i < skip_factor - 1; i++) {
        traj_map_buffer_ptr->pop_back();
        iteration_counter++;
      }
      lock.unlock();

      std::unordered_map<std::string, std::vector<double>> trajectory_map;
      fromROSMsg(trajectory_map, latest_msg);

      //////////////////////////////
      ///// Animate Trajectory /////
      //////////////////////////////

      // Get Data
      const int NUM_KNOTS = trajectory_map.at("time").size();
      auto time_vector = trajectory_map.at("time");
      const double DT = 0.05;
      const double CHASSIS_WIDTH = 16.5 * ghost_util::INCHES_TO_METERS;
      const double WHEELBASE_WIDTH = 12.5 * ghost_util::INCHES_TO_METERS;
      const double WHEEL_WIDTH = 1.0 * ghost_util::INCHES_TO_METERS;
      const double WHEEL_RADIUS = 2.75 / 2.0 * ghost_util::INCHES_TO_METERS;

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
        x_pos.push_back(trajectory_map["base_pose_x"][k]);
        y_pos.push_back(trajectory_map["base_pose_y"][k]);
        x_vel.push_back(trajectory_map["base_vel_x"][k]);
        y_vel.push_back(trajectory_map["base_vel_y"][k]);

        x_angle_components.push_back(cos(trajectory_map["base_pose_theta"][k]));
        y_angle_components.push_back(sin(trajectory_map["base_pose_theta"][k]));

        auto angle = trajectory_map["m1_steering_angle"][k];
        auto wheel_force = trajectory_map["m1_wheel_torque"][k] / WHEEL_RADIUS;
        auto lateral_force = trajectory_map["m1_lateral_force"][k];
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

      int speed_up_knots = NUM_KNOTS / speedup_factor;
      for (int k_i = 0; k_i < speed_up_knots; k_i++) {
        int k = k_i * speedup_factor;
        // std::cout << 100.0 * k / NUM_KNOTS << "%" << std::endl;

        plt::clf();
        double x = trajectory_map["base_pose_x"][k];
        double y = trajectory_map["base_pose_y"][k];
        double theta = trajectory_map["base_pose_theta"][k];

        // Plot Chassis
        std::vector<double> chassis_x_points;
        std::vector<double> chassis_y_points;
        plotRectanglePoints(
          x, y, CHASSIS_WIDTH, CHASSIS_WIDTH, theta, chassis_x_points,
          chassis_y_points);

        plt::axis("scaled");
        plt::xlim(
          -24.0 * 3 * ghost_util::INCHES_TO_METERS,
          3 * 24.0 * ghost_util::INCHES_TO_METERS);
        plt::ylim(
          -24.0 * 3 * ghost_util::INCHES_TO_METERS,
          3 * 24.0 * ghost_util::INCHES_TO_METERS);

        plt::plot(
          chassis_x_points,
          chassis_y_points,
          std::map<std::string, std::string>{{"color", "black"}});

        // Plot Base Linear Velocity
        plt::plot(
          std::vector<double>{x, x + trajectory_map["base_vel_x"][k] / 10.0},
          std::vector<double>{y, y + trajectory_map["base_vel_y"][k] / 10.0},
          std::map<std::string, std::string>{{"color", "green"}});

        plt::plot(
          std::vector<double>{x, x + trajectory_map["base_accel_x"][k] / 10.0},
          std::vector<double>{y, y + trajectory_map["base_accel_y"][k] / 10.0},
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
            double steering_angle = trajectory_map[module_prefix + "steering_angle"][k];
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
            auto angle = trajectory_map[module_prefix + "steering_angle"][k] + theta;
            auto wheel_force = trajectory_map[module_prefix + "wheel_torque"][k] / WHEEL_RADIUS;
            auto lateral_force = trajectory_map[module_prefix + "lateral_force"][k];

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

        plot_wheel_module(Eigen::Vector2d(WHEELBASE_WIDTH / 2.0, WHEELBASE_WIDTH / 2.0), "m1_");
        plot_wheel_module(Eigen::Vector2d(-WHEELBASE_WIDTH / 2.0, WHEELBASE_WIDTH / 2.0), "m2_");
        plot_wheel_module(Eigen::Vector2d(-WHEELBASE_WIDTH / 2.0, -WHEELBASE_WIDTH / 2.0), "m3_");
        plot_wheel_module(Eigen::Vector2d(WHEELBASE_WIDTH / 2.0, -WHEELBASE_WIDTH / 2.0), "m4_");

        plt::title(std::string("Iteration ") + std::to_string(curr_iteration));
        plt::pause(DT);
      }
      curr_iteration = iteration_counter;
    }
    std::this_thread::sleep_for(1ms);
  }

  rclcpp::shutdown();
  return 0;
}
