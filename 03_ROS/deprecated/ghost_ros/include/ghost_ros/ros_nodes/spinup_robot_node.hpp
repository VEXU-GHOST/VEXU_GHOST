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

#ifndef GHOST_ROS__ROBOT_STATE_MACHINE_NODE_HPP
#define GHOST_ROS__ROBOT_STATE_MACHINE_NODE_HPP

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "eigen3/Eigen/Geometry"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "ghost_msgs/msg/ghost_robot_state.hpp"
#include "ghost_msgs/msg/v5_actuator_command.hpp"
#include "ghost_msgs/msg/v5_competition_state.hpp"
#include "ghost_msgs/msg/v5_joystick.hpp"
#include "ghost_msgs/msg/v5_sensor_update.hpp"

namespace ghost_ros
{

enum robot_state_e
{
  DISABLED = 0,
  TELEOP = 1,
  AUTONOMOUS = 2,
};

enum teleop_mode_e
{
  SHOOTER_MODE,
  INTAKE_MODE,
  EJECT_MODE,
  TILT_MODE,
  EXPANSION_MODE
};

class SpinUpRobotStateMachineNode : public rclcpp::Node
{
public:
  SpinUpRobotStateMachineNode();

private:
  void updateController();
  void teleop();
  void resetPose();

  void updateSwerveCommandsFromTwist(float angular_velocity, float x_velocity, float y_velocity);
  void updateSwerveVoltageCommandsFromTwist(
    float angular_velocity, float x_velocity,
    float y_velocity);

  void publishSwerveKinematicsVisualization(
    const Eigen::Vector2f & left_wheel_cmd,
    const Eigen::Vector2f & right_wheel_cmd,
    const Eigen::Vector2f & back_wheel_cmd);

  // Publishers
  rclcpp::Publisher<ghost_msgs::msg::V5ActuatorCommand>::SharedPtr actuator_command_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_viz_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;


  // Subscriptions
  rclcpp::Subscription<ghost_msgs::msg::V5CompetitionState>::SharedPtr competition_state_sub_;
  rclcpp::Subscription<ghost_msgs::msg::V5Joystick>::SharedPtr v5_joystick_sub_;
  rclcpp::Subscription<ghost_msgs::msg::GhostRobotState>::SharedPtr robot_state_sub_;
  rclcpp::Subscription<ghost_msgs::msg::V5SensorUpdate>::SharedPtr v5_sensor_update_sub_;

  // Robot States
  robot_state_e curr_robot_state_;
  std::chrono::time_point<std::chrono::system_clock> auton_start_time_;

  // Latest Msgs
  ghost_msgs::msg::V5Joystick::SharedPtr curr_joystick_msg_;
  ghost_msgs::msg::V5SensorUpdate::SharedPtr curr_encoder_msg_;
  ghost_msgs::msg::V5CompetitionState::SharedPtr curr_comp_state_msg_;
  ghost_msgs::msg::GhostRobotState::SharedPtr curr_robot_state_msg_;
  uint32_t curr_robot_state_msg_id_;
  uint32_t curr_encoder_msg_id_;
  uint32_t curr_joystick_msg_id_;
  uint32_t curr_comp_state_msg_id_;
  float max_linear_vel_;
  float max_angular_vel_;
  float max_steering_angular_vel_;
  float steering_kp_;
  float translate_kp_;
  float translate_kd_;
  float rotate_kp_;
  float rotate_kd_;
  float translation_slew_;
  float rotation_slew_;
  float translation_tolerance_;
  float max_motor_rpm_true_;
  float last_ang_vel_cmd_;
  Eigen::Vector2f last_xy_vel_cmd_;
  double pose_reset_x_;
  double pose_reset_y_;
  double pose_reset_theta_;
  float x_goal_;
  float y_goal_;

  Eigen::Vector2f left_wheel_pos_;
  Eigen::Vector2f right_wheel_pos_;
  Eigen::Vector2f back_wheel_pos_;

  ghost_msgs::msg::V5ActuatorCommand actuator_cmd_msg_;

  // Flywheel teleop modes
  bool r1_pressed_;
  teleop_mode_e teleop_mode;
};

} // namespace ghost_ros
#endif // GHOST_ROS__ROBOT_STATE_MACHINE_NODE_HPP
