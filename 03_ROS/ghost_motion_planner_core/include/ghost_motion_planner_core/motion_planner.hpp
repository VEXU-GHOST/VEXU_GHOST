#pragma once

#include <rclcpp/rclcpp.hpp>

#include "ghost_msgs/msg/drivetrain_command.hpp"
#include "ghost_msgs/msg/robot_trajectory.hpp"
#include "ghost_msgs/msg/v5_sensor_update.hpp"
#include "ghost_util/angle_util.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <ghost_v5_interfaces/robot_hardware_interface.hpp>
#include <ghost_v5_interfaces/util/device_config_factory_utils.hpp>

namespace ghost_motion_planner
{

class MotionPlanner
{
public:
  MotionPlanner() = default;
  virtual ~MotionPlanner() = default;

  ///////////////////////////
  ///// Virtual Methods /////
  ///////////////////////////
  /**
   * @brief Called directly after instantiation, when the robot is configured.
   *
   * Member variables provide access to ROS Node for adding ROS Interfaces (topics, services, etc.) and for loading
   * ROS Params at runtime.
   *
   * This will block any other competition functionality until it completes.
   * This method will not update actuator commands.
   */
  virtual void initialize() = 0;

  // Blocking, ignore sensor updates while making new plan
  // overload for teleop and auton?
  /**
   * @brief Called when a DrivetrainCommand msg is recieved
   *
   * Generates and publishes a RobotTrajectory msg when completed
   *
   */
  virtual void generateMotionPlan(const ghost_msgs::msg::DrivetrainCommand::SharedPtr cmd) = 0;


  //////////////////////////////
  ///// Base Class Methods /////
  //////////////////////////////

  /**
   * @brief Called for all motion planner classes after construction. Calls user-defined intialize method internally.
   */
  void configure(std::string node_name);

  /**
   * @brief Returns a shared pointer to the ROS node for this robot instance
   *
   * @return std::shared_ptr<rclcpp::Node>
   */
  std::shared_ptr<rclcpp::Node> getROSNodePtr() const
  {
    if (!configured_) {
      throw std::runtime_error(
              "[motion_planner::getROSNodePtr] Error: This plugin has not been configured yet!");
    }
    return node_ptr_;
  }

protected:
  std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> robot_hardware_interface_ptr_;
  rclcpp::Publisher<ghost_msgs::msg::RobotTrajectory>::SharedPtr trajectory_pub_;
  std::shared_ptr<rclcpp::Node> node_ptr_;
  double current_x_ = 0.0;
  double current_y_ = 0.0;
  double current_x_vel_ = 0.0;
  double current_y_vel_ = 0.0;
  double current_theta_rad_ = 0.0;
  double current_theta_vel_rad_ = 0.0;

private:
  void sensorUpdateCallback(const ghost_msgs::msg::V5SensorUpdate::SharedPtr msg);
  void loadRobotHardwareInterface();
  void setNewCommand(const ghost_msgs::msg::DrivetrainCommand::SharedPtr cmd);
  void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg);

  bool configured_ = false;
  std::atomic_bool planning_ = false;
  rclcpp::Subscription<ghost_msgs::msg::V5SensorUpdate>::SharedPtr sensor_update_sub_;
  rclcpp::Subscription<ghost_msgs::msg::DrivetrainCommand>::SharedPtr pose_command_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

} // namespace ghost_motion_planner
