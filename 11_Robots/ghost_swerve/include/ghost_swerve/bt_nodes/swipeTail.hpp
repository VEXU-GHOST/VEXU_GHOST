#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "ghost_swerve/swerve_model.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ghost_swerve
{

class SwipeTail : public BT::StatefulActionNode
{
public:
  SwipeTail(
    const std::string & name, const BT::NodeConfig & config, std::shared_ptr<rclcpp::Node> node_ptr,
    std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr,
    std::shared_ptr<SwerveModel> swerve_ptr);

  // It is mandatory to define this STATIC method.
  static BT::PortsList providedPorts();

  /// Method called once, when transitioning from the state IDLE.
  /// If it returns RUNNING, this becomes an asynchronous node.
  BT::NodeStatus onStart();

  /// method invoked when the action is already in the RUNNING state.
  BT::NodeStatus onRunning();

  /// when the method halt() is called and the action is RUNNING, this method is invoked.
  /// This is a convenient place todo a cleanup, if needed.
  void onHalted();

private:
  template<typename T>
  T get_input(std::string key);

  std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr_;
  std::shared_ptr<SwerveModel> swerve_ptr_;
  bool started_;
  std::shared_ptr<rclcpp::Node> node_ptr_;
  std::chrono::time_point<std::chrono::system_clock> start_time_;
};

} // namespace ghost_swerve
