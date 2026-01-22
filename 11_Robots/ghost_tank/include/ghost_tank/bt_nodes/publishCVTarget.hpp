#pragma once

#include <chrono>
#include <mutex>

#include "behaviortree_cpp/behavior_tree.h"
#include "ghost_tank/bt_nodes/bt_util.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace ghost_tank
{

class PublishCVTarget : public BT::StatefulActionNode
{
public:
  PublishCVTarget(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  // Core pointers from blackboard
  std::shared_ptr<rclcpp::Node> node_ptr_;
  BT::Blackboard::Ptr blackboard_;

  // Subscription to CV target positions
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cv_sub_;

  // Target data (updated by subscription callback)
  std::mutex target_mutex_;
  double target_x_{0.0};
  double target_y_{0.0};
  int target_id_{-1};
  bool has_target_{false};
  std::chrono::time_point<std::chrono::steady_clock> last_detection_time_;

  // Timing
  std::chrono::time_point<std::chrono::system_clock> start_time_;

  // Parameters
  int target_object_id_{-1};  // Which object ID to track (-1 = any)

  // Subscription callback
  void cvCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
};

} // namespace ghost_tank
