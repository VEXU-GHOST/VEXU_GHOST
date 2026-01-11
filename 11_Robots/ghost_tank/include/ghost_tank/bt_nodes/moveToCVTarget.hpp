#pragma once

#include <atomic>
#include <chrono>
#include <mutex>

#include "behaviortree_cpp/behavior_tree.h"
#include "ghost_tank/bt_nodes/bt_util.hpp"
#include "ghost_tank/tank_model.hpp"
#include "ghost_util/angle_util.hpp"
#include "ghost_util/unit_conversion_utils.hpp"
#include "ghost_control/pid_controller.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace ghost_tank
{

class MoveToCVTarget : public BT::StatefulActionNode
{
public:
  MoveToCVTarget(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  // Core pointers from blackboard
  std::shared_ptr<rclcpp::Node> node_ptr_;
  std::shared_ptr<TankModel> tank_model_ptr_;
  BT::Blackboard::Ptr blackboard_;

  // Subscription to CV target positions
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cv_sub_;

  // Target data (updated by subscription callback)
  std::mutex target_mutex_;
  double target_x_{0.0};  // Forward distance in meters (camera_link frame)
  double target_y_{0.0};  // Lateral distance in meters (positive = left)
  int target_id_{-1};
  bool has_target_{false};
  std::chrono::time_point<std::chrono::steady_clock> last_detection_time_;

  // Control
  std::shared_ptr<ghost_control::PIDController> turn_controller_ptr_;
  std::shared_ptr<ghost_control::PIDController> linear_controller_ptr_;

  // Timing
  std::chrono::time_point<std::chrono::system_clock> start_time_;
  bool first_loop_{true};

  // Parameters (from ports)
  double distance_threshold_m_{0.0};
  double angle_threshold_rad_{0.0};
  double max_linear_speed_{0.0};
  double max_angular_speed_{0.0};
  int timeout_ms_{0};
  int target_lost_timeout_ms_{500};  // How long to wait if target disappears
  int target_object_id_{-1};  // Which object ID to track (-1 = any)

  // Subscription callback
  void cvCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
};

} // namespace ghost_tank
