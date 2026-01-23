#include "ghost_tank/bt_nodes/publishCVTarget.hpp"
#include <cmath>

namespace ghost_tank
{

PublishCVTarget::PublishCVTarget(const std::string& name, const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{
  std::cout << "[PublishCVTarget::PublishCVTarget]" << std::endl;

  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);
  

  // Subscribe to CV target positions from real.py
  // Message format: [rviz_x, rviz_y, obj_id]
  // rviz_x = forward distance (meters), rviz_y = lateral distance (meters, + = left)
  cv_sub_ = node_ptr_->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/object_xy_positions", 10,
    std::bind(&PublishCVTarget::cvCallback, this, std::placeholders::_1));

  
}

BT::PortsList PublishCVTarget::providedPorts()
{
  return {
  //  BT::OutputPort<double>("cv_x", 0.0, "object x position"),
  //  BT::OutputPort<double>("cv_y", 0.0, "object y position"),
  };
}

void PublishCVTarget::cvCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() >= 3) {
    std::lock_guard<std::mutex> lock(target_mutex_);

    int detected_id = static_cast<int>(msg->data[2]);

    // Only update if we're tracking any object (-1) or this specific object
    if (target_object_id_ == -1 || detected_id == target_object_id_) {
      target_y_ = msg->data[0];  // Forward distance
      target_x_ = -msg->data[1];  // Lateral distance (+ = left)
      target_id_ = detected_id;
      has_target_ = true;
      last_detection_time_ = std::chrono::steady_clock::now();

      RCLCPP_INFO(node_ptr_->get_logger(),
        "PublishCVTarget: CV callback - target at x=%.2fm, y=%.2fm, id=%d",
        target_x_, target_y_, target_id_);
    }
  }
}

BT::NodeStatus PublishCVTarget::onStart()
{
  RCLCPP_INFO(node_ptr_->get_logger(), "PublishCVTarget: Started");

  
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    has_target_ = false;
  }

  start_time_ = std::chrono::system_clock::now();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PublishCVTarget::onRunning()
{
  double target_lost_timeout_ms_ = 500;
  // Check timeout
  int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now() - start_time_).count();

  // Get current target data (thread-safe)
  double current_x, current_y;
  bool target_valid;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    current_x = target_x_;
    current_y = target_y_;
    target_valid = has_target_;

    // Check if target detection is stale
    if (has_target_) {
      auto time_since_detection = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_detection_time_).count();
      if (time_since_detection > target_lost_timeout_ms_) {
        target_valid = false;
      }
    }
  }

  if (target_valid) {
    auto time_since_detection = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - last_detection_time_).count();
    // Publish target data
    std::lock_guard<std::mutex> lock(target_mutex_);
    blackboard_->set<double>("cv_x", current_x);
    blackboard_->set<double>("cv_y", current_y);
    return BT::NodeStatus::SUCCESS;
  }

  

  return BT::NodeStatus::RUNNING;
}

void PublishCVTarget::onHalted()
{
 
  resetStatus();
}

} // namespace ghost_tank
