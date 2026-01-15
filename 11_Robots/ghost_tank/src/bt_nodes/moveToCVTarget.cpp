#include "ghost_tank/bt_nodes/moveToCVTarget.hpp"
#include <cmath>

namespace ghost_tank
{

MoveToCVTarget::MoveToCVTarget(const std::string& name, const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{
  std::cout << "[MoveToCVTarget::MoveToCVTarget]" << std::endl;

  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);


  // Using arc_turn_controller for turning, create a simple linear controller
  BT_Util::get_from_blackboard(blackboard_, "arc_turn_controller_ptr", turn_controller_ptr_);

  // Create linear controller with default gains (can be tuned via ports)
  ghost_control::PIDConfig linear_config;
  linear_config.kp = 0.5;
  linear_config.kd = 0.3;
  linear_controller_ptr_ = std::make_shared<ghost_control::PIDController>(linear_config);

  // Subscribe to CV target positions from real.py
  // Message format: [rviz_x, rviz_y, obj_id]
  // rviz_x = forward distance (meters), rviz_y = lateral distance (meters, + = left)
  cv_sub_ = node_ptr_->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/object_xy_positions", 10,
    std::bind(&MoveToCVTarget::cvCallback, this, std::placeholders::_1));

  first_loop_ = true;
}

BT::PortsList MoveToCVTarget::providedPorts()
{
  return {
    BT::InputPort<double>("distance_threshold_m", 0.3, "Distance to target to consider SUCCESS (meters)"),
    BT::InputPort<double>("angle_threshold_deg", 5.0, "Angle error threshold for alignment (degrees)"),
    BT::InputPort<double>("max_linear_speed", 0.5, "Max forward/backward speed (0-1)"),
    BT::InputPort<double>("max_angular_speed", 0.5, "Max turning speed (0-1)"),
    BT::InputPort<int>("timeout_ms", 10000, "Timeout in milliseconds"),
    BT::InputPort<int>("target_lost_timeout_ms", 500, "How long to wait if target lost (ms)"),
    BT::InputPort<int>("target_object_id", -1, "Object ID to track (-1 = any)"),
    BT::InputPort<double>("linear_kp", 0.2, "Linear P gain"),
    BT::InputPort<double>("linear_kd", 0.5, "Linear D gain"),
  };
}

void MoveToCVTarget::cvCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() >= 3) {
    std::lock_guard<std::mutex> lock(target_mutex_);

    int detected_id = static_cast<int>(msg->data[2]);

    // Only update if we're tracking any object (-1) or this specific object
    if (target_object_id_ == -1 || detected_id == target_object_id_) {
      target_x_ = msg->data[0];  // Forward distance
      target_y_ = msg->data[1];  // Lateral distance (+ = left)
      target_id_ = detected_id;
      has_target_ = true;
      last_detection_time_ = std::chrono::steady_clock::now();

      RCLCPP_INFO(node_ptr_->get_logger(),
        "MoveToCVTarget: CV callback - target at x=%.2fm, y=%.2fm, id=%d",
        target_x_, target_y_, target_id_);
    }
  }
}

BT::NodeStatus MoveToCVTarget::onStart()
{
  RCLCPP_INFO(node_ptr_->get_logger(), "MoveToCVTarget: Started");

  // Read parameters from ports
  distance_threshold_m_ = BT_Util::get_input<double>(this, "distance_threshold_m", 0.1);
  angle_threshold_rad_ = BT_Util::get_input<double>(this, "angle_threshold_deg", 5.0) * ghost_util::DEG_TO_RAD;
  max_linear_speed_ = BT_Util::get_input<double>(this, "max_linear_speed", 0.5);
  max_angular_speed_ = BT_Util::get_input<double>(this, "max_angular_speed", 0.5);
  timeout_ms_ = BT_Util::get_input<int>(this, "timeout_ms", 10000);
  target_lost_timeout_ms_ = BT_Util::get_input<int>(this, "target_lost_timeout_ms", 500);
  target_object_id_ = BT_Util::get_input<int>(this, "target_object_id", -1);

  // Update linear controller gains from ports
  double linear_kp = BT_Util::get_input<double>(this, "linear_kp", 0.2);
  double linear_kd = BT_Util::get_input<double>(this, "linear_kd", 0.5);
  ghost_control::PIDConfig linear_config;
  linear_config.kp = linear_kp;
  linear_config.kd = linear_kd;
  linear_controller_ptr_ = std::make_shared<ghost_control::PIDController>(linear_config);

  // Reset state
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    has_target_ = false;
  }

  turn_controller_ptr_->reset();
  linear_controller_ptr_->reset();
  start_time_ = std::chrono::system_clock::now();
  first_loop_ = true;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveToCVTarget::onRunning()
{
  // Check timeout
  int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now() - start_time_).count();

  if (time_elapsed > timeout_ms_) {
    RCLCPP_WARN(node_ptr_->get_logger(), "MoveToCVTarget: Timeout");
    tank_model_ptr_->driveCommandArcade(0.0, 0.0);
    return BT::NodeStatus::FAILURE;
  }

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

  // No target detected - stop and wait
  if (!target_valid) {
    auto time_since_detection = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - last_detection_time_).count();
    RCLCPP_INFO_THROTTLE(node_ptr_->get_logger(), *node_ptr_->get_clock(), 1000,
      "MoveToCVTarget: Waiting for target... (has_target=%d, stale_ms=%ld, timeout_ms=%d)",
      has_target_, time_since_detection, target_lost_timeout_ms_);
    tank_model_ptr_->driveCommandArcade(0.0, 0.0);
    return BT::NodeStatus::RUNNING;
  }

  first_loop_ = false;

  // Calculate distance and angle to target
  // In camera_link frame: x = forward, y = left
  double distance_to_target = std::sqrt(current_x * current_x + current_y * current_y);
  double angle_to_target = std::atan2(current_y, current_x);  // Positive = target is to the left

  // Check success condition
  //double safety_buffer_m = 0.15;
  if (distance_to_target < (distance_threshold_m_)) {
    tank_model_ptr_->driveCommandArcade(0.0, 0.0);
    RCLCPP_INFO(node_ptr_->get_logger(), "MoveToCVTarget: SUCCESS - reached target at %.2fm", distance_to_target);    
    return BT::NodeStatus::SUCCESS;
  }

  // Control strategy:
  // 1. Turn to face target (reduce angle_to_target)
  // 2. Drive forward (reduce distance_to_target)
  // Get robot velocity for derivative term

  double angular_velocity = tank_model_ptr_->getWorldTwist().z();

  double linear_velocity = tank_model_ptr_->getWorldTwist().head<2>().norm();

  // Get robot velocity for derivative term
  // 1. Calculate the turn command (This part is solid)
  double turn_cmd = turn_controller_ptr_->calculateCommand(angle_to_target, -angular_velocity);
  turn_cmd = ghost_util::clamp(turn_cmd, -max_angular_speed_, max_angular_speed_);

  // 2. Calculate the BASE forward command using current_x (the Forward component)
  // Negative current_x makes the error (0 - (-current_x)) positive
  //double base_fwd = linear_controller_ptr_->calculateCommand(-current_x, -linear_velocity);

  double base_fwd = linear_controller_ptr_->calculateCommand(current_x, -linear_velocity);
  // 3. Apply the Speed Ceiling (Hardware governor)
  // As current_x approaches 0, max possible speed approaches 0.
  double speed_limit = std::abs(current_x) * 0.4; // Try 0.3 if still too fast
  double capped_fwd = ghost_util::clamp(base_fwd, -speed_limit, speed_limit);

  // 4. Apply the Alignment Factor 
  // This ensures we only give it gas if we are facing the right way
  double alignment_factor = std::cos(angle_to_target);
  alignment_factor = std::max(0.0, alignment_factor); // Don't drive backwards if facing away

  // 5. Final Command
  double fwd_cmd = capped_fwd * alignment_factor;
  fwd_cmd = ghost_util::clamp(fwd_cmd, -max_linear_speed_, max_linear_speed_);

  // 6. Hard Stop Safety
  // If we are super close, just kill the fwd_cmd entirely to let momentum carry us
  if (distance_to_target < (distance_threshold_m_)) {
    fwd_cmd = 0.0;
  }

  // Debug logging
  RCLCPP_INFO_THROTTLE(node_ptr_->get_logger(), *node_ptr_->get_clock(), 200,
    "MoveToCVTarget: target=(%.2f, %.2f)m, dist=%.2fm, angle=%.1fdeg, fwd=%.2f, turn=%.2f",
    current_x, current_y, distance_to_target, angle_to_target * ghost_util::RAD_TO_DEG, fwd_cmd, turn_cmd);

  // Send drive command
  tank_model_ptr_->driveCommandArcade(-fwd_cmd, turn_cmd);

  return BT::NodeStatus::RUNNING;
}

void MoveToCVTarget::onHalted()
{
  RCLCPP_INFO(node_ptr_->get_logger(), "MoveToCVTarget: Halted");
  tank_model_ptr_->driveCommandArcade(0.0, 0.0);
  resetStatus();
}

} // namespace ghost_tank
