#pragma once

#include <chrono>
#include <math.h>
#include <string>
#include "behaviortree_cpp/behavior_tree.h"
#include "ghost_tank/bt_nodes/bt_util.hpp"
#include "ghost_tank/tank_model.hpp"
#include "ghost_util/angle_util.hpp"
#include "ghost_util/math_util.hpp"
#include "ghost_util/unit_conversion_utils.hpp"
#include "rclcpp/rclcpp.hpp"

#include <ghost_control/pid_controller.hpp>

namespace ghost_tank {

/**
 * SwingToAngle - Swing turn to a world heading: one side of the drive holds
 * still (locked_side = "left" or "right") and the other side drives, so the
 * robot pivots around the locked wheels.
 *
 * angle_deg is the field heading (0 = +x, 90 = +y, CCW positive), the same frame
 * as init_theta_degrees. Uses the arc_turn PID (same sign convention as
 * TurnToPoint / TurnToAngle) and exits once the heading has stayed within
 * angle_exit_threshold_deg for angle_settle_ms, or on timeout_ms. Mirroring
 * flips both the heading and the locked side.
 */
class SwingToAngle : public BT::StatefulActionNode {
public:
  SwingToAngle(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart();
  BT::NodeStatus onRunning();
  void onHalted();

private:
  std::shared_ptr<TankModel> tank_model_ptr_;
  std::shared_ptr<ghost_control::PIDController> m_arc_turn_controller_ptr;
  BT::Blackboard::Ptr blackboard_;

  std::chrono::time_point<std::chrono::system_clock> start_time_;
  double des_ang_rad;
  bool lock_left;
  int timeout_ms;
  double angle_exit_threshold_rad;
  int angle_settle_ms;
  double max_effort;
  bool angle_settling_{false};
  std::chrono::time_point<std::chrono::system_clock> angle_settle_start_;
};

} // namespace ghost_tank
