/*
 *   Copyright (c) 2024 Jake Wendling
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

#include "ghost_tank/bt_nodes/intakeCmd.hpp"

namespace ghost_tank
{

// SyncActionNode (synchronous action) with an input port.
// If your Node has ports, you must use this constructor signature
IntakeCmd::IntakeCmd(
  const std::string & name, const BT::NodeConfig & config)
: BT::DecoratorNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "rhi_ptr", rhi_ptr_);

  double conveyor_num_links = node_ptr_->get_parameter("tank_robot_plugin.conveyor_num_links").as_double();
  double conveyor_sprocket_teeth = node_ptr_->get_parameter("tank_robot_plugin.conveyor_sprocket_teeth").as_double();
  double conveyor_num_hooks = node_ptr_->get_parameter("tank_robot_plugin.conveyor_num_hooks").as_double();
  m_conveyor_ticks_per_loop = 360.0 * conveyor_num_links / conveyor_sprocket_teeth;
  m_conveyor_ticks_per_hook = m_conveyor_ticks_per_loop / conveyor_num_hooks;

  m_conveyor_hook_align_threshold = node_ptr_->get_parameter("tank_robot_plugin.conveyor_hook_align_threshold").as_double();
  m_conveyor_hook_align_power = node_ptr_->get_parameter("tank_robot_plugin.conveyor_hook_align_power").as_double();

  m_conveyor_hook_throw_fraction = node_ptr_->get_parameter("tank_robot_plugin.conveyor_hook_throw_fraction").as_double();
  m_conveyor_hook_throw_duration = node_ptr_->get_parameter("tank_robot_plugin.conveyor_hook_throw_duration").as_double();
}

// It is mandatory to define this STATIC method.
BT::PortsList IntakeCmd::providedPorts()
{
  // This action has a single input port called "message"
  return {
    BT::InputPort<bool>("lower"),
    BT::InputPort<bool>("hook"),
  };
}

void IntakeCmd::halt()
{
  haltChild();
}

// Override the virtual function tick()
BT::NodeStatus IntakeCmd::tick()
{
  bool lower = BT_Util::get_input<bool>(this, "lower");
  bool hook = BT_Util::get_input<bool>(this, "hook");

  updateIntake(lower, hook);

  switch (child()->executeTick()) {
    case BT::NodeStatus::SUCCESS:
      haltChild();
      break;
    case BT::NodeStatus::FAILURE:
      haltChild();
      return BT::NodeStatus::FAILURE;
    case BT::NodeStatus::RUNNING:
      return BT::NodeStatus::RUNNING;
    default:
      throw BT::LogicError("A child node must never return IDLE");
  }

  return BT::NodeStatus::SUCCESS;
}

void IntakeCmd::updateIntake(bool lower, bool hook)
{
  // Manual Ground Pickup control
  double ground_pickup_power = 0;
  int32_t ground_pickup_current = 0;
  static bool conveyor_hook_is_aligned = false;
  static double conveyor_last_aligned_position = 0.0;

  double conveyor_position_abs = rhi_ptr_->getMotorPosition("conveyor_motor");
  double conveyor_position_rel = std::fmod(conveyor_position_abs, m_conveyor_ticks_per_loop);
  conveyor_position_rel += (conveyor_position_rel < 0.0) ? m_conveyor_ticks_per_loop : 0.0;
  double hook_fraction = std::fmod(conveyor_position_rel, m_conveyor_ticks_per_hook) / m_conveyor_ticks_per_hook;

  if (lower) {
    ground_pickup_power = 1.0;
    ground_pickup_current = 2500;
  } /*else if (R) {
    ground_pickup_power = -1.0;
    ground_pickup_current = 2500;
  }*/ /*else {
    ground_pickup_power = 0.0;
    ground_pickup_current = 0;
  }*/

  // Conveyor control
  // We assume any manual conveyor control misaligns the hooks
  double conveyor_power = 0;
  int32_t conveyor_current = 0;
  if (hook) {
    conveyor_power = 1.0;
    conveyor_current = 2500;
    conveyor_hook_is_aligned = false;
  } /*else if (L1 && !lower) {
    conveyor_power = -1.0;
    conveyor_current = 2500;
    conveyor_hook_is_aligned = false;
  } else {
    conveyor_power = 0.0;
    conveyor_current = 0;
  }*/

  // Align Conveyor when Ground Pickup is active and there are no commands going to manual Conveyor control
  if (lower && !hook /*&& !m_conveyor_hook_is_ejecting*/) {
    conveyor_hook_is_aligned = !(hook_fraction < m_conveyor_hook_align_threshold);
    if (conveyor_hook_is_aligned) {
      conveyor_last_aligned_position = conveyor_position_abs;
      conveyor_power = 0;
      conveyor_current = 0;
    } else {
      conveyor_power = m_conveyor_hook_align_power;
      conveyor_current = 1000;
    }
  }

  // // Transition to ejection mode
  // static double ejecting_start_time = 0.0;
  // if (lower && L1 && conveyor_hook_is_aligned && !m_conveyor_hook_is_ejecting) {
  //   ejecting_start_time = current_time;
  //   m_conveyor_hook_is_ejecting = true;
  //   conveyor_hook_is_aligned = false;
  // }

  // // Max timeout on ejection
  // if (m_conveyor_hook_is_ejecting && current_time > ejecting_start_time + 1.5) {
  //   m_conveyor_hook_is_ejecting = false;
  // }

  // // During ejection, run until we reach throw position, then transition to throw
  // if (m_conveyor_hook_is_ejecting) {
  //   double throw_dist_rel = (1 + m_conveyor_hook_throw_fraction) * m_conveyor_ticks_per_hook;
  //   if ((conveyor_position_abs - conveyor_last_aligned_position) > throw_dist_rel && !m_conveyor_is_throwing) {
  //     m_conveyor_is_throwing = true;
  //     m_conveyor_hook_is_ejecting = false;
  //     m_conveyor_throw_start_time = current_time;
  //   }
  //   conveyor_power = 1.0;
  //   conveyor_current = 2500;
  // }

  // // Throw reverses for set duration and then zeros conveyor and returns to manual control
  // if (m_conveyor_is_throwing) {
  //   conveyor_power = -0.1;
  //   conveyor_current = 500;
  //   if (current_time > m_conveyor_throw_start_time + m_conveyor_hook_throw_duration) {
  //     m_conveyor_is_throwing = false;
  //     conveyor_power = 0.0;
  //     conveyor_current = 0;
  //   }
  // }

  rhi_ptr_->setMotorVoltageCommandPercent("ground_pickup_motor", ground_pickup_power);
  rhi_ptr_->setMotorCurrentLimitMilliAmps("ground_pickup_motor", ground_pickup_current);

  rhi_ptr_->setMotorVoltageCommandPercent("conveyor_motor", conveyor_power);
  rhi_ptr_->setMotorCurrentLimitMilliAmps("conveyor_motor", conveyor_current);
}

} // ghost_tank
