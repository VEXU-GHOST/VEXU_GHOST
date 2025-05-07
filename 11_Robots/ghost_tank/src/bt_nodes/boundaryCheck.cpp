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

#include "ghost_tank/bt_nodes/boundaryCheck.hpp"

namespace ghost_tank
{

// SyncActionNode (synchronous action) with an input port.
// If your Node has ports, you must use this constructor signature
BoundaryCheck::BoundaryCheck(
  const std::string & name, const BT::NodeConfig & config)
: BT::DecoratorNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
}

// It is mandatory to define this STATIC method.
BT::PortsList BoundaryCheck::providedPorts()
{
  // This action has a single input port called "message"
  return {
    BT::InputPort<double>("x_lower_bound_tiles"),
    BT::InputPort<double>("y_lower_bound_tiles"),
    BT::InputPort<double>("x_upper_bound_tiles"),
    BT::InputPort<double>("y_upper_bound_tiles")
  };
}

void BoundaryCheck::halt()
{
  haltChild();
}

// Override the virtual function tick()
BT::NodeStatus BoundaryCheck::tick()
{
  static constexpr double tile_to_meters = 0.6096;
  double x_lower_bound_m = BT_Util::get_input<double>(this, "x_lower_bound_tiles") * tile_to_meters;
  double y_lower_bound_m = BT_Util::get_input<double>(this, "y_lower_bound_tiles") * tile_to_meters;
  double x_upper_bound_m = BT_Util::get_input<double>(this, "x_upper_bound_tiles") * tile_to_meters;
  double y_upper_bound_m = BT_Util::get_input<double>(this, "y_upper_bound_tiles") * tile_to_meters;

  double current_x_m = tank_model_ptr_->getWorldPose().x();
  double current_y_m = tank_model_ptr_->getWorldPose().y();

  bool x_bounds_violated = (current_x_m < x_lower_bound_m) || (current_x_m > x_upper_bound_m);
  bool y_bounds_violated = (current_y_m < y_lower_bound_m) || (current_y_m > y_upper_bound_m);

  if (x_bounds_violated || y_bounds_violated) {
    haltChild();
    return BT::NodeStatus::FAILURE;
  }

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

} // ghost_tank
