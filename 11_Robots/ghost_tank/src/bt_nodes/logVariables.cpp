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

#include "ghost_tank/bt_nodes/logVariables.hpp"

#include <sstream>

// SyncActionNode that prints the value of a configurable list of numeric
// blackboard variables. If your Node has ports, you must use this constructor
// signature.
LogVariables::LogVariables(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  blackboard_ = config.blackboard;
  if (!blackboard_->get("node_ptr", node_ptr_)) {
    std::cout << name << ": node_ptr not found in blackboard" << std::endl;
  }
}

// It is mandatory to define this STATIC method.
BT::PortsList LogVariables::providedPorts()
{
  return {
    // Names of the blackboard variables to print. Separate multiple names with
    // a comma, semicolon, or whitespace, e.g. "block_x_tiles, block_y_tiles".
    BT::InputPort<std::string>("variables"),
    // Optional prefix printed before the variables to label the log line.
    BT::InputPort<std::string>("message"),
    // Optional sample rate. <= 0 (default) logs every tick; otherwise the log
    // line is emitted at most rate_hz times per second (e.g. 2.0 to match a
    // 2 Hz CV feed) so the tracked data is a clean time series, not a flood.
    BT::InputPort<double>("rate_hz")
  };
}

// Override the virtual function tick()
BT::NodeStatus LogVariables::tick()
{
  // Rate limiting: skip this tick entirely if we logged less than 1/rate_hz ago.
  // Returns SUCCESS so the surrounding Sequence keeps flowing between samples.
  const double rate_hz = BT_Util::get_input<double>(this, "rate_hz", 0.0);
  if (rate_hz > 0.0 && node_ptr_) {
    const rclcpp::Time now = node_ptr_->now();
    if (have_logged_ && (now - last_log_time_).seconds() < 1.0 / rate_hz) {
      return BT::NodeStatus::SUCCESS;
    }
    last_log_time_ = now;
    have_logged_ = true;
  }

  const std::string variables = BT_Util::get_input<std::string>(this, "variables");
  const std::string prefix = BT_Util::get_input<std::string>(this, "message", std::string(""));

  std::ostringstream out;
  if (!prefix.empty()) {
    out << prefix << ": ";
  }

  // Split the list on commas, semicolons, and whitespace so the same node works
  // regardless of how the names are separated in the XML.
  std::string name;
  std::istringstream stream(variables);
  bool first = true;
  while (stream >> name) {
    // Drop any trailing comma/semicolon that came along with the token.
    while (!name.empty() && (name.back() == ',' || name.back() == ';')) {
      name.pop_back();
    }
    if (name.empty()) {
      continue;
    }

    if (!first) {
      out << ", ";
    }
    first = false;

    // getAnyLocked returns a falsy handle when the key was never declared.
    auto any_locked = blackboard_->getAnyLocked(name);
    if (!any_locked) {
      out << name << "=<missing>";
      continue;
    }

    const BT::Any * any = any_locked.get();
    if (!any || any->empty()) {
      out << name << "=<uninit>";
      continue;
    }

    // tryCast safely converts ints/floats/numeric-strings to double without
    // throwing; non-numeric entries report as such instead of crashing.
    auto value = any->tryCast<double>();
    if (value) {
      out << name << "=" << value.value();
    } else {
      out << name << "=<non-number>";
    }
  }

  RCLCPP_INFO(node_ptr_->get_logger(), "%s", out.str().c_str());
  return BT::NodeStatus::SUCCESS;
}
