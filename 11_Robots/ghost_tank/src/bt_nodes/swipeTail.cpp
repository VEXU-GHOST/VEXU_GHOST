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

#include "ghost_tank/bt_nodes/swipeTail.hpp"

namespace ghost_tank
{

SwipeTail::SwipeTail(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<rclcpp::Node> node_ptr,
  std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr,
  std::shared_ptr<tankModel> tank_ptr)
: BT::StatefulActionNode(name, config),
  node_ptr_(node_ptr),
  rhi_ptr_(rhi_ptr),
  tank_ptr_(tank_ptr)
{
  started_ = false;
}

// It is mandatory to define this STATIC method.
BT::PortsList SwipeTail::providedPorts()
{
  return {
    BT::InputPort<int>("num_swipes"),
  };
}

template<typename T>
T SwipeTail::get_input(std::string key)
{
  BT::Expected<T> input = getInput<T>(key);
  // Check if expected is valid. If not, throw its error
  if (!input) {
    throw BT::RuntimeError(
            "missing required input [" + key + "]: ",
            input.error() );
  }
  return input.value();
}

/// Method called once, when transitioning from the state IDLE.
/// If it returns RUNNING, this becomes an asynchronous node.
BT::NodeStatus SwipeTail::onStart()
{
  started_ = false;
  return BT::NodeStatus::RUNNING;
}

/// when the method halt() is called and the action is RUNNING, this method is invoked.
/// This is a convenient place todo a cleanup, if needed.
void SwipeTail::onHalted()
{
  resetStatus();
}

BT::NodeStatus SwipeTail::onRunning()
{
  auto m_digital_io = std::vector<bool>(8, false);
  auto m_digital_io_name_map = std::unordered_map<std::string, size_t>{
    {"tail", 0},
    {"claw", 1}
  };
  auto status = BT::NodeStatus::RUNNING;
  int num_swipes = get_input<int>("num_swipes");

  if (num_swipes <= 0) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "NumSwipes: invalid input");
    return BT::NodeStatus::FAILURE;
  }

  int time_elapsed = 0;
  if (started_) {
    auto now = std::chrono::system_clock::now();
    time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_).count();
  } else {
    start_time_ = std::chrono::system_clock::now();
    started_ = true;
    rhi_ptr_->setMotorCurrentLimitMilliAmps("tail_motor", 2500);
    rhi_ptr_->setMotorPositionCommand("tail_motor", tank_ptr_->getConfig().stick_angle_normal);
  }

  double tail_mtr_pos = rhi_ptr_->getMotorPosition("tail_motor");
  double stick_turn_offset = tank_ptr_->getConfig().stick_turn_offset;
  #define MTR_CLOSE_TO(x) (fabs(tail_mtr_pos - x) < stick_turn_offset)

  RCLCPP_INFO(node_ptr_->get_logger(), "time elapsed: %f", time_elapsed * 0.001);
  // std::cout << "time elapsed:" << time_elapsed * 0.001;

  if (500 < time_elapsed && time_elapsed < 1000 * num_swipes + 500) {
    if (time_elapsed % 1000 <= 400) {
      rhi_ptr_->setMotorPositionCommand("tail_motor", tank_ptr_->getConfig().stick_angle_normal);
    } else {
      rhi_ptr_->setMotorPositionCommand("tail_motor", -tank_ptr_->getConfig().stick_angle_normal);
    }
    status = BT::NodeStatus::RUNNING;
  } else if (time_elapsed > 1000 * num_swipes + 500) {
    rhi_ptr_->setMotorPositionCommand("tail_motor", tank_ptr_->getConfig().stick_upright_angle);
    if (MTR_CLOSE_TO(tank_ptr_->getConfig().stick_upright_angle)) { // within n degrees of upright
      rhi_ptr_->setMotorCurrentLimitMilliAmps("tail_motor", 100);        // i'm going to give it less but not none so it can hold itself centered
    }
    status = BT::NodeStatus::SUCCESS;
  }

  if (status == BT::NodeStatus::SUCCESS) {
    m_digital_io[m_digital_io_name_map.at("tail")] = false;
  } else {
    m_digital_io[m_digital_io_name_map.at("tail")] = true;
  }

  rhi_ptr_->setDigitalIO(m_digital_io);
  return status;
}

} // namespace ghost_tank
