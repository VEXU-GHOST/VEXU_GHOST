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

#include "ghost_swerve/bt_nodes/climb.hpp"

namespace ghost_swerve
{

Climb::Climb(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<rclcpp::Node> node_ptr,
  std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr,
  std::shared_ptr<SwerveModel> swerve_ptr)
: BT::StatefulActionNode(name, config),
  node_ptr_(node_ptr),
  rhi_ptr_(rhi_ptr),
  swerve_ptr_(swerve_ptr)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList Climb::providedPorts()
{
  return {
  };
}

template<typename T>
T Climb::get_input(std::string key)
{
  BT::Expected<T> input = getInput<T>(key);
  // Check if expected is valid. If not, throw its error
  if (!input) {
    throw BT::RuntimeError(
            "missing required input [" + key + "]: ",
            input.error());
  }
  return input.value();
}

/// Method called once, when transitioning from the state IDLE.
/// If it returns RUNNING, this becomes an asynchronous node.
BT::NodeStatus Climb::onStart()
{
  climbing_ = false;
  reaching_ = true;
  return BT::NodeStatus::RUNNING;
}

/// when the method halt() is called and the action is RUNNING, this method is invoked.
/// This is a convenient place todo a cleanup, if needed.
void Climb::onHalted()
{
  rhi_ptr_->setMotorCurrentLimitMilliAmps("lift_r1", 0);
  rhi_ptr_->setMotorCurrentLimitMilliAmps("lift_r2", 0);
  rhi_ptr_->setMotorCurrentLimitMilliAmps("lift_l1", 0);
  rhi_ptr_->setMotorCurrentLimitMilliAmps("lift_l2", 0);

  resetStatus();
}

BT::NodeStatus Climb::onRunning()
{
  auto m_digital_io = std::vector<bool>(8, false);
  auto m_digital_io_name_map = std::unordered_map<std::string, size_t>{
    {"tail", 0},
    {"claw", 1}
  };

  bool claw_open;
  auto status = BT::NodeStatus::RUNNING;

  double lift_target_up = swerve_ptr_->getConfig().lift_up_angle;
  double lift_target_down = swerve_ptr_->getConfig().lift_climbed_angle;

  double posR = rhi_ptr_->getMotorPosition("lift_r1");
  double posL = rhi_ptr_->getMotorPosition("lift_l1");
  RCLCPP_INFO(node_ptr_->get_logger(), "up: %f", lift_target_up);
  

  if (posL > lift_target_up)
  {
    reaching_ = false;
    climbing_ = true;
  } else if (posL < lift_target_down)
  {
    climbing_ = false;
    reaching_ = false;
  }

  if (reaching_) {
    lift_target_ = lift_target_up;
    claw_open = true;
  } else if (climbing_) {
    lift_target_ = lift_target_down;
    claw_open = false;
  } else {
    // lift_target_ = lift_target_down;
    claw_open = false;
  }

  RCLCPP_INFO(node_ptr_->get_logger(), "target: %f", lift_target_);

  m_digital_io[m_digital_io_name_map.at("claw")] = !claw_open;

  rhi_ptr_->setDigitalIO(m_digital_io);

  if (reaching_) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Reaching");

    rhi_ptr_->setMotorCurrentLimitMilliAmps("lift_l1", 2500);
    rhi_ptr_->setMotorVoltageCommandPercent("lift_l1", -1.0);

    rhi_ptr_->setMotorCurrentLimitMilliAmps("lift_l2", 2500);
    rhi_ptr_->setMotorVoltageCommandPercent("lift_l2", -1.0);

    rhi_ptr_->setMotorCurrentLimitMilliAmps("lift_r1", 2500);
    rhi_ptr_->setMotorVoltageCommandPercent("lift_r1", -1.0);

    rhi_ptr_->setMotorCurrentLimitMilliAmps("lift_r2", 2500);
    rhi_ptr_->setMotorVoltageCommandPercent("lift_r2", -1.0);
  } else if (climbing_) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Climbing");

    rhi_ptr_->setMotorCurrentLimitMilliAmps("lift_l1", 2500);
    rhi_ptr_->setMotorVoltageCommandPercent("lift_l1", 1.0);

    rhi_ptr_->setMotorCurrentLimitMilliAmps("lift_l2", 2500);
    rhi_ptr_->setMotorVoltageCommandPercent("lift_l2", 1.0);

    rhi_ptr_->setMotorCurrentLimitMilliAmps("lift_r1", 2500);
    rhi_ptr_->setMotorVoltageCommandPercent("lift_r1", 1.0);

    rhi_ptr_->setMotorCurrentLimitMilliAmps("lift_r2", 2500);
    rhi_ptr_->setMotorVoltageCommandPercent("lift_r2", 1.0);
  } else {
    RCLCPP_INFO(node_ptr_->get_logger(), "Done climbing");

    rhi_ptr_->setMotorCurrentLimitMilliAmps("lift_l1", 0);
    rhi_ptr_->setMotorVoltageCommandPercent("lift_l1", 0);

    rhi_ptr_->setMotorCurrentLimitMilliAmps("lift_l2", 0);
    rhi_ptr_->setMotorVoltageCommandPercent("lift_l2", 0);

    rhi_ptr_->setMotorCurrentLimitMilliAmps("lift_r1", 0);
    rhi_ptr_->setMotorVoltageCommandPercent("lift_r1", 0);

    rhi_ptr_->setMotorCurrentLimitMilliAmps("lift_r2", 0);
    rhi_ptr_->setMotorVoltageCommandPercent("lift_r2", 0);
  }

  return status;
}

} // namespace ghost_swerve
