#include "ghost_tank/bt_nodes/moveScissor.hpp"
#include <cmath>

namespace ghost_tank
{

MoveScissor::MoveScissor(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "rhi_ptr", rhi_ptr_);
  first_loop_ = true;
}

BT::PortsList MoveScissor::providedPorts()
{
  return {
    BT::InputPort<double>("scissor_position_in"),
    BT::InputPort<double>("offset"),
    BT::InputPort<int>("timeout_ms")
  };
}

BT::NodeStatus MoveScissor::onStart()
{
  first_loop_ = true;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveScissor::onRunning()
{
  const double INCH_TO_DEG = 1. / (1. / 4. * 4.) * 360.;
  double target = BT_Util::get_input<double>(this, "scissor_position_in") * INCH_TO_DEG;
  double tolerance = BT_Util::get_input<double>(this, "offset") * INCH_TO_DEG;
  timeout_ms = BT_Util::get_input<int>(this, "timeout_ms");

  if (first_loop_) {
    start_time_ = std::chrono::system_clock::now();
    first_loop_ = false;
  }
  int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now() - start_time_).count();
  if (time_elapsed > std::abs(timeout_ms)) {
    return BT::NodeStatus::FAILURE;
  }

  double current_position = rhi_ptr_->getMotorPosition("scissor_motor");
  double error = target - current_position;

  if (std::fabs(error) > tolerance) {
    // Bang bang control: full voltage in the direction of the error
    rhi_ptr_->setMotorCurrentLimitMilliAmps("scissor_motor", 2500);
    if (error > 0) {
      rhi_ptr_->setMotorVoltageCommandPercent("scissor_motor", 1.0);
    } else {
      rhi_ptr_->setMotorVoltageCommandPercent("scissor_motor", -1.0);
    }
    return BT::NodeStatus::RUNNING;
  } else {
    // Within tolerance, stop the motor.
    rhi_ptr_->setMotorCurrentLimitMilliAmps("scissor_motor", 0);
    rhi_ptr_->setMotorVoltageCommandPercent("scissor_motor", 0.0);
    return BT::NodeStatus::SUCCESS;
  }
}

void MoveScissor::onHalted()
{
  resetStatus();
}

} // namespace ghost_tank
