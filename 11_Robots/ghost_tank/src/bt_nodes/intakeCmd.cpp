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

IntakeCmd::IntakeCmd(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config){
    // ros params
    // node_ptr_->declare_parameter("tank_robot_plugin.burnout_absolute_velocity_threshold_rpm", 50.0);
    // burnout_absolute_rpm_threshold_        = node_ptr_->get_parameter("tank_robot_plugin.burnout_absolute_velocity_threshold_rpm").as_double();

    // node_ptr_->declare_parameter("tank_robot_plugin.burnout_stall_duration_ms", 1000);
    // burnout_stall_duration_ms_    = node_ptr_->get_parameter("tank_robot_plugin.burnout_stall_duration_ms").as_int();

    // node_ptr_->declare_parameter("tank_robot_plugin.burnout_cooldown_duration_ms", 1000);
    // burnout_cooldown_duration_ms_ = node_ptr_->get_parameter("tank_robot_plugin.burnout_cooldown_duration_ms").as_int();
    blackboard_ = config.blackboard;
  	blackboard_->get("rhi_ptr", rhi_ptr_);
	  blackboard_->get("node_ptr", node_ptr_);
}

// It is mandatory to define this STATIC method.
BT::PortsList IntakeCmd::providedPorts()
{
    return {
        BT::InputPort<bool>("in"),
    };
}

BT::NodeStatus IntakeCmd::tick()
{
    auto status = BT::NodeStatus::FAILURE;
    bool in = BT_Util::get_input<bool>(this, "in");

    // Intake
    double intake_voltage;
    bool intake_command;
    if (!in) {
        intake_command = true;
        rhi_ptr_->setMotorCurrentLimitMilliAmps("intake_motor", 2500);
        intake_voltage = -1.0;
    } else if (in) {
        intake_command = true;
        rhi_ptr_->setMotorCurrentLimitMilliAmps("intake_motor", 2500);
        intake_voltage = 1.0;
    } else {
        intake_command = false;
        rhi_ptr_->setMotorCurrentLimitMilliAmps("intake_motor", 0);
        intake_voltage = 0.0;
    }

    bool intake_up = false;
    double intake_lift_target;
    if (!in) {    // intake lift
        intake_up = true;
        intake_lift_target = 0.0;
    } else {
        intake_up = false;
        intake_lift_target = 7.0;
    }

    if (std::fabs(rhi_ptr_->getMotorPosition("intake_lift_motor") - intake_lift_target) < 0.05) {
        rhi_ptr_->setMotorCurrentLimitMilliAmps("intake_lift_motor", 0);
        if (intake_up && !intake_command) {
          intake_voltage = 0.0;
        }
    } else {
        rhi_ptr_->setMotorCurrentLimitMilliAmps("intake_lift_motor", 2500);
        if (intake_up && !intake_command) {
            rhi_ptr_->setMotorCurrentLimitMilliAmps("intake_motor", 1000);
            intake_voltage = -1.0;
        }
    }

    rhi_ptr_->setMotorPositionCommand("intake_lift_motor", intake_lift_target);


    // If INTAKE_MOTOR stalling, update state and timer
    if ((intake_command) &&
        (std::fabs(rhi_ptr_->getMotorVelocityRPM("intake_motor")) < burnout_absolute_rpm_threshold_))
    {
        if (!intake_stalling_) {
            intake_stall_start_ = std::chrono::system_clock::now();

            intake_stalling_ = true;
        }
    } else {
        intake_stalling_ = false;
    }

    // If INTAKE_MOTOR stalled for too long, start cooldown period
    if (!intake_cooling_down_ && intake_stalling_ &&
        (std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now() -
        intake_stall_start_).count() > burnout_stall_duration_ms_) )
    {
        status = BT::NodeStatus::SUCCESS;
        intake_voltage = 0;
        rhi_ptr_->setMotorCurrentLimitMilliAmps("intake_motor", 0);
    }

    rhi_ptr_->setMotorVoltageCommandPercent("intake_motor", intake_voltage);
    // RCLCPP_INFO(node_ptr_->get_logger(), "intaking");

    return status;
}

} // namespace ghost_tank
