/*
 *   Copyright (c) 2024 Jake Wendling, Xander Wilson
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

#include "ghost_tank/bt_nodes/isHanging.hpp"

namespace ghost_tank
{

    IsHanging::IsHanging(const std::string &name, const BT::NodeConfig &config) : BT::StatefulActionNode(name, config)
    {
        blackboard_ = config.blackboard;
        BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
    }

    BT::PortsList IsHanging::providedPorts()
    {
        return {
            BT::InputPort<double>("vel_threshold_tps"),
            BT::InputPort<double>("test_thrust_pct"),
            BT::InputPort<int>("test_duration_ms")};
    }

    BT::NodeStatus IsHanging::onStart()
    {
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus IsHanging::onRunning()
    {
        if (first_loop_)
        {
            start_time_ = std::chrono::system_clock::now();
            first_loop_ = false;
        }

        vel_threshold_mps_ = BT_Util::get_input<double>(this, "vel_threshold_tps") * ghost_util::TILES_TO_METERS;
        test_thrust_pct_ = BT_Util::get_input<double>(this, "test_thrust_pct") * ghost_util::TILES_TO_METERS;
        test_duration_ms_ = BT_Util::get_input<int>(this, "test_duration_ms") * ghost_util::TILES_TO_METERS;

        double wheel_speed = tank_model_ptr_->getCurrentHighestWheelLinearVelocity();

        int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time_).count();
        if (time_elapsed < test_duration_ms_)
        {
            tank_model_ptr_->driveCommandArcade(test_thrust_pct_, 0.0);
            return BT::NodeStatus::RUNNING;
        }
        else
        {
            bool hanging = (wheel_speed > vel_threshold_mps_);

            std::cout << "[IsHanging::onRunning] Current Wheel Linear Speed (m/s): " << wheel_speed << std::endl;
            std::cout << "[IsHanging::onRunning] Velocity Threshold (m/s): " << vel_threshold_mps_ << std::endl;

            tank_model_ptr_->driveCommandArcade(0.0, 0.0);
            return hanging ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }
    }

    void IsHanging::onHalted()
    {
        resetStatus();
    }

} // namespace ghost_tank
