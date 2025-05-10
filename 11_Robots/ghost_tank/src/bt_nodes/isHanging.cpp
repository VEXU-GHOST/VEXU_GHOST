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

namespace ghost_tank {

    IsHanging::IsHanging(const std::string & name, const BT::NodeConfig & config) :
        BT::StatefulActionNode(name, config)
    {
        blackboard_ = config.blackboard;
        BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);

        hanging_ = false;
        test_duration_ = BT_Util::get_input<float>(this, "test_duration");
        threshold_ = BT_Util::get_input<float>(this, "threshold");

        ekf_sub_ = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
            "/map_ekf/odometry",
            rclcpp::SensorDataQoS(),
            std::bind(&IsHanging::ekfCallback, this, _1)
        );
    }

    BT::PortsList IsHanging::providedPorts() {
        return {
            BT::InputPort<float>("test_duration"),
            BT::InputPort<float>("threshold")
        };
    }

    void IsHanging::ekfCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (msg->twist.twist.angular.x > threshold_) hanging_ = true;
        if (msg->twist.twist.angular.y > threshold_) hanging_ = true;
    }

    BT::NodeStatus IsHanging::onStart() {
        start_time_ = node_ptr_->now();
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus IsHanging::onRunning() {

        if ((node_ptr_->now() - start_time_).seconds() < test_duration_) {
            return BT::NodeStatus::RUNNING;
        }

        return hanging_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

} // namespace ghost_tank
