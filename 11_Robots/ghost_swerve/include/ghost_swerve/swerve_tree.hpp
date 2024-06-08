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

#pragma once
#include "behaviortree_cpp/bt_factory.h"
#include "bt_nodes/checkForRestart.hpp"
#include "bt_nodes/intakeCmd.hpp"
#include "bt_nodes/loggingNode.hpp"
#include "bt_nodes/moveToPoseCubic.hpp"
#include "bt_nodes/swipeTail.hpp"
#include "bt_nodes/autoDone.hpp"
#include "bt_nodes/autonTimer.hpp"
#include "bt_nodes/climb.hpp"
#include "ghost_swerve/swerve_model.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

// file that contains the custom nodes definitions
// #include "dummy_nodes.h"
// using namespace DummyNodes;

namespace ghost_swerve
{

class SwerveTree
{
public:
	SwerveTree(std::string bt_path,
			   std::string bt_path_interaction,
	           std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> robot_hardware_interface_ptr,
	           std::shared_ptr<SwerveModel> swerve_ptr,
	           std::shared_ptr<rclcpp::Node> node_ptr);
	void tick_tree();
	void tick_tree_interaction();

private:
  std::string bt_path_;
  std::string bt_path_interaction_;
  BT::Tree tree_;
  BT::Tree tree_interaction_;
  std::shared_ptr<rclcpp::Node> node_ptr_;
};

} // namespace ghost_swerve
