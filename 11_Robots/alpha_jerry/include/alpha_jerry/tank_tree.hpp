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

#include "alpha_jerry/bt_nodes/loggingNode.hpp"
#include "alpha_jerry/bt_nodes/autoDone.hpp"
#include "alpha_jerry/bt_nodes/autonTimer.hpp"
#include "alpha_jerry/bt_nodes/moveToPoseBezier.hpp"
#include "alpha_jerry/bt_nodes/moveToPoseBoomerang.hpp"
#include "alpha_jerry/bt_nodes/moveToPosePurepursuit.hpp"
#include "alpha_jerry/bt_nodes/biteCmd.hpp"
#include "alpha_jerry/bt_nodes/clampCmd.hpp"
#include "alpha_jerry/bt_nodes/shutoffNode.hpp"
#include "alpha_jerry/bt_nodes/intakeCmd.hpp"
#include "alpha_jerry/bt_nodes/goalRushCmd.hpp"
#include "alpha_jerry/bt_nodes/setMirrored.hpp"
#include "alpha_jerry/bt_nodes/setColorTarget.hpp"
#include "alpha_jerry/bt_nodes/conveyorCmd.hpp"
#include "alpha_jerry/bt_nodes/waitCmd.hpp"
#include "alpha_jerry/bt_nodes/neutralStakeCmd.hpp"
#include "alpha_jerry/bt_nodes/goalRushDetected.hpp"
#include "alpha_jerry/bt_nodes/goalDetected.hpp"

#include "alpha_jerry/bt_nodes/bt_util.hpp"

#include "alpha_jerry/tank_model.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace alpha_jerry
{

class TankTree
{
public:
	TankTree(std::string bt_path);
	void tick_tree();
	void init_tree();
	template<typename T>
	void set_variable(std::string name, T value){
		if(global_blackboard_){
			global_blackboard_->set<T>(name, value);
			// std::cout << "Set bt variable:" << name << std::endl;
		} else {
			std::cout << "ERROR: Tried to set BT variable before the BT constructor" << std::endl;
		}
	}
	template<typename T>
	bool get_variable(std::string name, T & value){
		if(!global_blackboard_){
			std::cout << "ERROR: Tried to get BT variable before the BT constructor" << std::endl;
			return false;
		} else {
			return global_blackboard_->get<T>(name, value);
		}
	}
private:
	std::string bt_path_;
	BT::Blackboard::Ptr global_blackboard_;
	BT::Tree tree_;
};

} // namespace alpha_jerry
