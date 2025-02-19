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

#include "ghost_tank/bt_nodes/shutoffNode.hpp"
#include "ghost_tank/pdcontrol.hpp"

using std::placeholders::_1;

namespace ghost_tank
{

// If your Node has ports, you must use this constructor signature
ShutoffNode::ShutoffNode(const std::string& name, const BT::NodeConfig& config):
	BT::StatefulActionNode(name, config){
  	std::cout << "[ShutoffNode::ShutoffNode]" << std::endl;
		
	blackboard_ = config.blackboard;
	BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);
	BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
}

// It is mandatory to define this STATIC method.
BT::PortsList ShutoffNode::providedPorts(){
	return {
	};
}

/// Method called once, when transitioning from the state IDLE.
/// If it returns RUNNING, this becomes an asynchronous node.
BT::NodeStatus ShutoffNode::onStart(){
	return BT::NodeStatus::RUNNING;
}

/// when the method halt() is called and the action is RUNNING, this method is invoked.
/// This is a convenient place todo a cleanup, if needed.
void ShutoffNode::onHalted(){
	resetStatus();
}

BT::NodeStatus ShutoffNode::onRunning() {
	tank_model_ptr_->driveCommand(0.0, 0.0);
	
  	return BT::NodeStatus::RUNNING;
}

} // namespace ghost_tank
