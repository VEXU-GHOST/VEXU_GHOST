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

#include <ghost_util/angle_util.hpp>
#include "ghost_tank/tank_tree.hpp"


// file that contains the custom nodes definitions
// #include "dummy_nodes.h"
// using namespace DummyNodes;

namespace ghost_tank
{

tankTree::tankTree(std::string bt_path,
					   std::string bt_path_interaction,
                       std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> robot_hardware_interface_ptr,
                       std::shared_ptr<tankModel> tank_ptr,
                       std::shared_ptr<rclcpp::Node> node_ptr) :
	bt_path_(bt_path),
	bt_path_interaction_(bt_path_interaction),
	node_ptr_(node_ptr){
	BT::BehaviorTreeFactory factory;

	// add all nodes here
	factory.registerNodeType<LoggingNode>("Logging", node_ptr_);
	factory.registerNodeType<CheckForRestart>("CheckForRestart", node_ptr_, robot_hardware_interface_ptr);
	factory.registerNodeType<MoveToPoseCubic>("MoveToPoseCubic", node_ptr_, robot_hardware_interface_ptr, tank_ptr);
	factory.registerNodeType<SwipeTail>("SwipeTail", node_ptr_, robot_hardware_interface_ptr, tank_ptr);
	factory.registerNodeType<IntakeCmd>("IntakeCmd", node_ptr_, robot_hardware_interface_ptr, tank_ptr);
	factory.registerNodeType<Climb>("Climb", node_ptr_, robot_hardware_interface_ptr, tank_ptr);
	factory.registerNodeType<AutoDone>("AutoDone", node_ptr_, robot_hardware_interface_ptr, tank_ptr);
	factory.registerNodeType<AutonTimer>("AutonTimer", node_ptr_, tank_ptr);

    tree_ = factory.createTreeFromFile(bt_path_);
	tree_interaction_ = factory.createTreeFromFile(bt_path_interaction_);
}

void tankTree::tick_tree()
{
  tree_.tickExactlyOnce();
}

void tankTree::tick_tree_interaction()
{
  tree_interaction_.tickExactlyOnce();
}

} // namespace ghost_tank
