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

TankTree::TankTree(std::string bt_path, std::string bt_path_interaction) :
	bt_path_(bt_path),
	bt_path_interaction_(bt_path_interaction){

	BT::BehaviorTreeFactory factory;
	global_blackboard = BT::Blackboard::create();

	// add all nodes here
	factory.registerNodeType<LoggingNode>("Logging");
	factory.registerNodeType<MoveToPoseCubic>("MoveToPoseCubic");
	factory.registerNodeType<IntakeCmd>("IntakeCmd");
	factory.registerNodeType<AutoDone>("AutoDone");
	factory.registerNodeType<AutonTimer>("AutonTimer");

    tree_ = factory.createTreeFromFile(bt_path_, global_blackboard);

	tree_interaction_ = factory.createTreeFromFile(bt_path_interaction_, global_blackboard);
}

void TankTree::tick_tree()
{
  tree_.tickExactlyOnce();
}

void TankTree::tick_tree_interaction()
{
  tree_interaction_.tickExactlyOnce();
}

} // namespace ghost_tank
