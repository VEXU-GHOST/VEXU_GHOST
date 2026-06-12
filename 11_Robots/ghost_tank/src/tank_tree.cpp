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

TankTree::TankTree(std::string bt_path)
: bt_path_(bt_path)
{
  global_blackboard_ = BT::Blackboard::create();
}

void TankTree::init_tree()
{
  BT::BehaviorTreeFactory factory;

  // add all nodes here
  factory.registerNodeType<LoggingNode>("Logging");
  factory.registerNodeType<AutoDone>("AutoDone");
  factory.registerNodeType<AutonTimer>("AutonTimer");
  factory.registerNodeType<LoadPathFromCSV>("LoadPathFromCSV");
  factory.registerNodeType<BiteCmd>("BiteCmd");
  factory.registerNodeType<ClampCmd>("ClampCmd");
  factory.registerNodeType<ClimbCmd>("ClimbCmd");
  factory.registerNodeType<DescorerCmd>("DescorerCmd");
  factory.registerNodeType<AdjustSwitcherCmd>("AdjustSwitcherCmd");
  factory.registerNodeType<SorterCmd>("SorterCmd");
  factory.registerNodeType<AutoSortCmd>("AutoSortCmd");
  factory.registerNodeType<SwitcherCmd>("SwitcherCmd");
  factory.registerNodeType<LeftBlockerCmd>("LeftBlockerCmd");
  factory.registerNodeType<RightBlockerCmd>("RightBlockerCmd");
  factory.registerNodeType<LittleWillCmd>("LittleWillCmd");
  factory.registerNodeType<MatchLoadingCmd>("MatchLoadingCmd");
  factory.registerNodeType<ScoreBallCmd>("ScoreBallCmd");
  factory.registerNodeType<ScorePosCmd>("ScorePosCmd");
  factory.registerNodeType<OuttakeBallsCmd>("OuttakeBallsCmd");
  factory.registerNodeType<ShutoffNode>("ShutoffNode");
  factory.registerNodeType<IntakeCmd>("IntakeCmd");
  factory.registerNodeType<IntakeCmd>("IsHanging");
  factory.registerNodeType<GoalRushCmd>("GoalRushCmd");
  factory.registerNodeType<SetColorTarget>("SetColorTarget");
  factory.registerNodeType<SetMirrored>("SetMirrored");
  factory.registerNodeType<ConveyorCmd>("ConveyorCmd");
  factory.registerNodeType<WaitCmd>("WaitCmd");
  factory.registerNodeType<NeutralStakeCmd>("NeutralStakeCmd");
  factory.registerNodeType<GoalRushDetected>("GoalRushDetected");
  factory.registerNodeType<GoalDetected>("GoalDetected");
  factory.registerNodeType<DistanceThreshold>("DistanceThreshold");
  factory.registerNodeType<BoundaryCheck>("BoundaryCheck");
  factory.registerNodeType<MoveVoltage>("MoveVoltage");
  factory.registerNodeType<TurnToPoint>("TurnToPoint");
  factory.registerNodeType<GenerateBezierPath>("GenerateBezierPath");
  factory.registerNodeType<GeneratePlannerPath>("GeneratePlannerPath");
  factory.registerNodeType<GoalPoseListener>("GoalPoseListener");
  factory.registerNodeType<FollowPathPurePursuit>("FollowPathPurePursuit");
  factory.registerNodeType<FollowPathControllerServer>("FollowPathControllerServer");
  factory.registerNodeType<ArcturnToPoint>("ArcturnToPoint");
  factory.registerNodeType<MoveToPoint>("MoveToPoint");
  factory.registerNodeType<MoveScissor>("MoveScissor");

  tree_ = factory.createTreeFromFile(bt_path_, global_blackboard_);
  std::cout << "Tree created: " << bt_path_ << std::endl;
}

void TankTree::set_path(std::string path)
{
  bt_path_ = path;
}

void TankTree::tick_tree()
{
  tree_.tickExactlyOnce();
}

} // namespace ghost_tank
