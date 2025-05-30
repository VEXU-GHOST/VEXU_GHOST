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

#include "ghost_tank/bt_nodes/load_path_from_csv.hpp"

namespace ghost_tank
{

// If your Node has ports, you must use this constructor signature
LoadPathFromCSV::LoadPathFromCSV(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  BT_Util::get_from_blackboard(blackboard_, "config_path", config_path);
  BT_Util::get_from_blackboard(blackboard_, "tank_trajectory_ptr", tank_trajectory_ptr_);

}

// It is mandatory to define this STATIC method.
BT::PortsList LoadPathFromCSV::providedPorts()
{
  return {
    BT::InputPort<std::string>("file_path"),
  };
}

BT::NodeStatus LoadPathFromCSV::onStart()
{
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LoadPathFromCSV::onRunning()
{
  // Load path from file
  auto file_path = BT_Util::get_input<std::string>(this, "file_path");
  auto path = ghost_util::readPathFromFile(config_path + '/' + file_path);

  // Clear trajectory
  int num_points = path[0].size();
  tank_trajectory_ptr_->clear();
  tank_trajectory_ptr_->resize(num_points);

  tank_trajectory_ptr_->x = path[0];
  tank_trajectory_ptr_->y = path[1];
  tank_trajectory_ptr_->theta = path[2];

  for (int i = 0; i < num_points; i++) {
    if (BT_Util::get_from_blackboard<bool>(blackboard_, "mirrored")) {
      tank_trajectory_ptr_->x[i] = 6.0 * ghost_util::TILES_TO_METERS - tank_trajectory_ptr_->x[i];
      tank_trajectory_ptr_->theta[i] = ghost_util::WrapAngle2PI(M_PI - tank_trajectory_ptr_->theta[i]);
    }
    tank_trajectory_ptr_->t[i] = static_cast<double>(i) / static_cast<double>(num_points);
  }

  tank_trajectory_ptr_->calculateRemainingPathLengths();

  return BT::NodeStatus::SUCCESS;
}

void LoadPathFromCSV::onHalted()
{
  resetStatus();
}

} // namespace ghost_tank
