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

#include <string>
#include <vector>
#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "ghost_tank/tank_model.hpp"
#include "ghost_tank/bt_nodes/bt_util.hpp"
#include "push_back_cv/msg/field_block_array.hpp"

namespace ghost_tank
{

// Pure selection logic, factored out of the BT node so it can be unit-tested
// without ROS / TankModel / a blackboard. Returns the index into `blocks` of the
// nearest block (Euclidean, map frame) that passes the colour/range/age filters,
// or -1 if none qualify.
//   match_alliance   : when true, keep only blocks whose is_red == is_red_alliance
//   max_range_m      : <= 0 disables the range filter
//   max_age_s        : <= 0 disables the age filter (also skipped when a block's
//                      last_seen carries no real stamp, i.e. sec == 0)
//   now_s            : current time in seconds, only used for the age filter
int selectClosestBlockIndex(
  const std::vector<push_back_cv::msg::FieldBlock> & blocks,
  double robot_x, double robot_y,
  bool match_alliance, bool is_red_alliance,
  double max_range_m, double max_age_s, double now_s);

// Condition/action node that finds the single closest CV-detected block to the
// robot and writes its position onto the blackboard via output ports. Blocks come
// from the CV pipeline on /field/blocks (push_back_cv/FieldBlockArray), already in
// the map frame. By default only blocks matching our alliance colour are
// considered (the "is_red_alliance" blackboard flag); set match_alliance="false"
// to consider blocks of any colour.
//
// Succeeds (and sets the output ports) when a matching block is found; fails when
// there is no CV data yet or no block passes the colour/range/age filters. Output
// positions are in tiles so they wire straight into the move nodes
// (e.g. ActuallyMoveToPoint posX_tiles="{block_x_tiles}").
class GetClosestBlock : public BT::SyncActionNode
{
public:
  GetClosestBlock(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

  void blocksUpdate(const push_back_cv::msg::FieldBlockArray::SharedPtr msg);

private:
  std::shared_ptr<rclcpp::Node> node_ptr_;
  std::shared_ptr<TankModel> tank_model_ptr_;
  BT::Blackboard::Ptr blackboard_;

  rclcpp::Subscription<push_back_cv::msg::FieldBlockArray>::SharedPtr blocks_sub_;

  // Latest blocks message, cached by the subscription callback.
  push_back_cv::msg::FieldBlockArray latest_blocks_;
  bool have_blocks_ = false;
};

} // namespace ghost_tank
