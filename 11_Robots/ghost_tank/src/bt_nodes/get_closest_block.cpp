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

#include "ghost_tank/bt_nodes/get_closest_block.hpp"

#include <cmath>
#include <limits>

#include "ghost_util/unit_conversion_utils.hpp"

namespace ghost_tank
{

using std::placeholders::_1;

int selectClosestBlockIndex(
  const std::vector<push_back_cv::msg::FieldBlock> & blocks,
  double robot_x, double robot_y,
  bool match_alliance, bool is_red_alliance,
  double max_range_m, double max_age_s, double now_s)
{
  double best_dist_m = std::numeric_limits<double>::max();
  int best_idx = -1;

  for (size_t i = 0; i < blocks.size(); ++i) {
    const auto & block = blocks[i];

    // Colour filter.
    if (match_alliance && (block.is_red != is_red_alliance)) {
      continue;
    }

    // Age filter (only when last_seen carries a real stamp and a limit is set).
    if (max_age_s > 0.0 && block.last_seen.sec != 0) {
      const double last_seen_s =
        static_cast<double>(block.last_seen.sec) + block.last_seen.nanosec * 1e-9;
      if (now_s - last_seen_s > max_age_s) {
        continue;
      }
    }

    const double dist_m = std::hypot(block.x - robot_x, block.y - robot_y);

    // Range filter.
    if (max_range_m > 0.0 && dist_m > max_range_m) {
      continue;
    }

    if (dist_m < best_dist_m) {
      best_dist_m = dist_m;
      best_idx = static_cast<int>(i);
    }
  }

  return best_idx;
}

GetClosestBlock::GetClosestBlock(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);

  if (node_ptr_) {
    // /field/blocks is published by block_map_node with the default QoS (reliable,
    // depth 10), so match that here or no samples are delivered.
    blocks_sub_ = node_ptr_->create_subscription<push_back_cv::msg::FieldBlockArray>(
      "/field/blocks", 10,
      std::bind(&GetClosestBlock::blocksUpdate, this, _1));
  }
}

void GetClosestBlock::blocksUpdate(const push_back_cv::msg::FieldBlockArray::SharedPtr msg)
{
  latest_blocks_ = *msg;
  have_blocks_ = true;
}

BT::PortsList GetClosestBlock::providedPorts()
{
  return {
    // Inputs (all optional)
    BT::InputPort<bool>("match_alliance"),    // true (default): only our alliance colour
    BT::InputPort<double>("max_range_tiles"), // ignore blocks farther than this (default: no limit)
    BT::InputPort<double>("max_age_s"),       // ignore blocks last_seen older than this (default: no limit)

    // Outputs (written only when a block is found)
    BT::OutputPort<double>("block_x_tiles"),
    BT::OutputPort<double>("block_y_tiles"),
    BT::OutputPort<int>("block_id"),
    BT::OutputPort<double>("block_dist_tiles"),
  };
}

BT::NodeStatus GetClosestBlock::tick()
{
  if (!node_ptr_ || !tank_model_ptr_) {
    return BT::NodeStatus::FAILURE;
  }

  // No CV data has arrived yet.
  if (!have_blocks_ || latest_blocks_.blocks.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  const bool match_alliance = BT_Util::get_input<bool>(this, "match_alliance", true);
  const double max_range_tiles = BT_Util::get_input<double>(this, "max_range_tiles", -1.0);
  const double max_age_s = BT_Util::get_input<double>(this, "max_age_s", -1.0);

  // Our alliance colour (defaults to red if the flag was never published).
  bool is_red_alliance = true;
  BT_Util::get_from_blackboard(blackboard_, "is_red_alliance", is_red_alliance, true);

  // <= 0 disables the range filter.
  const double max_range_m =
    (max_range_tiles > 0.0) ? max_range_tiles * ghost_util::TILES_TO_METERS : -1.0;

  // Robot position in the map frame (matches the blocks' frame).
  const Eigen::Vector3d world_pose = tank_model_ptr_->getWorldPose();
  const double robot_x = world_pose.x();
  const double robot_y = world_pose.y();

  const int idx = selectClosestBlockIndex(
    latest_blocks_.blocks, robot_x, robot_y,
    match_alliance, is_red_alliance, max_range_m, max_age_s,
    node_ptr_->now().seconds());

  if (idx < 0) {
    return BT::NodeStatus::FAILURE;
  }

  const auto & best = latest_blocks_.blocks[idx];
  const double dist_m = std::hypot(best.x - robot_x, best.y - robot_y);

  setOutput("block_x_tiles", best.x / ghost_util::TILES_TO_METERS);
  setOutput("block_y_tiles", best.y / ghost_util::TILES_TO_METERS);
  setOutput("block_id", static_cast<int>(best.id));
  setOutput("block_dist_tiles", dist_m / ghost_util::TILES_TO_METERS);

  RCLCPP_INFO(
    node_ptr_->get_logger(),
    "[GetClosestBlock] closest %s block id=%u at map (%.2f, %.2f) m, %.2f m away",
    best.is_red ? "red" : "blue", best.id, best.x, best.y, dist_m);

  return BT::NodeStatus::SUCCESS;
}

} // namespace ghost_tank
