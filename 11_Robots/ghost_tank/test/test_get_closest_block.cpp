#include "ghost_tank/bt_nodes/get_closest_block.hpp"
#include "gtest/gtest.h"

using push_back_cv::msg::FieldBlock;
using ghost_tank::selectClosestBlockIndex;

namespace
{
// Build a block. last_seen_sec == 0 means "no real stamp" (age filter skipped).
FieldBlock mk(uint32_t id, double x, double y, bool is_red, int last_seen_sec = 0)
{
  FieldBlock b;
  b.id = id;
  b.x = x;
  b.y = y;
  b.z = 0.0;
  b.is_red = is_red;
  b.confidence = 0.9f;
  b.last_seen.sec = last_seen_sec;
  b.last_seen.nanosec = 0;
  return b;
}
}  // namespace

// Robot at origin, red alliance: pick the nearest RED block, ignore the blue one.
TEST(GetClosestBlock, PicksNearestAllianceColour)
{
  std::vector<FieldBlock> blocks = {
    mk(1, 0.5, 0.5, true),    // dist ~0.71  <- nearest red
    mk(2, 2.0, 1.0, true),    // dist ~2.24
    mk(3, 0.3, 0.2, false),   // blue (nearest overall, must be ignored)
  };
  const int idx = selectClosestBlockIndex(blocks, 0.0, 0.0, true, true, -1, -1, 0);
  ASSERT_EQ(idx, 0);
  EXPECT_EQ(blocks[idx].id, 1u);
}

// Nearest overall is blue; with alliance filtering we must skip it for the red one.
TEST(GetClosestBlock, IgnoresWrongColour)
{
  std::vector<FieldBlock> blocks = {mk(1, 1.0, 0.0, true), mk(3, 0.1, 0.0, false)};
  const int idx = selectClosestBlockIndex(blocks, 0.0, 0.0, true, true, -1, -1, 0);
  ASSERT_EQ(idx, 0);
  EXPECT_EQ(blocks[idx].id, 1u);
}

// match_alliance=false -> nearest regardless of colour (the blue one here).
TEST(GetClosestBlock, MatchAllianceFalseIgnoresColour)
{
  std::vector<FieldBlock> blocks = {mk(1, 1.0, 0.0, true), mk(3, 0.1, 0.0, false)};
  const int idx = selectClosestBlockIndex(blocks, 0.0, 0.0, false, true, -1, -1, 0);
  ASSERT_EQ(idx, 1);
  EXPECT_EQ(blocks[idx].id, 3u);
}

// Range filter excludes everything beyond max_range_m.
TEST(GetClosestBlock, RespectsMaxRange)
{
  std::vector<FieldBlock> blocks = {mk(1, 1.0, 0.0, true), mk(2, 2.0, 0.0, true)};
  EXPECT_EQ(selectClosestBlockIndex(blocks, 0.0, 0.0, true, true, 0.5, -1, 0), -1);
  // Loosen the range and the nearer one comes back.
  EXPECT_EQ(selectClosestBlockIndex(blocks, 0.0, 0.0, true, true, 1.5, -1, 0), 0);
}

// Age filter excludes stale tracks; fresh ones pass.
TEST(GetClosestBlock, RespectsMaxAge)
{
  std::vector<FieldBlock> blocks = {mk(1, 1.0, 0.0, true, /*last_seen_sec=*/100)};
  // now=110 -> age 10s > 5s limit -> excluded.
  EXPECT_EQ(selectClosestBlockIndex(blocks, 0.0, 0.0, true, true, -1, 5.0, 110.0), -1);
  // now=103 -> age 3s < 5s -> included.
  EXPECT_EQ(selectClosestBlockIndex(blocks, 0.0, 0.0, true, true, -1, 5.0, 103.0), 0);
}

// Robot pose is taken into account (closest is relative to the robot, not origin).
TEST(GetClosestBlock, NearestIsRelativeToRobot)
{
  std::vector<FieldBlock> blocks = {mk(1, 0.0, 0.0, true), mk(2, 2.0, 0.0, true)};
  // Robot near (2,0): block id 2 is the closest.
  const int idx = selectClosestBlockIndex(blocks, 1.9, 0.0, true, true, -1, -1, 0);
  ASSERT_EQ(idx, 1);
  EXPECT_EQ(blocks[idx].id, 2u);
}

// No blocks -> no selection.
TEST(GetClosestBlock, EmptyReturnsNone)
{
  std::vector<FieldBlock> blocks;
  EXPECT_EQ(selectClosestBlockIndex(blocks, 0.0, 0.0, true, true, -1, -1, 0), -1);
}
