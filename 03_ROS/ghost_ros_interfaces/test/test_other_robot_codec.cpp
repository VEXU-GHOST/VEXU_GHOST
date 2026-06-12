/*
 *   Copyright (c) 2024 Maxx Wilson
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

#include <gtest/gtest.h>
#include "ghost_ros_interfaces/msg_helpers/msg_helpers.hpp"

using namespace ghost_ros_interfaces::msg_helpers;
using ghost_v5_interfaces::inter_robot::OTHER_ROBOT_PACKET_SIZE;
using ghost_v5_interfaces::inter_robot::OtherRobotBytes;

// The wire format must be exactly the documented size, with no CDR/encapsulation/padding overhead.
TEST(TestOtherRobotCodec, wireSizeIsSeven) {
  EXPECT_EQ(OTHER_ROBOT_PACKET_SIZE, 7u);
}

// pack -> unpack must be the identity on every field.
TEST(TestOtherRobotCodec, packUnpackRoundTrip) {
  ghost_msgs::msg::OtherRobot in{};
  in.x = 12;
  in.y = 200;
  in.theta = 64;
  in.status = ghost_msgs::msg::OtherRobot::STATUS_SCORING;
  in.target_x = 33;
  in.target_y = 250;
  in.seq = 7;

  OtherRobotBytes bytes{};
  packOtherRobot(in, bytes);

  ghost_msgs::msg::OtherRobot out{};
  unpackOtherRobot(bytes, out);

  EXPECT_EQ(in.x, out.x);
  EXPECT_EQ(in.y, out.y);
  EXPECT_EQ(in.theta, out.theta);
  EXPECT_EQ(in.status, out.status);
  EXPECT_EQ(in.target_x, out.target_x);
  EXPECT_EQ(in.target_y, out.target_y);
  EXPECT_EQ(in.seq, out.seq);
}

// The byte order must be stable (a peer on a different build must agree on the layout).
TEST(TestOtherRobotCodec, byteOrderIsStable) {
  ghost_msgs::msg::OtherRobot in{};
  in.x = 1;
  in.y = 2;
  in.theta = 3;
  in.status = 4;
  in.target_x = 5;
  in.target_y = 6;
  in.seq = 7;

  OtherRobotBytes bytes{};
  packOtherRobot(in, bytes);

  const OtherRobotBytes expected{1, 2, 3, 4, 5, 6, 7};
  EXPECT_EQ(bytes, expected);
}

// A VERSION packet carries the protocol version in target_x; the codec stays a pure byte copy.
TEST(TestOtherRobotCodec, versionPacketRoundTrip) {
  ghost_msgs::msg::OtherRobot in{};
  in.status = ghost_msgs::msg::OtherRobot::STATUS_VERSION;
  in.target_x = 42;  // protocol version

  OtherRobotBytes bytes{};
  packOtherRobot(in, bytes);

  ghost_msgs::msg::OtherRobot out{};
  unpackOtherRobot(bytes, out);

  EXPECT_EQ(out.status, ghost_msgs::msg::OtherRobot::STATUS_VERSION);
  EXPECT_EQ(out.target_x, 42);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
