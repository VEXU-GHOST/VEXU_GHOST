/*
 *   Copyright (c) 2026 Karmanyaah Malhotra
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

#include <array>
#include <cstddef>
#include <cstdint>

namespace ghost_v5_interfaces
{

namespace inter_robot
{

// Wire size, in bytes, of the OtherRobot inter-robot comms packet relayed over VEXlink.
//
// This constant is the ONLY piece of the inter-robot packet definition that crosses the
// ROS / no-ROS (V5 brain) boundary. The field layout itself lives solely in
// ghost_msgs/msg/OtherRobot.msg and is interpreted only on the ROS side (see
// ghost_ros_interfaces::msg_helpers::packOtherRobot / unpackOtherRobot). The V5 brain relays
// these bytes opaquely and never inspects them, so it needs nothing but the size to size its
// RobotHardwareInterface slot and its VEXlink transmit/receive calls.
//
// MUST equal the number of bytes written by packOtherRobot() — guarded by a static_assert there,
// so a change to OtherRobot.msg that is not mirrored here fails the build/unit tests.
constexpr std::size_t OTHER_ROBOT_PACKET_SIZE = 7;

// Fixed-size byte buffer holding exactly one serialized OtherRobot packet.
using OtherRobotBytes = std::array<std::uint8_t, OTHER_ROBOT_PACKET_SIZE>;

}  // namespace inter_robot

}  // namespace ghost_v5_interfaces
