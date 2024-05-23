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

#include "ghost_v5_interfaces/devices/joystick_device_interface.hpp"
#include "ghost_v5_interfaces/test/device_test_utils.hpp"

#include <stdlib.h>
#include "gtest/gtest.h"

using ghost_v5_interfaces::devices::hardware_type_e;
using ghost_v5_interfaces::devices::JoystickDeviceData;
using namespace ghost_v5_interfaces::test_util;

TEST(TestJoystickDeviceInterface, testEqualityOperator) {
  auto j1 = getRandomJoystickData();
  auto j2 = std::make_shared<JoystickDeviceData>(*j1);
  EXPECT_EQ(*j1, *j2);
}

TEST(TestJoystickDeviceInterface, testSerializationCoprocessorToV5) {
  auto j1 = getRandomJoystickData();
  auto j2 = std::make_shared<JoystickDeviceData>("joy_test");
  auto j3 = std::make_shared<JoystickDeviceData>("joy_test");
  j2->name = j1->name;
  j3->name = j1->name;

  // Test Serialization from Coprocessor -> V5
  // We expect this to do nothing, joysticks dont go from Coprocessor to V5
  auto serial_stream_1 = j1->serialize(hardware_type_e::COPROCESSOR);
  j2->deserialize(serial_stream_1, hardware_type_e::V5_BRAIN);

  EXPECT_EQ(*j2, *j3);
}

TEST(TestJoystickDeviceInterface, testSerializationV5ToCoprocessor) {
  auto j1 = getRandomJoystickData();
  auto j2 = std::make_shared<JoystickDeviceData>("joy_test");
  j2->name = j1->name;

  // Test Serialization from Coprocessor -> V5
  auto serial_stream_1 = j1->serialize(hardware_type_e::V5_BRAIN);
  j2->deserialize(serial_stream_1, hardware_type_e::COPROCESSOR);

  EXPECT_EQ(*j1, *j2);
}

TEST(TestJoystickDeviceInterface, testClone) {
  auto data_1 = getRandomJoystickData();
  auto data_2 = data_1->clone()->as<JoystickDeviceData>();
  EXPECT_EQ(*data_1, *data_2);
}

TEST(TestJoystickDeviceInterface, testSerialMsgLengths) {
  auto j1 = getRandomJoystickData();
  auto serial_stream_1 = j1->serialize(hardware_type_e::V5_BRAIN);
  EXPECT_EQ(serial_stream_1.size(), j1->getSensorPacketSize());

  auto serial_stream_2 = j1->serialize(hardware_type_e::COPROCESSOR);
  EXPECT_EQ(serial_stream_2.size(), j1->getActuatorPacketSize());
}
