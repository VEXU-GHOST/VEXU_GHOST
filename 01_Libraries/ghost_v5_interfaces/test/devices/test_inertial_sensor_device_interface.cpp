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

#include "ghost_v5_interfaces/devices/inertial_sensor_device_interface.hpp"
#include "ghost_v5_interfaces/test/device_test_utils.hpp"


#include <stdlib.h>
#include "gtest/gtest.h"

using ghost_v5_interfaces::devices::hardware_type_e;
using ghost_v5_interfaces::devices::InertialSensorDeviceConfig;
using ghost_v5_interfaces::devices::InertialSensorDeviceData;
using ghost_v5_interfaces::test_util::getRandomInertialSensorData;
using ghost_v5_interfaces::test_util::getRandomInertialSensorSerialConfig;

const int NUM_TESTS = 50;
TEST(TestInertialSensorDeviceInterface, testSerialization) {
  for (int i = 0; i < NUM_TESTS; i++) {
    auto serial_config = getRandomInertialSensorSerialConfig();
    auto data_1 = getRandomInertialSensorData(serial_config);
    auto data_2 = std::make_shared<InertialSensorDeviceData>("test", serial_config);
    auto serial_stream_1 = data_1->serialize(hardware_type_e::COPROCESSOR);
    data_2->deserialize(serial_stream_1, hardware_type_e::V5_BRAIN);

    auto serial_stream_2 = data_1->serialize(hardware_type_e::V5_BRAIN);
    data_2->deserialize(serial_stream_2, hardware_type_e::COPROCESSOR);

    EXPECT_EQ(*data_1, *data_2);
  }
}

TEST(TestJoystickDeviceInterface, testSerialMsgLengths) {
  for (int i = 0; i < NUM_TESTS; i++) {
    auto serial_config = getRandomInertialSensorSerialConfig();
    auto r1 = getRandomInertialSensorData(serial_config);
    EXPECT_EQ(r1->serialize(hardware_type_e::V5_BRAIN).size(), r1->getSensorPacketSize());
    EXPECT_EQ(r1->serialize(hardware_type_e::COPROCESSOR).size(), r1->getActuatorPacketSize());
  }
}
