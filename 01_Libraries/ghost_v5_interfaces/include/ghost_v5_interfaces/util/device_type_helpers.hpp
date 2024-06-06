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

#pragma once

#include "ghost_v5_interfaces/devices/device_config_map.hpp"

namespace ghost_v5_interfaces
{

namespace util
{

// Maps device types from string to enum
const std::unordered_map<std::string, devices::device_type_e> STRING_TO_DEVICE_TYPE_MAP{
  {"MOTOR", devices::device_type_e::MOTOR},
  {"ROTATION_SENSOR", devices::device_type_e::ROTATION_SENSOR},
  {"JOYSTICK", devices::device_type_e::JOYSTICK},
  {"INERTIAL_SENSOR", devices::device_type_e::INERTIAL_SENSOR},
  {"DISTANCE_SENSOR", devices::device_type_e::DISTANCE_SENSOR},
  {"OPTICAL_SENSOR", devices::device_type_e::OPTICAL_SENSOR},
  {"VISION_SENSOR", devices::device_type_e::VISION_SENSOR},
  {"GPS_SENSOR", devices::device_type_e::GPS_SENSOR},
  {"RADIO", devices::device_type_e::RADIO},
  {"DIGITAL", devices::device_type_e::DIGITAL},
  {"INVALID", devices::device_type_e::INVALID},
};
const std::unordered_map<devices::device_type_e, std::string> DEVICE_TYPE_TO_STRING_MAP{
  {devices::device_type_e::MOTOR, "MOTOR"},
  {devices::device_type_e::ROTATION_SENSOR, "ROTATION_SENSOR"},
  {devices::device_type_e::JOYSTICK, "JOYSTICK"},
  {devices::device_type_e::INERTIAL_SENSOR, "INERTIAL_SENSOR"},
  {devices::device_type_e::DISTANCE_SENSOR, "DISTANCE_SENSOR"},
  {devices::device_type_e::OPTICAL_SENSOR, "OPTICAL_SENSOR"},
  {devices::device_type_e::VISION_SENSOR, "VISION_SENSOR"},
  {devices::device_type_e::GPS_SENSOR, "GPS_SENSOR"},
  {devices::device_type_e::RADIO, "RADIO"},
  {devices::device_type_e::DIGITAL, "DIGITAL"},
  {devices::device_type_e::INVALID, "INVALID"}
};
const std::unordered_map<devices::digital_io_type_e, std::string> DIGITAL_IO_TYPE_TO_STRING_MAP{
  {devices::digital_io_type_e::SENSOR, "SENSOR"},
  {devices::digital_io_type_e::ACTUATOR, "ACTUATOR"}
};
const std::unordered_map<std::string, devices::digital_io_type_e> STRING_TO_DIGITAL_IO_TYPE_MAP{
  {"SENSOR", devices::digital_io_type_e::SENSOR},
  {"ACTUATOR", devices::digital_io_type_e::ACTUATOR}
};

} // namespace ghost_v5_interfaces

} // namespace util
