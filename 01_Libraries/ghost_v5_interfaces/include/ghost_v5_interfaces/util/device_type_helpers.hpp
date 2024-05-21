#pragma once

#include "ghost_v5_interfaces/devices/device_config_map.hpp"

namespace ghost_v5_interfaces {

namespace util {

// Maps device types from string to enum
const std::unordered_map<std::string, devices::device_type_e> STRING_TO_DEVICE_TYPE_MAP{
	{"MOTOR",           devices::device_type_e::MOTOR},
	{"ROTATION_SENSOR", devices::device_type_e::ROTATION_SENSOR},
	{"JOYSTICK",        devices::device_type_e::JOYSTICK},
	{"INERTIAL_SENSOR", devices::device_type_e::INERTIAL_SENSOR},
	{"DISTANCE_SENSOR", devices::device_type_e::DISTANCE_SENSOR},
	{"OPTICAL_SENSOR",  devices::device_type_e::OPTICAL_SENSOR},
	{"VISION_SENSOR",   devices::device_type_e::VISION_SENSOR},
	{"GPS_SENSOR",      devices::device_type_e::GPS_SENSOR},
	{"RADIO",           devices::device_type_e::RADIO},
	{"INVALID",         devices::device_type_e::INVALID},
};
const std::unordered_map<devices::device_type_e, std::string> DEVICE_TYPE_TO_STRING_MAP{
	{devices::device_type_e::MOTOR,                 "MOTOR"},
	{devices::device_type_e::ROTATION_SENSOR,       "ROTATION_SENSOR"},
	{devices::device_type_e::JOYSTICK,              "JOYSTICK"},
	{devices::device_type_e::INERTIAL_SENSOR,       "INERTIAL_SENSOR"},
	{devices::device_type_e::DISTANCE_SENSOR,       "DISTANCE_SENSOR"},
	{devices::device_type_e::OPTICAL_SENSOR,        "OPTICAL_SENSOR"},
	{devices::device_type_e::VISION_SENSOR,         "VISION_SENSOR"},
	{devices::device_type_e::GPS_SENSOR,            "GPS_SENSOR"},
	{devices::device_type_e::RADIO,                 "RADIO"},
	{devices::device_type_e::INVALID,               "INVALID"}
};

} // namespace ghost_v5_interfaces

} // namespace util