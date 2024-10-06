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

#ifndef GHOST_PROS__V5_GLOBALS_HPP
#define GHOST_PROS__V5_GLOBALS_HPP

#include <atomic>
#include <deque>
#include <map>
#include <string>

#include "ghost_v5_interfaces/devices/device_config_map.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"

#include "ghost_v5/motor/v5_motor_interface.hpp"
#include "ghost_v5/screen/screen_interface.hpp"
#include "ghost_v5/serial/v5_serial_node.hpp"

#include "pros/apix.h"

namespace v5_globals {

extern uint32_t last_cmd_time;
extern uint32_t cmd_timeout_ms;
extern uint32_t loop_frequency;
extern std::atomic<bool> run;

extern pros::Controller controller_main;
extern pros::Controller controller_partner;

extern pros::Mutex actuator_update_lock;

extern std::shared_ptr<ghost_v5_interfaces::devices::DeviceConfigMap> robot_device_config_map_ptr;
extern std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> robot_hardware_interface_ptr;

extern std::unordered_map<std::string, std::shared_ptr<ghost_v5::V5MotorInterface> > motor_interfaces;
extern std::unordered_map<std::string, std::shared_ptr<pros::Rotation> > encoders;
extern std::unordered_map<std::string, std::shared_ptr<pros::Imu> > imus;

extern const pros::controller_analog_e_t joy_channels[4];
extern const pros::controller_digital_e_t joy_btns[12];

extern pros::ADIDigitalOut adi_ports[8];
extern std::vector<bool> digital_out_cmds;

// Serial Port
extern std::shared_ptr<ghost_v5::V5SerialNode> serial_node_ptr;

// Screen Interface
extern std::shared_ptr<ghost_v5::ScreenInterface> screen_interface_ptr;

} // namespace v5_globals

#endif // GHOST_PROS__V5_GLOBALS_HPP