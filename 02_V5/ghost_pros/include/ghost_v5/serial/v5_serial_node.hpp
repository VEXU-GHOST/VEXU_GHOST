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

#include <atomic>
#include <memory>

#include "pros/apix.h"

#include "ghost_serial/base_interfaces/v5_serial_base.hpp"
#include "ghost_util/byte_utils.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"

namespace ghost_v5 {

class V5SerialNode {
public:
	V5SerialNode(std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> robot_hardware_interface_ptr);
	~V5SerialNode();

	void initSerial();
	bool readV5ActuatorUpdate();
	void writeV5StateUpdate();

private:
	void updateActuatorCommands(std::vector<unsigned char>& buffer);

	// Device Config
	std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> hardware_interface_ptr_;

	// Serial Interface
	std::unique_ptr<ghost_serial::V5SerialBase> serial_base_interface_;
	std::vector<unsigned char> new_msg_;
	int actuator_command_msg_len_;
	int sensor_update_msg_len_;

	// Reader Thread
	std::unique_ptr<pros::Task> reader_thread_;
	std::atomic_bool reader_thread_init_;

	// Msg IDs
	uint32_t read_msg_id_;
	uint32_t write_msg_id_;
};

} // namespace ghost_v5