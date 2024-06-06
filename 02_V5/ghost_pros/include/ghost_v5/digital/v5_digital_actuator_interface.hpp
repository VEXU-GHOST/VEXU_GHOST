/*
 *   Copyright (c) 2024 Maxx Wilson, Xander Wilson
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
#include "ghost_v5_interfaces/devices/digital_device_interface.hpp"
#include "pros/apix.h"
#include "pros/error.h"
#include "pros/motors.hpp"

#include <unordered_map>

namespace ghost_v5 {

class V5DigitalActuatorInterface {
public:
	V5DigitalActuatorInterface(std::shared_ptr<const ghost_v5_interfaces::devices::DigitalDeviceConfig>);

	void updateInterface();

	bool getDeviceIsConnected(){
		return device_connected_;
	}

	void setValue(bool value){
		value_ = value;
	}

	std::shared_ptr<pros::ADIDigitalOut> getDigitalActuatorPtr(){
		return digital_actuator_ptr_;
	}

private:
    bool value_;
	std::shared_ptr<pros::ADIDigitalOut> digital_actuator_ptr_;
	bool device_connected_;
	std::shared_ptr<const ghost_v5_interfaces::devices::DigitalDeviceConfig> config_ptr_;
};

} // namespace ghost_v5