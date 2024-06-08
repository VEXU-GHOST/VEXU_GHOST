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

#include "ghost_v5/digital/v5_digital_actuator_interface.hpp"

using namespace ghost_v5_interfaces::devices;

namespace ghost_v5 {

V5DigitalActuatorInterface::V5DigitalActuatorInterface(std::shared_ptr<const DigitalDeviceConfig> config_ptr){
	config_ptr_ = config_ptr->clone()->as<const DigitalDeviceConfig>();
	digital_actuator_ptr_ = std::make_shared<pros::ADIDigitalOut>(config_ptr->port - 21);
}

void V5DigitalActuatorInterface::updateInterface(){
    digital_actuator_ptr_->set_value(value_);
}

} // namespace ghost_v5