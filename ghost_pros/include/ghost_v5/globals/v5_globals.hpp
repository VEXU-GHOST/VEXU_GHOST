#ifndef GHOST_PROS__V5_GLOBALS_HPP
#define GHOST_PROS__V5_GLOBALS_HPP

#include "pros/apix.h"
#include "ghost_v5/serial/v5_serial_node.hpp"

namespace v5_globals{

extern uint32_t last_cmd_time_;
extern bool run;

extern pros::Motor drive_motors[8];
extern pros::Motor turret_motor;

extern pros::Rotation encoders[3];

extern pros::Controller controller_main;
extern const pros::controller_analog_e_t joy_channels[4];

extern const pros::controller_digital_e_t joy_btns[12];

extern pros::ADIPort digital_out[8];

// Serial Port
extern ghost_v5::V5SerialNode serial_node_;

} // namespace v5_globals

#endif // GHOST_PROS__V5_GLOBALS_HPP