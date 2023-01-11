#ifndef GHOST_PROS__V5_GLOBALS_HPP
#define GHOST_PROS__V5_GLOBALS_HPP

#include <map>

#include "pros/apix.h"
#include "ghost_v5/serial/v5_serial_node.hpp"
#include "ghost_v5/motor/ghost_motor.hpp"
#include "ghost_ros/robot_config/v5_port_config.hpp"
namespace v5_globals
{
    extern uint32_t last_cmd_time;
    extern uint32_t cmd_timeout_ms;
    extern uint32_t loop_frequency;
    extern bool run;

    extern pros::Controller controller_main;

    extern pros::Mutex actuator_update_lock;

    extern std::map<ghost_v5_config::v5_motor_id_enum, std::shared_ptr<ghost_v5::GhostMotor>> motors;
    extern std::map<ghost_v5_config::v5_sensor_id_enum, std::shared_ptr<pros::Rotation>> encoders;

    extern const pros::controller_analog_e_t joy_channels[4];
    extern const pros::controller_digital_e_t joy_btns[12];

    extern pros::ADIPort adi_ports[8];

    // Serial Port
    extern ghost_v5::V5SerialNode serial_node_;

} // namespace v5_globals

#endif // GHOST_PROS__V5_GLOBALS_HPP