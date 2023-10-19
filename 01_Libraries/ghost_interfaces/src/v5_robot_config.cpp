/**
 * This file should be modified for your specific robot!
 *
 *
 *
 *
 */

#include "ghost_interfaces/v5_robot_config_defs.hpp"

namespace ghost_interfaces {

//////////////////////////////////////////
////////// Motor Configurations //////////
//////////////////////////////////////////
const V5MotorInterfaceConfig drive_motor_config = {
	.gear_ratio = ghost_gearset::GEARSET_600,
	.brake_mode = ghost_brake_mode::BRAKE_MODE_COAST,
	// .ctl__vel_gain = 17.5,  // RPM -> mV
	// .ctl__ff_vel_gain = 1.1f,
};

const V5MotorInterfaceConfig intake_motor_config = {
	.gear_ratio = ghost_gearset::GEARSET_600,
	.brake_mode = ghost_brake_mode::BRAKE_MODE_BRAKE,
	// .ctl__vel_gain = 25.0,
	// .ctl__ff_vel_gain = 1.2f,
};

///////////////////////////////////////
////////// Motor Definitions //////////
///////////////////////////////////////
const std::unordered_map<std::string, motor_access_helper> motor_config_map{
	{"DRIVE_LEFT_FRONT_LEFT_MOTOR",  motor_access_helper(1, false, drive_motor_config)},
	{"DRIVE_LEFT_FRONT_RIGHT_MOTOR",  motor_access_helper(2, false, drive_motor_config)},
	{"DRIVE_LEFT_BACK_LEFT_MOTOR",   motor_access_helper(3, false, drive_motor_config)},
	{"DRIVE_LEFT_BACK_RIGHT_MOTOR",   motor_access_helper(4, false, drive_motor_config)},
	{"DRIVE_RIGHT_FRONT_LEFT_MOTOR",  motor_access_helper(5, false, drive_motor_config)},
	{"DRIVE_RIGHT_FRONT_RIGHT_MOTOR",  motor_access_helper(6, false, drive_motor_config)},
	{"DRIVE_RIGHT_BACK_LEFT_MOTOR",  motor_access_helper(7, false, drive_motor_config)},
	{"DRIVE_RIGHT_BACK_RIGHT_MOTOR",  motor_access_helper(8, false, drive_motor_config)},
};

/////////////////////////////////////////
////////// Encoder Definitions //////////
/////////////////////////////////////////
const std::unordered_map<std::string, encoder_access_helper> encoder_config_map{
	{"STEERING_FRONT_LEFT_ENCODER",     encoder_access_helper(9, true)},
	{"STEERING_FRONT_RIGHT_ENCODER",    encoder_access_helper(10, true)},
	{"STEERING_BACK_LEFT_ENCODER",      encoder_access_helper(11, true)},
	{"STEERING_BACK_RIGHT_ENCODER",     encoder_access_helper(12, true)},
};

} // namespace ghost_interfaces