#include "ghost_v5/motor/v5_motor_interface.hpp"

using namespace ghost_v5_interfaces::devices;

namespace ghost_v5 {

// PROS Config Mappings
const std::unordered_map<ghost_gearset, pros::motor_gearset_e_t> RPM_TO_GEARING_MAP{
	{ghost_gearset::GEARSET_100,    pros::E_MOTOR_GEAR_100},
	{ghost_gearset::GEARSET_200,    pros::E_MOTOR_GEAR_200},
	{ghost_gearset::GEARSET_600,    pros::E_MOTOR_GEAR_600}};

const std::unordered_map<ghost_brake_mode, pros::motor_brake_mode_e_t> GHOST_BRAKE_MODE_MAP{
	{ghost_brake_mode::BRAKE_MODE_COAST,   pros::E_MOTOR_BRAKE_COAST},
	{ghost_brake_mode::BRAKE_MODE_BRAKE,   pros::E_MOTOR_BRAKE_BRAKE},
	{ghost_brake_mode::BRAKE_MODE_HOLD,    pros::E_MOTOR_BRAKE_HOLD},
	{ghost_brake_mode::BRAKE_MODE_INVALID, pros::E_MOTOR_BRAKE_INVALID}};

const std::unordered_map<ghost_encoder_unit, pros::motor_encoder_units_e_t> GHOST_ENCODER_UNIT_MAP{
	{ghost_encoder_unit::ENCODER_DEGREES,   pros::E_MOTOR_ENCODER_DEGREES},
	{ghost_encoder_unit::ENCODER_ROTATIONS, pros::E_MOTOR_ENCODER_ROTATIONS},
	{ghost_encoder_unit::ENCODER_COUNTS,    pros::E_MOTOR_ENCODER_COUNTS},
	{ghost_encoder_unit::ENCODER_INVALID,   pros::E_MOTOR_ENCODER_INVALID}};

V5MotorInterface::V5MotorInterface(std::shared_ptr<const MotorDeviceConfig> config_ptr) :
	MotorController(config_ptr->controller_config, config_ptr->filter_config, config_ptr->model_config),
	device_connected_{false}{
	config_ptr_ = config_ptr->clone()->as<const MotorDeviceConfig>();
	motor_interface_ptr_ = std::make_shared<pros::Motor>(
		config_ptr_->port,
		pros::E_MOTOR_GEARSET_INVALID,
		config_ptr_->reversed,
		pros::E_MOTOR_ENCODER_INVALID);

	// Set Motor Config w PROS Enum values
	motor_interface_ptr_->set_gearing(RPM_TO_GEARING_MAP.at(config_ptr_->gearset));
	motor_interface_ptr_->set_brake_mode(GHOST_BRAKE_MODE_MAP.at(config_ptr_->brake_mode));
	motor_interface_ptr_->set_encoder_units(GHOST_ENCODER_UNIT_MAP.at(config_ptr_->encoder_units));
}

void V5MotorInterface::updateInterface(){
	float position = motor_interface_ptr_->get_position();
	float velocity = motor_interface_ptr_->get_actual_velocity();
	device_connected_ = (velocity != PROS_ERR_F);

	float cmd_voltage = updateMotor(position, velocity);

	if(controllerActive()){
		motor_interface_ptr_->move_voltage(cmd_voltage);
	}
	else{
		motor_interface_ptr_->set_current_limit(0);
		motor_interface_ptr_->move_voltage(0);
	}
}

} // namespace ghost_v5