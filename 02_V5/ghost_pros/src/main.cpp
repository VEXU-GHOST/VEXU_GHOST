#include "main.h"
#include "pros/motors.h"

#include <algorithm>
#include <iostream>
#include <numeric>
#include <vector>
#include <unistd.h>

#include "pros/apix.h"
#include "pros/rtos.h"

#include "ghost_v5_interfaces/util/device_type_helpers.hpp"

#include "ghost_v5/motor/v5_motor_interface.hpp"
#include "ghost_v5/screen/screen_interface.hpp"
#include "ghost_v5/serial/v5_serial_node.hpp"

using ghost_v5_interfaces::devices::hardware_type_e::V5_BRAIN;
using namespace ghost_v5;
using namespace ghost_v5_interfaces::devices;
using namespace ghost_v5_interfaces::util;
using namespace ghost_v5_interfaces;

void exit_main_loop(const std::exception& e){
	v5_globals::run = false;

	// Clear current screen data
	v5_globals::screen_interface_ptr->reset();

	// Update screen with error data
	v5_globals::screen_interface_ptr->setTitle("ERROR");
	v5_globals::screen_interface_ptr->addToPrintQueue(e.what());
}

void zero_actuators(){
	std::unique_lock<pros::Mutex> actuator_lock(v5_globals::actuator_update_lock);

	// Zero all motor commands
	for(auto & m : v5_globals::motor_interfaces){
		m.second->setControlMode(false, false, false, false);
		m.second->setMotorCommand(0.0, 0.0, 0.0, 0.0);
	}

	// // Zero Pneumatics
	// for(int i = 0; i < 8; i++){
	// 	v5_globals::adi_ports[i].set_value(false);
	// }
	actuator_lock.unlock();
}

void update_actuators(){
	std::unique_lock<pros::Mutex> actuator_lock(v5_globals::actuator_update_lock);

	// Update velocity filter and motor controller for all motors
	for(auto & m : v5_globals::motor_interfaces){
		m.second->updateInterface();
	}

	// Update Pneumatics
	for(int i = 0; i < 8; i++){
		v5_globals::adi_ports[i].set_value(v5_globals::digital_out_cmds[i]);
	}
	actuator_lock.unlock();
}

void screen_update_loop(){
	uint32_t loop_time = pros::millis();
	auto refresh_rate = v5_globals::screen_interface_ptr->getRefreshRateMilliseconds();
	while(true){
		v5_globals::screen_interface_ptr->updateScreen();
		pros::c::task_delay_until(&loop_time, refresh_rate);
	}
}

void actuator_timeout_loop(){
	uint32_t loop_time = pros::millis();
	while(v5_globals::run){
		try{
			if(pros::millis() > v5_globals::last_cmd_time + v5_globals::cmd_timeout_ms){
				zero_actuators();
			}
			pros::c::task_delay_until(&loop_time, v5_globals::cmd_timeout_ms);
		}
		catch(std::exception& e){
			exit_main_loop(e);
		}
	}
}

void reader_loop(){
	uint32_t loop_time = pros::millis();
	while(v5_globals::run){
		try{
			// Process incoming msgs and update motor cmds
			bool update_recieved = v5_globals::serial_node_ptr->readV5ActuatorUpdate();

			// Reset actuator timeout
			if(update_recieved){
				v5_globals::last_cmd_time = pros::millis();
			}
			// Reader thread blocks waiting for data, so loop frequency must run faster than producer to avoid msg queue backup
			pros::c::task_delay_until(&loop_time, v5_globals::loop_frequency / 2);
		}
		catch(std::exception& e){
			exit_main_loop(e);
		}
	}
}

void ghost_main_loop(){
	static int count = 0;
	try{
		// Send robot state over serial to coprocessor
		v5_globals::serial_node_ptr->writeV5StateUpdate();

		// Zero All Motors if disableds
		if(pros::competition::is_disabled()){
			zero_actuators();
		}

		update_actuators();
	}
	catch(std::exception& e){
		exit_main_loop(e);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize(){
	try{
		// Setup LCD Screen
		v5_globals::screen_interface_ptr = std::make_shared<ghost_v5::ScreenInterface>();
		pros::Task screen_output_thread(screen_update_loop, "screen_update_thread");

		// Get Robot Device Configuration from compile-time generated file
		v5_globals::robot_device_config_map_ptr.reset(getRobotConfig());

		// Instantiate Hardware Interface and Serial Node
		v5_globals::robot_hardware_interface_ptr = std::make_shared<RobotHardwareInterface>(v5_globals::robot_device_config_map_ptr, V5_BRAIN);
		v5_globals::serial_node_ptr = std::make_shared<V5SerialNode>(v5_globals::robot_hardware_interface_ptr);

		for(const auto& [device_name, config_ptr] : *v5_globals::robot_device_config_map_ptr){
			switch(config_ptr->type){
				case device_type_e::MOTOR:
				{
					auto motor_config_ptr = config_ptr->as<const MotorDeviceConfig>();
					v5_globals::motor_interfaces[device_name] = std::make_shared<ghost_v5::V5MotorInterface>(motor_config_ptr);
				}
				break;

				case device_type_e::ROTATION_SENSOR:
				{
					auto rotation_sensor_config_ptr = config_ptr->as<const RotationSensorDeviceConfig>();
					v5_globals::encoders[device_name] = std::make_shared<pros::Rotation>(rotation_sensor_config_ptr->port);
					if(rotation_sensor_config_ptr->reversed){
						v5_globals::encoders[device_name]->reverse();
					}
					v5_globals::encoders[device_name]->set_data_rate(rotation_sensor_config_ptr->data_rate);
				}
				break;

				case device_type_e::JOYSTICK:
					// Do nothing, these are initialized already
				break;

				case device_type_e::INVALID:
				{
					std::string err_string = "ERROR: Device type is listed as INVALID for device_name: ";
					throw std::runtime_error(err_string + device_name);
				}
				break;

				default:
				{
					std::string err_string = "ERROR: Attempted to initialize unsupported device using "
					                         "ghost_v5_interfaces. Device Name: " + device_name + " Device Type: ";
					if(DEVICE_TYPE_TO_STRING_MAP.count(config_ptr->type) != 0){
						err_string += DEVICE_TYPE_TO_STRING_MAP.at(config_ptr->type);
					}
					else{
						err_string += std::to_string(config_ptr->type);
					}
					throw std::runtime_error(err_string);
				}
				break;
			}
		}

		zero_actuators();
		for(int i = 0; i < 8; i++){
			v5_globals::adi_ports[i].set_value(false);
		}
		v5_globals::serial_node_ptr->initSerial();
		pros::Task reader_thread(reader_loop, "reader thread");
		pros::Task actuator_timeout_thread(actuator_timeout_loop, "actuator timeout thread");
	}
	catch(std::exception &e){
		exit_main_loop(e);
	}
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled(){
	uint32_t loop_time = pros::millis();
	while(pros::competition::is_disabled() && v5_globals::run){
		ghost_main_loop();
		pros::c::task_delay_until(&loop_time, v5_globals::loop_frequency);
	}
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous(){
	uint32_t loop_time = pros::millis();
	while(pros::competition::is_autonomous() && v5_globals::run){
		ghost_main_loop();
		pros::c::task_delay_until(&loop_time, v5_globals::loop_frequency);
	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol(){
	uint32_t loop_time = pros::millis();
	while(!pros::competition::is_autonomous() && !pros::competition::is_disabled() && v5_globals::run){
		ghost_main_loop();
		pros::c::task_delay_until(&loop_time, v5_globals::loop_frequency);
	}
}

// void opcontrol(){
// 	uint32_t loop_time = pros::millis();
// 	auto m1 = pros::Motor(11, pros::motor_gearset_e_t::E_MOTOR_GEAR_600);
// 	pros::Controller joy (pros::E_CONTROLLER_MASTER);

// 	std::cout << "Voltage, Velocity, Current, Torque, Power, Efficiency, Temperature" << std::endl;

// 	while(!pros::competition::is_autonomous() && !pros::competition::is_disabled()){
// 		m1.move_voltage(joy.get_analog(ANALOG_RIGHT_Y) / 127.0 * 12000.0);

// 		std::cout << m1.get_voltage() << ", " << m1.get_actual_velocity() << ", " << m1.get_current_draw()
// 		          << ", " << m1.get_torque() << ", " << m1.get_power() << ", " << m1.get_efficiency() << ", "
// 		          << m1.get_temperature() << std::endl;


// 		pros::c::task_delay_until(&loop_time, 10);
// 	}
// }