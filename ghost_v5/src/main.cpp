#include "main.h"
#include "pros/motors.h"

#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <unistd.h>

// Globals
uint32_t last_cmd_time_ = 0;
bool run_ = true;

/*
	------ Producer Message Encoding ------
	PROS Overhead:
	5x Bytes - Packet Header (NULL + "sout")
	1x Byte - Delimiter (\x00)
	Sum: 6x Bytes -> 0.41ms

	Six Independent Drive Motors:
		4x Bytes - int32_t for Raw Position
	Sum: 24x Bytes -> 1.67ms

	Each V5 Swerve Encoder (Three on a drivetrain, One on Turret):
		4x Bytes - int32_t for Angle
	Sum: 16x Bytes -> 1.11ms
	
	For each of four joystick channels:
		4x Bytes - int32_t for Joystick Value
	Sum: 16x Bytes -> 1.11ms

	Misc:
		0.5 Bytes - Competition Mode
		1.5 Bytes - Joystick Buttons
		1x Byte - Digital Outs
		1x Byte - Checksum
	Sum: 4x Bytes -> 0.27ms
	
	Total Sum: 66 Byte Packet
	66 Bytes x 8 bits / byte * 1 sec / 115200 bits * 1000ms / 1s = 4.58ms
	*/
void producer_main(){
	int sout_int = fileno(stdout);
	char buffer[] = {66, 0, 10};
	pros::c::fdctl(sout_int, SERCTL_DEACTIVATE, NULL);

	for(int i = 0; i < 100; i++){
		write(sout_int, buffer, sizeof(buffer));
	}

	while(run_){
		// Poll sensor data

		// Get Competition Mode
		
		// Compress to serial msg

		// Add Checksum

		// Send
		std::cout << "Producer" << std::endl;
		pros::delay(250);
	}
	fclose(stdout);
}

/*
	------ Producer Message Encoding ------
	PROS Overhead:
	5x Bytes - Packet Header (NULL + "sout")
	1x Byte - Delimiter (\x00)
	Sum: 6x Bytes -> 0.41ms

	2x Flywheel Motors
		4x Bytes - int32_t for Velocity Command
		2x Bytes - int16_t for Voltage Command
	Sum: 12 Bytes -> 0.83ms

	6x Independent Drive Motors, 1x Turret:
		4x Bytes - int32_t for Velocity Command
		2x Bytes - int16_t for Voltage Command
	Sum: 42 Bytes -> 2.92ms

	Misc:
		1x Byte - Checksum
	Sum: 1x Byte -> 0.07ms
	
	Total Sum: 61 Byte Packet
	 61 Bytes x 8 bits / byte * 1 sec / 115200 bits * 1000ms / 1s = 4.24ms
	*/
void consumer_main(){
	while(run_){
		// Read serial buffer
		
		// Collect packets
		
		// Verify Check Sum
		
		// Actuator Mutex
		
		// Update Actuators
		// std::cout << "Consumer" << std::endl;
		pros::delay(500);
	}
}

void actuator_timeout_main(){
	while(run_){
		std::cout << "Timeout" << std::endl;
		pros::delay(1000);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();

	// Start Serial Interface tasks
	pros::Task producer_thread(producer_main, "producer thread");
	pros::Task consumer_thread(consumer_main, "consumer thread");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	while(pros::competition::is_disabled()){
		std::cout << "Disabled" << std::endl;
		pros::delay(10);
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
void competition_initialize() {}

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
void autonomous() {
	while(pros::competition::is_autonomous()){
		std::cout << "Auton" << std::endl;
		pros::delay(10);
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
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	while(run_){
		pros::delay(10);
	}
}