#include "main.h"
#include "pros/motors.h"

#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <unistd.h>

#include "pros/apix.h"

#include "ghost_serial/msg_parser/msg_parser.hpp"
#include "ghost_estimation/filters/first_order_low_pass_filter.hpp"
#include "ghost_estimation/filters/second_order_low_pass_filter.hpp"

// Globals
uint32_t last_cmd_time_ = 0;
bool run_ = true;

pros::Motor drive_motors[8] = {
	pros::Motor(1, pros::E_MOTOR_GEAR_600, false, pros::motor_encoder_units_e::E_MOTOR_ENCODER_COUNTS),		// 
	pros::Motor(2, pros::E_MOTOR_GEAR_600, false, pros::motor_encoder_units_e::E_MOTOR_ENCODER_COUNTS),		// 
	pros::Motor(3, pros::E_MOTOR_GEAR_600, false, pros::motor_encoder_units_e::E_MOTOR_ENCODER_COUNTS),		// 
	pros::Motor(4, pros::E_MOTOR_GEAR_600, false, pros::motor_encoder_units_e::E_MOTOR_ENCODER_COUNTS),		// 
	pros::Motor(5, pros::E_MOTOR_GEAR_600, false, pros::motor_encoder_units_e::E_MOTOR_ENCODER_COUNTS),		// 
	pros::Motor(6, pros::E_MOTOR_GEAR_600, false, pros::motor_encoder_units_e::E_MOTOR_ENCODER_COUNTS),		// 
	pros::Motor(7, pros::E_MOTOR_GEAR_600, false, pros::motor_encoder_units_e::E_MOTOR_ENCODER_COUNTS),		// 
	pros::Motor(8, pros::E_MOTOR_GEAR_600, false, pros::motor_encoder_units_e::E_MOTOR_ENCODER_COUNTS),		// 
};
pros::Motor turret_motor(8, pros::E_MOTOR_GEAR_600, false, pros::motor_encoder_units_e::E_MOTOR_ENCODER_COUNTS);	// 

pros::Rotation encoders[3] = {
	pros::Rotation(17),	//
	pros::Rotation(18),	//
	pros::Rotation(19),	//
};

pros::Controller controller_main(pros::E_CONTROLLER_MASTER);
const pros::controller_analog_e_t joy_channels[4] = {
	ANALOG_LEFT_X,
	ANALOG_LEFT_Y,
	ANALOG_RIGHT_X,
	ANALOG_RIGHT_Y
};

const pros::controller_digital_e_t joy_btns[12] = {
	DIGITAL_A,
	DIGITAL_B,
	DIGITAL_X,
	DIGITAL_Y,
	DIGITAL_UP,
	DIGITAL_DOWN,
	DIGITAL_LEFT,
	DIGITAL_RIGHT,
	DIGITAL_L1,
	DIGITAL_L2,
	DIGITAL_R1,
	DIGITAL_R2,
};

pros::ADIPort digital_out[8] = {
	pros::ADIPort(1),
	pros::ADIPort(2),
	pros::ADIPort(3),
	pros::ADIPort(4),
	pros::ADIPort(5),
	pros::ADIPort(6),
	pros::ADIPort(7),
	pros::ADIPort(8),
};

uint32_t checksum_bitmask[8] = {
	0x00000001,
	0x00000002,
	0x00000004,
	0x00000008,
	0x00000010,
	0x00000020,
	0x00000040,
	0x00000080,
};

// Serial Port
int stdin_fd_ = fileno(stdin);
int stdout_fd_ = fileno(stdout);
pros::Mutex serial_io_mutex_;

void move_voltage_slew(pros::Motor &motor, double cmd_voltage, const double max_slew_percentage){
	double motor_gearing[] = {100, 200, 600};
	
	// Normalize voltage command from millivolts
	double cmd_voltage_scaled = cmd_voltage / 12000.0;

	// Normalize velocity by nominal free speed
	double curr_vel_scaled = motor.get_actual_velocity() / motor_gearing[motor.get_gearing()];

	double cmd;
	// Maximum voltage difference in motor is capped by slew rate parameter
	if(fabs(curr_vel_scaled - cmd_voltage_scaled) > max_slew_percentage){
		double sign = (curr_vel_scaled - cmd_voltage_scaled > 0) ? 1.0 : -1.0;
		cmd = curr_vel_scaled - sign*fabs(max_slew_percentage);
	}
	else{
		cmd = cmd_voltage_scaled;
	}
	
	// Set motor
	motor.move_voltage(cmd*12000);
}

/*
	------ Producer Message Encoding ------
	PROS Overhead:
	5x Bytes - Packet Header (COBS Byte + "sout")
	1x Byte - Delimiter (\x00)
	Sum: 6x Bytes -> 0.41ms

	6x Independent Drive Motors, 1x Turret:
		4x Bytes - int32_t for Position
		4x Bytes - float for Velocity
	Sum: 56x Bytes -> 3.89ms

	Each V5 Swerve Encoder (Three on drivetrain, One for Turret):
		4x Bytes - int32_t for Angle
		4x Bytes - int32_t for Velocity
	Sum: 24x Bytes -> 1.67ms
	
	4x Joystick Channels:
		4x Bytes - int32_t for Joystick Value
	Sum: 16x Bytes -> 1.11ms

	Misc:
		1.5 Bytes - Joystick Buttons
		0.5 Bytes - Competition Mode (Enabled - Autonomous - Competition Connected - None)
		1x Byte - Digital Outs
		1x Byte - Checksum
	Sum: 4x Bytes -> 0.27ms
	
	Total Sum: 66 Byte Packet
	106 Bytes x 8 bits / byte * 1 sec / 115200 bits * 1000ms / 1s = 7.36ms

	------ Packet Format ------
	Header (6x Bytes)

	6x Drive Motor Positions 	(24x Bytes)
	1x Turret Motor Position 	(4x Bytes)
	3x Encoder Angles 			(12x Bytes)
	4x Joystick Channels 		(16x Bytes)

	6x Drive Motor Velocities 	(24x Bytes)
	1x Turret Motor Velocity 	(4x Bytes)
	3x Encoder Velocities 		(12x Bytes)

	12x Joystick Buttons 		(1.5 Bytes / 12 bits)
	Enabled 					(1x bit)
	Autonomous 					(1x bit)
	Competition Connected 		(1x bit)
	Empty Bit					(1x bit)
	Digital Outs				(1x Byte / 8 bits)
	Checksum					(1x Byte, 8 bits)
*/
void producer_main(){
	
	std::vector<int32_t> int_buffer(6+1+3+3+4, 0);
	
	std::vector<float> float_buffer(6+1, 0.0);

	uint16_t digital_states = 0;
	uint8_t digital_outs = 0;

	// Add one at end for checksum byte
	int char_buffer_len = 4*int_buffer.size() + 4*float_buffer.size() + sizeof(digital_states) + sizeof(digital_outs) + 1;
	unsigned char char_buffer[char_buffer_len] = {0,};

	while(run_){
		//// MOTORS ////
		// Poll drive motors
		for(int i = 0; i < 6; i++){
			uint32_t timestamp;
			int_buffer[i] = drive_motors[i].get_raw_position(&timestamp);
			float_buffer[i] = drive_motors[i].get_actual_velocity();
		}

		// Poll turret motor
		int_buffer[6] = turret_motor.get_position();
		float_buffer[6] = turret_motor.get_actual_velocity();

		//// SENSORS ////
		// Poll drive encoders
		for(int i = 0; i < 3; i++){
			int_buffer[i+7] = encoders[i].get_angle();
			int_buffer[i+7+3] = encoders[i].get_velocity();
		}

		//// JOYSTICK ////
		// Poll joystick channels
		for(int i = 0; i < 4; i++){
			int_buffer[i+13] = controller_main.get_analog(joy_channels[i]);
		}

		// Poll joystick button channels
		digital_states = 0;
		for(int i = 0; i < 12; i++){
			digital_states += (uint32_t) controller_main.get_digital(joy_btns[i]);
			digital_states <<= 1;
		}

		// Poll competition mode
		digital_states += pros::competition::is_disabled();
		digital_states <<= 1;

		digital_states += pros::competition::is_autonomous();
		digital_states <<= 1;

		digital_states += pros::competition::is_connected();
		digital_states <<= 1;

		// TODO: New joystick input bit (Sampled at 50ms intervals)


		// Digital Outs
		for(int i = 0; i < 7; i++){
			digital_outs += digital_out[i].get_value();
			digital_outs <<= 1;
		}
		// digital_outs += digital_out[7].get_value();

		// Copy each buffer to single buffer of unsigned char
		// Otherwise we are sending three packets with additional overhead
		memcpy(char_buffer, int_buffer.data(), 4*int_buffer.size());
		memcpy(char_buffer + 4*int_buffer.size(), float_buffer.data(), 4*float_buffer.size());
		memcpy(char_buffer + 4*int_buffer.size() + 4*float_buffer.size(), &digital_states, 2);
		memcpy(char_buffer + 4*int_buffer.size() + 4*float_buffer.size() + 2, &digital_outs, 1);

		// Calculate Checksum
		uint8_t checksum_byte = 0;
		for(int i = 0; i < char_buffer_len - 1; i++){
			checksum_byte += char_buffer[i];
		}
		
		// Append checksum byte
		char_buffer[char_buffer_len - 1] = checksum_byte;

		// Write single char buffer to serial port (as one packet)
		serial_io_mutex_.lock();
		write(stdout_fd_, char_buffer, sizeof(char_buffer));
		serial_io_mutex_.unlock();

		pros::delay(10);
	}
}

/*
	------ Producer Message Encoding ------
	PROS Overhead:
	5x Bytes - Packet Header (NULL + "sout")
	1x Byte - Delimiter (\x00)
	Sum: 6x Bytes -> 0.41ms

	6x Independent Drive Motors, 1x Turret:
		4x Bytes - 
		4x Bytes - int32_t for Velocity Command
		2x Bytes - int16_t for Voltage Command
	Sum: 42 Bytes -> 2.92ms

	2x Flywheel Motors
		4x Bytes - int32_t for Velocity Command
		2x Bytes - int16_t for Voltage Command
	Sum: 12 Bytes -> 0.83ms

	Misc:
		1x Byte - Checksum
	Sum: 1x Byte -> 0.07ms
	
	Total Sum: 61 Byte Packet
	 61 Bytes x 8 bits / byte * 1 sec / 115200 bits * 1000ms / 1s = 4.24ms
	*/
void consumer_main(){
	int32_t read_buffer_len = 128;
	unsigned char read_buffer[read_buffer_len] = {0,};
	unsigned char write_buffer[read_buffer_len] = {0,};
	while(run_){
		// Read serial buffer
		serial_io_mutex_.lock();
		int bytes_available = std::min(pros::c::fdctl(stdin_fd_, DEVCTL_FIONREAD, NULL), read_buffer_len);
		int bytes_read = read(stdin_fd_, read_buffer, bytes_available);

		write_buffer[0] = 'b';
		write_buffer[1] = 'e';
		write_buffer[2] = 'e';
		write_buffer[3] = 'p';
		write_buffer[4] = '_';
		for(int i = 0; i < bytes_read; i++){
			write_buffer[i+5] = read_buffer[i];
		}

		write_buffer[bytes_read + 5] = '_';
		write_buffer[bytes_read + 6] = 'b';
		write_buffer[bytes_read + 7] = 'e';
		write_buffer[bytes_read + 8] = 'e';
		write_buffer[bytes_read + 9] = 'p';

		write(stdout_fd_, write_buffer, 10 + bytes_read);
		serial_io_mutex_.unlock();

		
		// Collect packets
		
		// Verify Check Sum
		
		// Actuator Mutex
		
		// Update Actuators
		// std::cout << "Consumer" << std::endl;
		pros::delay(10);
	}
}

void actuator_timeout_main(){
	while(run_){
		std::cout << "Timeout" << std::endl;
		pros::delay(1000);
	}
}

void button_callback(){
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::Controller controller_main(pros::E_CONTROLLER_MASTER);

	// Start Serial Interface tasks
	// serial_init();
	// pros::Task producer_thread(producer_main, "producer thread");
	// pros::Task consumer_thread(consumer_main, "consumer thread");

	// Callback serial test
	// pros::lcd::register_btn0_cb(&button_callback);
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
	auto m1 = pros::Motor(11, pros::E_MOTOR_GEAR_600, false, pros::motor_encoder_units_e::E_MOTOR_ENCODER_COUNTS);
	auto m2 = pros::Motor(12, pros::E_MOTOR_GEAR_600, false, pros::motor_encoder_units_e::E_MOTOR_ENCODER_COUNTS);
	
	auto lpf_2 = ghost_estimation::SecondOrderLowPassFilter(100, 0.707, 0.01);

	while(run_){
		auto start = pros::millis();
		// double wheel_vel = controller_main.get_analog(ANALOG_RIGHT_Y)*500.0/127.0;	// 500 RPM
		// double module_vel = controller_main.get_analog(ANALOG_LEFT_Y)*200.0/127.0;	// 200 RPM

		// std::cout << wheel_vel << ", " << module_vel << std::endl;
		// std::cout << 0.9*(0.6*wheel_vel + 1.5*module_vel) << std::endl;
		// std::cout << 0.9*(-0.6*wheel_vel + 1.5*module_vel) << std::endl;
		// std::cout << std::endl;

		// m1.move_velocity(0.9*(0.6*wheel_vel + 1.5*module_vel));
		// m2.move_velocity(0.9*(-0.6*wheel_vel + 1.5*module_vel));

		double input1 = controller_main.get_analog(ANALOG_RIGHT_Y)/127.0;
		double input2 = controller_main.get_analog(ANALOG_LEFT_Y)/127.0;

		move_voltage_slew(m1, input1*12000, 1.0);
		move_voltage_slew(m2, input2*12000, 1.0);
		auto raw_vel = m2.get_actual_velocity();
		auto f2_vel = lpf_2.updateFilter(raw_vel);
		std::cout << m2.get_actual_velocity() << " " << f1_vel << " " << f2_vel << std::endl;

		// std::cout << m1.get_actual_velocity() << std::endl;
		pros::delay(10 - (pros::millis() - start));

	}
}