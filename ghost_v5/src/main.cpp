#include "main.h"
#include "pros/motors.h"

#include <vector>
#include <algorithm>
#include <numeric>

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
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
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

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
void autonomous() {}

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
	pros::Motor left_mtr(1, pros::motor_gearset_e::E_MOTOR_GEAR_600);
	pros::Motor right_mtr(2, pros::motor_gearset_e::E_MOTOR_GEAR_600);
	

	int v_buf_count = 0;
	int e_buf_count = 0;
	int p_buf_count = 0;
	int num_samples = 100;

	std::vector<double> vel_buffer(num_samples, 0);
	std::vector<double> eff_buffer(num_samples, 0);
	std::vector<double> pwr_buffer(num_samples, 0);

	while (true) {
		vel_buffer[v_buf_count%num_samples] = left_mtr.get_actual_velocity();
		double v_min = *std::min_element(vel_buffer.begin(), vel_buffer.end());
		double v_max = *std::max_element(vel_buffer.begin(), vel_buffer.end());
		double v_avg = std::accumulate(vel_buffer.begin(), vel_buffer.end(), 0)/ ((double) num_samples);
		v_buf_count++;

		pwr_buffer[p_buf_count%num_samples] = left_mtr.get_power();
		double p_min = *std::min_element(pwr_buffer.begin(), pwr_buffer.end());
		double p_max = *std::max_element(pwr_buffer.begin(), pwr_buffer.end());
		double p_avg = std::accumulate(pwr_buffer.begin(), pwr_buffer.end(), 0)/((double) num_samples);
		p_buf_count++;

		if(v_buf_count % num_samples == 0){
			pros::lcd::print(0, "RPM, Efficiency, W");
			pros::lcd::print(1, "%lf %lf %lf", v_min, v_avg, v_max);
			pros::lcd::print(2, "%lf %lf %lf", p_min, p_avg, p_max);
		}
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);

		left_mtr = left;
		right_mtr = right;
		pros::delay(5);
	}
}
