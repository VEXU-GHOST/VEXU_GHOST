#pragma once
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>
namespace ghost_v5_config {

enum ghost_gearset {
	GEARSET_100,
	GEARSET_200,
	GEARSET_600
};

enum ghost_brake_mode {
	BRAKE_MODE_COAST,
	BRAKE_MODE_BRAKE,
	BRAKE_MODE_HOLD,
	BRAKE_MODE_INVALID
};

enum ghost_encoder_unit {
	ENCODER_DEGREES,
	ENCODER_ROTATIONS,
	ENCODER_COUNTS,
	ENCODER_INVALID
};

struct MotorControllerConfig {
	// FF-PD Controller
	// pos_gain and vel_gain are standard PD control
	// ff_vel_gain takes a velocity setpoint and scales it to estimate the required open-loop voltage
	// Ideally this is 1.0. If your motor runs faster than nominal, increase this a bit.
	// If you zero the other gains, you can tune this by sending a velocity and tweaking until true velocity matches.
	// ff_torque_gain does the same as above but for torque. Controlling torque with voltage is not very accurate, fyi.
	float pos_gain{0.0};
	float vel_gain{10.0};
	float ff_vel_gain{1.0};
	float ff_torque_gain{0.0};

	bool operator==(const MotorControllerConfig& rhs) const {
		return (pos_gain == rhs.pos_gain) && (vel_gain == rhs.vel_gain) &&
		       (ff_vel_gain == rhs.ff_vel_gain) && (ff_torque_gain == rhs.ff_torque_gain);
	}
};

struct LowPassFilterConfig {
	// 2nd Order Low Pass Filter for Motor Velocity
	// If you aren't familiar with LPFs, default tuning is probably fine for any VEX system.
	// If you want it smoother, lower the cutoff frequency.
	// "Ideal setting" is 10 / ((time for system to go from zero to full speed given max voltage) * 0.632)
	float filter__cutoff_frequency{100.0};
	float filter__damping_ratio{0.707}; // Don't Change
	float filter__timestep{0.01};       // Don't Change

	bool operator==(const LowPassFilterConfig& rhs) const {
		return (filter__cutoff_frequency == rhs.filter__cutoff_frequency) &&
		       (filter__damping_ratio == rhs.filter__damping_ratio) && (filter__timestep == rhs.filter__timestep);
	}
};

class MotorModelConfig {
public:
	// This configures an 11W V5 Motor.
	// NOTE: The gear ratio is set by the ghost_gearset param above and scaling is applied within the motor class.
	// This means, YOU PROBABLY SHOULDN'T TOUCH ANY OF THESE.
	float nominal_free_speed{120};
	float stall_torque{3.6};
	float free_current{0.14};
	float stall_current{4.25};
	float max_voltage{12};

	bool operator==(const MotorModelConfig& rhs) const {
		return (nominal_free_speed == rhs.nominal_free_speed) && (stall_torque == rhs.stall_torque) &&
		       (free_current == rhs.free_current) && (stall_current == rhs.stall_current) && (max_voltage == rhs.max_voltage);
	}
};

struct V5MotorInterfaceConfig {
	// These three map 1:1 to their PROS counterpart on the V5 Side.
	ghost_encoder_unit encoder_units{ghost_encoder_unit::ENCODER_DEGREES};
	ghost_gearset gear_ratio{ghost_gearset::GEARSET_600};
	ghost_brake_mode brake_mode{ghost_brake_mode::BRAKE_MODE_COAST};

	// Configures control and estimation modules for a given motor
	MotorControllerConfig controller;
	LowPassFilterConfig filter;
	MotorModelConfig model;

	bool operator==(const V5MotorInterfaceConfig& rhs) const {
		return (encoder_units == rhs.encoder_units) && (gear_ratio == rhs.gear_ratio) && (brake_mode == rhs.brake_mode) &&
		       (controller == rhs.controller) && (filter == rhs.filter) && (model == rhs.model);
	}
};

// Helper classes for accessing motor/encoders from their config maps
struct motor_access_helper {
	motor_access_helper(int port_init, bool reversed_init, V5MotorInterfaceConfig config_init){
		port = port_init;
		reversed = reversed_init;
		config = config_init;
	}

	bool operator==(const motor_access_helper& rhs) const {
		return (port == rhs.port) && (reversed == rhs.reversed) && (config == rhs.config);
	}

	int port;
	bool reversed;
	V5MotorInterfaceConfig config;
};
struct encoder_access_helper {
	encoder_access_helper(int port_init, bool reversed_init){
		port = port_init;
		reversed = reversed_init;
	}

	bool operator==(const encoder_access_helper& rhs) const {
		return (port == rhs.port) && (reversed == rhs.reversed);
	}

	int port;
	bool reversed;
};
extern const std::unordered_map<std::string, motor_access_helper> motor_config_map;
extern const std::unordered_map<std::string, encoder_access_helper> encoder_config_map;

// Serial Msg Config
extern const int actuator_cmd_extra_byte_count;
extern const int actuator_update_packet_byte_size;
extern const int motor_sensor_packet_byte_size;
extern const int encoder_sensor_packet_byte_size;
extern const int sensor_update_extra_byte_count;

}