#include "ghost_v5/serial/v5_serial_node.hpp"

#include "ghost_v5/globals/v5_globals.hpp"
#include "ghost_v5/motor/v5_motor_interface.hpp"

#include "pros/adi.h"
#include "pros/apix.h"

using ghost_serial::BITMASK_ARR_32BIT;

namespace ghost_v5 {

V5SerialNode::V5SerialNode(std::string read_msg_start_seq,
                           bool use_checksum) :
	read_msg_id_{1},
	write_msg_id_{1}{
	// Calculate Msg Sizes based on robot configuration
	actuator_command_msg_len_ = ghost_v5_config::get_actuator_command_msg_len();
	sensor_update_msg_len_ = ghost_v5_config::get_sensor_update_msg_len();
	int num_motors_ = 0;

	// Array to store latest incoming msg
	new_msg_ = std::vector<unsigned char>(actuator_command_msg_len_, 0);

	// Construct Serial Interface
	serial_base_interface_ = std::make_unique<ghost_serial::V5SerialBase>(
		read_msg_start_seq,
		actuator_command_msg_len_,
		use_checksum);
}

V5SerialNode::~V5SerialNode(){
}

void V5SerialNode::initSerial(){
	pros::c::serctl(SERCTL_DISABLE_COBS, NULL);
	pros::c::serctl(SERCTL_BLKWRITE, NULL);
}

bool V5SerialNode::readV5ActuatorUpdate(){
	int msg_len;
	bool msg_recieved = serial_base_interface_->readMsgFromSerial(new_msg_.data(), actuator_command_msg_len_);
	if(msg_recieved){
		std::unique_lock<pros::Mutex> actuator_lock(v5_globals::actuator_update_lock);
		updateActuatorCommands(new_msg_.data());
		actuator_lock.unlock();
	}
	return msg_recieved;
}

void V5SerialNode::updateActuatorCommands(unsigned char buffer[]){
	// Index to count 32-bit values from buffer
	int buffer_8bit_index = 0;

	// Update each motor based on msg configuration and new values
	for(const auto &[name, config] : ghost_v5_config::motor_config_map){
		// For clarity of configuration file
		// Copy Current Limit
		int32_t current_limit;
		memcpy(&current_limit, buffer + buffer_8bit_index, 4);
		buffer_8bit_index += 4;
		v5_globals::motors[name]->getMotorInterfacePtr()->set_current_limit(current_limit);

		// Copy Position Command
		int32_t position_command;
		memcpy(&position_command, buffer + buffer_8bit_index, 4);
		buffer_8bit_index += 4;

		// Copy Velocity Command
		float velocity_command;
		memcpy(&velocity_command, buffer + buffer_8bit_index, 4);
		buffer_8bit_index += 4;

		// Copy Voltage Command
		float voltage_command;
		memcpy(&voltage_command, buffer + buffer_8bit_index, 4);
		buffer_8bit_index += 4;

		// Copy Velocity Command
		float torque_command;
		memcpy(&torque_command, buffer + buffer_8bit_index, 4);
		buffer_8bit_index += 4;

		uint8_t actuator_flags_byte;
		memcpy(&actuator_flags_byte, buffer + buffer_8bit_index, 1);
		buffer_8bit_index++;

		bool position_control = (bool)(actuator_flags_byte & 0x01);
		actuator_flags_byte >>= 1;
		bool velocity_control = (bool)(actuator_flags_byte & 0x01);
		actuator_flags_byte >>= 1;
		bool voltage_control = (bool)(actuator_flags_byte & 0x01);
		actuator_flags_byte >>= 1;
		bool torque_control = (bool)(actuator_flags_byte & 0x01);

		v5_globals::motors[name]->setControlMode(position_control, velocity_control, voltage_control, torque_control);
		v5_globals::motors[name]->setMotorCommand(position_command, velocity_command, voltage_command, torque_command);
	}

	// Update incoming msg id
	uint32_t msg_id;
	memcpy(&msg_id, buffer + buffer_8bit_index, 4);
	buffer_8bit_index += 4;

	// Update Digital Outputs
	uint8_t digital_out_vector = 0;
	memcpy(&digital_out_vector, buffer + buffer_8bit_index, 1);
	buffer_8bit_index += 4;
	for(int i = 7; i >= 0; i--){
		v5_globals::digital_out_cmds[i] = (bool)(digital_out_vector & 0x01);
		digital_out_vector >>= 1;
	}
}

void V5SerialNode::writeV5StateUpdate(){
	int buffer_32bit_index = 0;
	unsigned char sensor_update_msg_buffer[sensor_update_msg_len_] = {
		0,
	};
	uint32_t device_connected_vector = 0;

	// Update V5 Motor Encoders
	for(const auto &[name, config] : ghost_v5_config::motor_config_map){
		float position = v5_globals::motors[name]->getMotorInterfacePtr()->get_position();
		float velocity = v5_globals::motors[name]->getVelocityFilteredRPM();
		float voltage = v5_globals::motors[name]->getVoltageCommand();
		float current = v5_globals::motors[name]->getMotorInterfacePtr()->get_current_draw();
		float temp = v5_globals::motors[name]->getMotorInterfacePtr()->get_temperature();
		float power = v5_globals::motors[name]->getMotorInterfacePtr()->get_power();

		// If device is connected (and recieving valid sensor updates), set corresponding bit in connected vector
		if(v5_globals::motors[name]->getDeviceIsConnected()){
			device_connected_vector |= (BITMASK_ARR_32BIT[config.port]);
		}
		else{
			device_connected_vector &= (~BITMASK_ARR_32BIT[config.port]);
		}

		memcpy(sensor_update_msg_buffer + 4 * (buffer_32bit_index++), &position, 4);
		memcpy(sensor_update_msg_buffer + 4 * (buffer_32bit_index++), &velocity, 4);
		memcpy(sensor_update_msg_buffer + 4 * (buffer_32bit_index++), &voltage, 4);
		memcpy(sensor_update_msg_buffer + 4 * (buffer_32bit_index++), &current, 4);
		memcpy(sensor_update_msg_buffer + 4 * (buffer_32bit_index++), &temp, 4);
		memcpy(sensor_update_msg_buffer + 4 * (buffer_32bit_index++), &power, 4);
	}

	// Update V5 Sensors
	for(const auto &[name, config] : ghost_v5_config::encoder_config_map){
		float position = ((float)v5_globals::encoders[name]->get_position()) / 100.0;
		float velocity = ((float)v5_globals::encoders[name]->get_velocity()) * 60.0 / 100.0 / 360.0; // Centidegrees -> RPM

		memcpy(sensor_update_msg_buffer + 4 * (buffer_32bit_index++), &position, 4);
		memcpy(sensor_update_msg_buffer + 4 * (buffer_32bit_index++), &velocity, 4);

		if((position != PROS_ERR) && (velocity != PROS_ERR_F) ){
			device_connected_vector |= (BITMASK_ARR_32BIT[config.port]);
		}
		else{
			device_connected_vector &= (~BITMASK_ARR_32BIT[config.port]);
		}
	}

	// Poll joystick channels
	for(int i = 0; i < 4; i++){
		float analog_input = ((float)v5_globals::controller_main.get_analog(v5_globals::joy_channels[i])) / 127.0;
		memcpy(sensor_update_msg_buffer + 4 * (buffer_32bit_index++), &analog_input, 4);
	}

	// Poll joystick button channels
	uint16_t digital_states = 0;
	for(auto &btn : v5_globals::joy_btns){
		digital_states += (uint16_t)v5_globals::controller_main.get_digital(btn);
		digital_states <<= 1;
	}

	// Poll competition mode
	digital_states += pros::competition::is_disabled();
	digital_states <<= 1;
	digital_states += pros::competition::is_autonomous();
	digital_states <<= 1;
	digital_states += pros::competition::is_connected();
	digital_states <<= 1;

	memcpy(sensor_update_msg_buffer + 4 * buffer_32bit_index, &digital_states, 2);
	memcpy(sensor_update_msg_buffer + 4 * (buffer_32bit_index) + 2, &device_connected_vector, 4);
	memcpy(sensor_update_msg_buffer + 4 * (buffer_32bit_index) + 2 + 4, &write_msg_id_, 4);

	serial_base_interface_->writeMsgToSerial(sensor_update_msg_buffer, sensor_update_msg_len_);
	write_msg_id_++;
}

} // namespace ghost_v5