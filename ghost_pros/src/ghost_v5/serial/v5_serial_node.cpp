
#include "ghost_v5/serial/v5_serial_node.hpp"

#include "ghost_ros/robot_config/v5_serial_msg_config.hpp"
#include "ghost_v5/globals/v5_globals.hpp"
#include "ghost_v5/motor/ghost_motor.hpp"

using ghost_serial::BITMASK_ARR_32BIT;

using ghost_v5_config::v5_motor_id_enum;
using ghost_v5_config::v5_sensor_id_enum;

namespace ghost_v5
{

	V5SerialNode::V5SerialNode(std::string msg_start_seq, int msg_len, bool use_checksum) : max_msg_len_(msg_len)
	{
		// Calculate Msg Sizes based on robot configuration
		actuator_command_msg_len_ = 2 * 4 * ghost_v5_config::actuator_command_config.size() + 1;
		for (auto &pair : ghost_v5_config::actuator_command_config)
		{
			actuator_command_msg_len_ += (pair.second) ? 4 : 0; // Add four bytes for each motor using position control
		}

		state_update_msg_len_ = (ghost_v5_config::state_update_motor_config.size() + ghost_v5_config::state_update_sensor_config.size()) * 2 * 4;
		state_update_msg_len_ += ghost_v5_config::state_update_extra_byte_count + use_checksum;

		// Array to store latest incoming msg
		new_msg_ = std::vector<unsigned char>(max_msg_len_, 0);

		// Construct Serial Interface
		serial_base_interface_ = std::make_unique<ghost_serial::V5SerialBase>(msg_start_seq, msg_len, use_checksum);
	}

	V5SerialNode::~V5SerialNode()
	{
	}

	void V5SerialNode::initSerial()
	{
	}

	bool V5SerialNode::readV5ActuatorUpdate()
	{
		int msg_len;
		bool msg_recieved = serial_base_interface_->readMsgFromSerial(new_msg_.data(), msg_len);
		if (msg_recieved)
		{
			std::unique_lock<pros::Mutex> actuator_lock(v5_globals::actuator_update_lock);
			updateActuatorCommands(new_msg_.data());
			actuator_lock.unlock();
		}
		return msg_recieved;
	}

	void V5SerialNode::updateActuatorCommands(unsigned char buffer[])
	{
		// Index to count 32-bit values from buffer
		int buffer_32bit_index = 0;

		// Update each motor based on msg configuration and new values
		for (auto &motor_pair : ghost_v5_config::actuator_command_config)
		{
			// For clarity of configuration file
			auto motor_id = motor_pair.first;
			bool use_position_control = motor_pair.second;

			// Copy Voltage Command
			float voltage_command;
			memcpy(&voltage_command, buffer + 4 * (buffer_32bit_index++), 4);

			// Copy Velocity Command
			float velocity_command;
			memcpy(&velocity_command, buffer + 4 * (buffer_32bit_index++), 4);

			if (use_position_control)
			{
				// Copy Angle Command, if enabled for this motor
				int32_t angle_command;
				memcpy(&angle_command, buffer + 4 * (buffer_32bit_index++), 4);

				v5_globals::motors[motor_id]->setMotorCommand(voltage_command, velocity_command, angle_command);
			}
			else
			{
				v5_globals::motors[motor_id]->setMotorCommand(voltage_command, velocity_command);
			}
		}

		// Update Digital Outputs
		uint8_t digital_out_vector = 0;
		memcpy(&digital_out_vector, buffer + 4 * buffer_32bit_index, 1);
		for (int i = 0; i < 8; i++)
		{
			v5_globals::adi_ports[i].set_value(digital_out_vector & 0x01);
			digital_out_vector >>= 1;
		}
	}

	void V5SerialNode::writeV5StateUpdate()
	{
		int buffer_32bit_index = 0;
		unsigned char state_update_msg_buffer[state_update_msg_len_] = {
			0,
		};
		uint32_t device_connected_vector = 0;

		// Update V5 Motor Encoders
		for (auto &motor_id : ghost_v5_config::state_update_motor_config)
		{
			int position = v5_globals::motors[motor_id]->get_position();
			float velocity = v5_globals::motors[motor_id]->getVelocityFilteredRPM();

			// If device is connected (and recieving valid sensor updates), set corresponding bit in connected vector
			if (v5_globals::motors[motor_id]->getDeviceIsConnected())
			{
				device_connected_vector |= (BITMASK_ARR_32BIT[motor_id]);
			}
			else
			{
				device_connected_vector &= (~BITMASK_ARR_32BIT[motor_id]);
			}

			memcpy(state_update_msg_buffer + 4 * (buffer_32bit_index++), &position, 4);
			memcpy(state_update_msg_buffer + 4 * (buffer_32bit_index++), &velocity, 4);
		}

		// Update V5 Sensors
		for (auto &sensor_id : ghost_v5_config::state_update_sensor_config)
		{
			int32_t position = v5_globals::encoders[sensor_id]->get_angle();
			float velocity = v5_globals::encoders[sensor_id]->get_velocity();

			memcpy(state_update_msg_buffer + 4 * (buffer_32bit_index++), &position, 4);
			memcpy(state_update_msg_buffer + 4 * (buffer_32bit_index++), &velocity, 4);

			if (position != PROS_ERR && velocity != PROS_ERR_F)
			{
				device_connected_vector |= (BITMASK_ARR_32BIT[sensor_id]);
			}
			else
			{
				device_connected_vector &= (~BITMASK_ARR_32BIT[sensor_id]);
			}
		}

		// Poll joystick channels
		for (int i = 0; i < 4; i++)
		{
			int32_t analog_input = v5_globals::controller_main.get_analog(v5_globals::joy_channels[i]);
			memcpy(state_update_msg_buffer + 4 * (buffer_32bit_index++), &analog_input, 4);
		}

		// Poll joystick button channels
		uint16_t digital_states = 0;
		for (auto &btn : v5_globals::joy_btns)
		{
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

		// TODO: New joystick input bit (Sampled at 50ms intervals)

		// Digital Outs
		uint8_t digital_outs = 0;
		for (auto &adi_port : v5_globals::adi_ports)
		{
			digital_outs += adi_port.get_value();
			digital_outs <<= 1;
		}
		digital_outs += v5_globals::adi_ports[7].get_value();

		memcpy(state_update_msg_buffer + 4 * buffer_32bit_index, &digital_states, 2);
		memcpy(state_update_msg_buffer + 4 * (buffer_32bit_index) + 2, &digital_outs, 1);
		memcpy(state_update_msg_buffer + 4 * (buffer_32bit_index) + 3, &device_connected_vector, 4);

		serial_base_interface_->writeMsgToSerial(state_update_msg_buffer, state_update_msg_len_);
	}

} // namespace ghost_v5