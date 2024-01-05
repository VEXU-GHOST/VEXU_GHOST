#define PROS_USE_SIMPLE_NAMES

#include "ghost_v5/serial/v5_serial_node.hpp"

#include "ghost_v5/globals/v5_globals.hpp"
#include "ghost_v5/motor/v5_motor_interface.hpp"

#include "ghost_v5_interfaces/devices/joystick_device_interface.hpp"
#include "ghost_v5_interfaces/devices/motor_device_interface.hpp"
#include "ghost_v5_interfaces/devices/rotation_sensor_device_interface.hpp"

#include "pros/adi.h"
#include "pros/apix.h"
#include "pros/misc.h"


using ghost_util::BITMASK_ARR_32BIT;
using namespace ghost_v5_interfaces::devices;
using namespace ghost_v5_interfaces;

namespace ghost_v5 {

V5SerialNode::V5SerialNode(std::shared_ptr<RobotHardwareInterface> robot_hardware_interface_ptr){
	// Set Hardware Interface
	hardware_interface_ptr_ = robot_hardware_interface_ptr;

	// Calculate Msg Sizes based on robot configuration
	actuator_command_msg_len_ = hardware_interface_ptr_->getActuatorCommandMsgLength();
	sensor_update_msg_len_ = hardware_interface_ptr_->getSensorUpdateMsgLength();

	// Array to store latest incoming msg
	new_msg_ = std::vector<unsigned char>(actuator_command_msg_len_, 0);

	// Construct Serial Interface
	serial_base_interface_ = std::make_unique<ghost_serial::V5SerialBase>(
		"msg",
		actuator_command_msg_len_,
		true);
}

V5SerialNode::~V5SerialNode(){
}

void V5SerialNode::initSerial(){
	pros::c::serctl(SERCTL_DISABLE_COBS, NULL);
	pros::c::serctl(SERCTL_BLKWRITE, NULL);
}

bool V5SerialNode::readV5ActuatorUpdate(){
	int msg_len;
	bool msg_recieved = serial_base_interface_->readMsgFromSerial(new_msg_, actuator_command_msg_len_);
	if(msg_recieved){
		std::unique_lock<pros::Mutex> actuator_lock(v5_globals::actuator_update_lock);
		updateActuatorCommands(new_msg_);
		actuator_lock.unlock();
	}
	return msg_recieved;
}

void V5SerialNode::updateActuatorCommands(std::vector<unsigned char>& buffer){
	hardware_interface_ptr_->deserialize(buffer);

	v5_globals::digital_out_cmds = hardware_interface_ptr_->getDigitalIO();

	for(const auto& name : *hardware_interface_ptr_){
		auto device_data_ptr = hardware_interface_ptr_->getDeviceData<DeviceData>(name);

		switch(device_data_ptr->type){
			case device_type_e::MOTOR:
			{
				auto motor_device_data_ptr = device_data_ptr->as<MotorDeviceData>();
				v5_globals::motor_interfaces.at(name)->setCurrentLimit(motor_device_data_ptr->current_limit);

				v5_globals::motor_interfaces.at(name)->setMotorCommand(
					motor_device_data_ptr->position_command,
					motor_device_data_ptr->velocity_command,
					motor_device_data_ptr->voltage_command,
					motor_device_data_ptr->torque_command);

				v5_globals::motor_interfaces.at(name)->setControlMode(
					motor_device_data_ptr->position_control,
					motor_device_data_ptr->velocity_control,
					motor_device_data_ptr->voltage_control,
					motor_device_data_ptr->torque_control);
			}
			break;

			case device_type_e::INVALID:
			{
				throw std::runtime_error("ERROR: Attempted to initialize unsupported device using ghost_v5_interfaces.");
			}
			break;

			default:
			{
				throw std::runtime_error("ERROR: Attempted to initialize unsupported device using ghost_v5_interfaces.");
			}
			break;
		}
	}
}

void V5SerialNode::writeV5StateUpdate(){
	// Competition States
	hardware_interface_ptr_->setDisabledStatus(pros::competition::is_disabled());
	hardware_interface_ptr_->setAutonomousStatus(pros::competition::is_autonomous());
	hardware_interface_ptr_->setConnectedStatus(pros::competition::is_connected());

	// Joysticks
	auto joy_1_data = std::make_shared<JoystickDeviceData>(MAIN_JOYSTICK_NAME);
	joy_1_data->left_x = v5_globals::controller_main.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
	joy_1_data->left_y = v5_globals::controller_main.get_analog(ANALOG_LEFT_Y);
	joy_1_data->right_x = v5_globals::controller_main.get_analog(ANALOG_RIGHT_X);
	joy_1_data->right_y = v5_globals::controller_main.get_analog(ANALOG_RIGHT_Y);
	joy_1_data->btn_a = v5_globals::controller_main.get_digital(DIGITAL_A);
	joy_1_data->btn_b = v5_globals::controller_main.get_digital(DIGITAL_B);
	joy_1_data->btn_x = v5_globals::controller_main.get_digital(DIGITAL_X);
	joy_1_data->btn_y = v5_globals::controller_main.get_digital(DIGITAL_Y);
	joy_1_data->btn_u = v5_globals::controller_main.get_digital(DIGITAL_UP);
	joy_1_data->btn_d = v5_globals::controller_main.get_digital(DIGITAL_DOWN);
	joy_1_data->btn_l = v5_globals::controller_main.get_digital(DIGITAL_LEFT);
	joy_1_data->btn_r = v5_globals::controller_main.get_digital(DIGITAL_RIGHT);
	joy_1_data->btn_l1 = v5_globals::controller_main.get_digital(DIGITAL_L1);
	joy_1_data->btn_l2 = v5_globals::controller_main.get_digital(DIGITAL_L2);
	joy_1_data->btn_r1 = v5_globals::controller_main.get_digital(DIGITAL_R1);
	joy_1_data->btn_r2 = v5_globals::controller_main.get_digital(DIGITAL_R2);
	hardware_interface_ptr_->setDeviceData(joy_1_data);


	if(hardware_interface_ptr_->contains(PARTNER_JOYSTICK_NAME)){
		auto joy_2_data = std::make_shared<JoystickDeviceData>(PARTNER_JOYSTICK_NAME);
		joy_2_data->left_x = v5_globals::controller_partner.get_analog(ANALOG_LEFT_X);
		joy_2_data->left_y = v5_globals::controller_partner.get_analog(ANALOG_LEFT_Y);
		joy_2_data->right_x = v5_globals::controller_partner.get_analog(ANALOG_RIGHT_X);
		joy_2_data->right_y = v5_globals::controller_partner.get_analog(ANALOG_RIGHT_Y);
		joy_2_data->btn_a = v5_globals::controller_partner.get_digital(DIGITAL_A);
		joy_2_data->btn_b = v5_globals::controller_partner.get_digital(DIGITAL_B);
		joy_2_data->btn_x = v5_globals::controller_partner.get_digital(DIGITAL_X);
		joy_2_data->btn_y = v5_globals::controller_partner.get_digital(DIGITAL_Y);
		joy_2_data->btn_u = v5_globals::controller_partner.get_digital(DIGITAL_UP);
		joy_2_data->btn_d = v5_globals::controller_partner.get_digital(DIGITAL_DOWN);
		joy_2_data->btn_l = v5_globals::controller_partner.get_digital(DIGITAL_LEFT);
		joy_2_data->btn_r = v5_globals::controller_partner.get_digital(DIGITAL_RIGHT);
		joy_2_data->btn_l1 = v5_globals::controller_partner.get_digital(DIGITAL_L1);
		joy_2_data->btn_l2 = v5_globals::controller_partner.get_digital(DIGITAL_L2);
		joy_2_data->btn_r1 = v5_globals::controller_partner.get_digital(DIGITAL_R1);
		joy_2_data->btn_r2 = v5_globals::controller_partner.get_digital(DIGITAL_R2);
		hardware_interface_ptr_->setDeviceData(joy_2_data);
	}

	// Motors
	for(const auto& [name, device] : v5_globals::motor_interfaces){
		auto motor_device_data_ptr = hardware_interface_ptr_->getDeviceData<MotorDeviceData>(name);
		auto motor_interface_ptr = device->getMotorInterfacePtr();
		motor_device_data_ptr->curr_position = motor_interface_ptr->get_position();
		motor_device_data_ptr->curr_velocity_rpm = device->getVelocityFilteredRPM();
		motor_device_data_ptr->curr_torque_nm = motor_interface_ptr->get_torque();
		motor_device_data_ptr->curr_voltage_mv = motor_interface_ptr->get_voltage();
		motor_device_data_ptr->curr_current_ma = motor_interface_ptr->get_current_draw();
		motor_device_data_ptr->curr_power_w = motor_interface_ptr->get_power();
		motor_device_data_ptr->curr_temp_c = motor_interface_ptr->get_temperature();
		hardware_interface_ptr_->setDeviceData(motor_device_data_ptr);
	}

	// Encoders
	for(const auto& [name, device] : v5_globals::encoders){
		auto rotation_sensor_data_ptr = hardware_interface_ptr_->getDeviceData<RotationSensorDeviceData>(name);
		rotation_sensor_data_ptr->angle = ((float) device->get_angle()) / 100.0;
		rotation_sensor_data_ptr->position = ((float) device->get_position()) / 100.0;
		rotation_sensor_data_ptr->velocity = ((float) device->get_velocity()) / 100.0;
		hardware_interface_ptr_->setDeviceData(rotation_sensor_data_ptr);
	}

	serial_base_interface_->writeMsgToSerial(hardware_interface_ptr_->serialize().data(), sensor_update_msg_len_);
}

} // namespace ghost_v5