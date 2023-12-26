#include "ghost_v5/serial/v5_serial_node.hpp"

#include "ghost_v5/globals/v5_globals.hpp"
#include "ghost_v5/motor/v5_motor_interface.hpp"

#include "pros/adi.h"
#include "pros/apix.h"

using ghost_util::BITMASK_ARR_32BIT;

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
	bool msg_recieved = serial_base_interface_->readMsgFromSerial(new_msg_.data(), actuator_command_msg_len_);
	if(msg_recieved){
		std::unique_lock<pros::Mutex> actuator_lock(v5_globals::actuator_update_lock);
		updateActuatorCommands(new_msg_.data());
		actuator_lock.unlock();
	}
	return msg_recieved;
}

void V5SerialNode::updateActuatorCommands(unsigned char buffer[]){
}

void V5SerialNode::writeV5StateUpdate(){
}

} // namespace ghost_v5