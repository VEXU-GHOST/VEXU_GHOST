
#include "ghost_v5/serial/v5_serial_node.hpp"
#include "ghost_v5/globals/v5_globals.hpp"
namespace ghost_v5{

V5SerialNode::V5SerialNode(std::string msg_start_seq, int msg_len, bool use_checksum) : msg_len_(msg_len){
    serial_base_interface_ = std::make_unique<ghost_serial::V5SerialBase>(msg_start_seq, msg_len, use_checksum);
    
    // Array to store latest incoming msg
    new_msg_ = std::vector<unsigned char>(msg_len_, 0);
}

V5SerialNode::~V5SerialNode(){

}

void V5SerialNode::initSerial(){

}

void V5SerialNode::readerLoop(){
	std::vector<unsigned char> msg_buffer(msg_len_, 0);

	while(v5_globals::run){
		if(serial_base_interface_->readMsgFromSerial(new_msg_.data())){
			updateActuatorCommands(new_msg_.data());
		}
		pros::delay(10);
	}
}

void V5SerialNode::updateActuatorCommands(unsigned char buffer[]){

}

void V5SerialNode::writeSensorUpdate(){

}


} // namespace ghost_v5