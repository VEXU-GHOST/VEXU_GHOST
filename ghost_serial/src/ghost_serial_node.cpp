
#include "ghost_serial_node.hpp"

using std::placeholders::_1;
using namespace std::literals::chrono_literals;
namespace ghost_serial{

GhostSerialNode::GhostSerialNode(std::string config_file) : Node("ghost_serial_node"){
    config_yaml_ = YAML::LoadFile(config_file);
    msg_len_ = config_yaml_["msg_len"].as<int>(); 

    std::vector<unsigned char> new_msg(msg_len_);

    serial_interface_ = std::make_unique<SerialInterface>(
        config_yaml_["port_name"].as<std::string>(),
        config_yaml_["msg_start_seq"].as<std::string>(),
        msg_len_);

    // Sensor Update Msg Publisher
    sensor_update_pub_ = create_publisher<ghost_msgs::msg::SensorUpdate>("v5_sensor_update", 10);

    // Actuator Command Msg Subscriber
    actuator_command_sub_ = create_subscription<ghost_msgs::msg::ActuatorCommands>(
        "v5_actuator_commands",
        10,
        std::bind(&GhostSerialNode::actuatorCommandCallback, this, _1)
        );
}

GhostSerialNode::~GhostSerialNode(){
    // reader_thread_.join();
}

void GhostSerialNode::initSerial(){
    bool serial_open = false;
    while(rclcpp::ok() && !serial_open){
        serial_open = serial_interface_->trySerialOpen();
        std::this_thread::sleep_for(10ms);
    }
}

void GhostSerialNode::startReaderThread(){
    reader_thread_ = std::thread(&GhostSerialNode::readerLoop, this);
}

void GhostSerialNode::readerLoop(){
    while(rclcpp::ok()){
        bool msg_found = serial_interface_->readMsgFromSerial(new_msg_.data());
        if(msg_found){
            publishSensorUpdate();
        }
    }
}

void GhostSerialNode::actuatorCommandCallback(const ghost_msgs::msg::ActuatorCommands::SharedPtr msg){
    //

}

void GhostSerialNode::publishSensorUpdate(){

}

} // namespace ghost_serial