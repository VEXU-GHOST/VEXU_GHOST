
#include "ghost_serial/v5_serial_node.hpp"

using std::placeholders::_1;
using namespace std::literals::chrono_literals;
namespace ghost_serial{

V5SerialNode::V5SerialNode(std::string config_file) : Node("ghost_serial_node"), reader_thread_init_(false){
    // Load config file
    config_yaml_ = YAML::LoadFile(config_file);
    msg_len_ = config_yaml_["msg_len"].as<int>(); 
    verbose_ = config_yaml_["verbose"].as<bool>();
    using_reader_thread_ = config_yaml_["using_reader_thread"].as<bool>(); 

    // Serial Interface
    serial_interface_ = std::make_shared<SerialInterface>(
        config_yaml_["port_name"].as<std::string>(),
        config_yaml_["msg_start_seq"].as<std::string>(),
        msg_len_,
        config_yaml_["use_checksum"].as<bool>(),
        verbose_);

    // Sensor Update Msg Publisher
    sensor_update_pub_ = create_publisher<ghost_msgs::msg::SensorUpdate>("v5_sensor_update", 10);

    // Actuator Command Msg Subscriber
    actuator_command_sub_ = create_subscription<ghost_msgs::msg::ActuatorCommands>(
        "v5_actuator_commands",
        10,
        std::bind(&V5SerialNode::actuatorCommandCallback, this, _1)
        );

    // Array to store latest incoming msg
    new_msg_ = std::vector<unsigned char>(msg_len_, 0);
}

V5SerialNode::~V5SerialNode(){
    if(reader_thread_init_){
        reader_thread_.join();
    }
}

void V5SerialNode::initSerialInterfaceBlocking(){
    // Wait for serial to become available
    bool serial_open = false;
    while(rclcpp::ok() && !serial_open){
        serial_open = serial_interface_->trySerialOpen();
        std::this_thread::sleep_for(10ms);
    }

    // Start Reader Thread
    if(rclcpp::ok() && using_reader_thread_){
        reader_thread_ = std::thread(&V5SerialNode::readerLoop, this);
    }
}

void V5SerialNode::readerLoop(){
    if(verbose_){
        std::cout << "[START] Serial Read Thread" << std::endl;
    }

    reader_thread_init_ = true;
    while(rclcpp::ok()){
        try{
            bool msg_found = serial_interface_->readMsgFromSerial(new_msg_.data());

            if(msg_found){
                publishSensorUpdate(new_msg_.data());
            }
        }
        catch(std::exception &e){
            std::cout << "Error: " << e.what() << std::endl;
        }
    }

    if(verbose_){
        std::cout << "[END] Serial Read Thread" << std::endl;
    }
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
void V5SerialNode::actuatorCommandCallback(const ghost_msgs::msg::ActuatorCommands::SharedPtr msg){
    if(verbose_){
        RCLCPP_INFO(get_logger(), "Actuator Command");
    }

    std::vector<int32_t> angle_buffer{
        msg->wheel_left_angle_cmd,
        msg->wheel_right_angle_cmd,
        msg->wheel_back_angle_cmd,
        msg->steering_left_angle_cmd,
        msg->steering_right_angle_cmd,
        msg->steering_back_angle_cmd,
        msg->turret_angle_cmd
    };

    std::vector<float> vel_buffer{
        msg->wheel_left_velocity_cmd,
        msg->wheel_right_velocity_cmd,
        msg->wheel_back_velocity_cmd,
        msg->steering_left_velocity_cmd,
        msg->steering_right_velocity_cmd,
        msg->steering_back_velocity_cmd,
        msg->turret_velocity_cmd,
        msg->flywheel_left_velocity_cmd,
        msg->flywheel_right_velocity_cmd,
    };

    std::vector<float> voltage_buffer{
        msg->wheel_left_voltage_cmd,
        msg->wheel_right_voltage_cmd,
        msg->wheel_back_voltage_cmd,
        msg->steering_left_voltage_cmd,
        msg->steering_right_voltage_cmd,
        msg->steering_back_voltage_cmd,
        msg->turret_voltage_cmd,
        msg->flywheel_right_voltage_cmd,
        msg->flywheel_left_voltage_cmd,
        msg->intake_voltage_cmd,
        msg->indexer_voltage_cmd,
    };

    // Digital Outputs
    uint8_t digital_out_vector = 0;
    digital_out_vector += msg->digital_out_1;
    digital_out_vector <<= 1;
    digital_out_vector += msg->digital_out_2;
    digital_out_vector <<= 1;
    digital_out_vector += msg->digital_out_3;
    digital_out_vector <<= 1;
    digital_out_vector += msg->digital_out_4;
    digital_out_vector <<= 1;
    digital_out_vector += msg->digital_out_5;
    digital_out_vector <<= 1;
    digital_out_vector += msg->digital_out_6;
    digital_out_vector <<= 1;
    digital_out_vector += msg->digital_out_7;
    digital_out_vector <<= 1;
    digital_out_vector += msg->digital_out_8;
    
    // Pack into single msg
    unsigned char msg_buffer[4*angle_buffer.size() + 4*vel_buffer.size() + 4*voltage_buffer.size() + 1] = {0,};
    memcpy(msg_buffer, angle_buffer.data(), 4*angle_buffer.size());
    memcpy(msg_buffer + 4*angle_buffer.size(), vel_buffer.data(), 4*vel_buffer.size());
    memcpy(msg_buffer + 4*angle_buffer.size() + 4*vel_buffer.size(), voltage_buffer.data(), 4*voltage_buffer.size());
    memcpy(msg_buffer + 4*angle_buffer.size() + 4*vel_buffer.size() + 4*voltage_buffer.size(), &digital_out_vector, 1);

    serial_interface_->writeMsgToSerial(msg_buffer, 109);
}

/*
	------ Packet Format ------
	H99x Bytes

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
	*/
void V5SerialNode::publishSensorUpdate(unsigned char buffer[]){
    if(verbose_){
        RCLCPP_INFO(get_logger(), "Sensor Update");
    }
    
    auto msg = ghost_msgs::msg::SensorUpdate{};
    
    msg.header.stamp = get_clock()->now() - rclcpp::Duration(7.36ms);

    // Buffers to store extracted V5 Msg
	uint8_t int_buffer_len = 6+1+3+3+4;
	int32_t int_buffer[int_buffer_len] = {0,};
	
	uint8_t float_buffer_len = 6+1;
	float float_buffer[float_buffer_len] = {0.0f,};

    uint16_t digital_states = 0;
	uint8_t digital_outs = 0;

    // Copy msg to buffers
	memcpy(int_buffer, buffer, 4*int_buffer_len);
	memcpy(float_buffer, buffer + 4*int_buffer_len, 4*float_buffer_len);
	memcpy(&digital_states, buffer + 4*int_buffer_len + 4*float_buffer_len, 2);
	memcpy(&digital_outs, buffer + 4*int_buffer_len + 4*float_buffer_len + 2, 1);

    // Motor Angles
    msg.drive_lf_angle = int_buffer[0];
    msg.drive_lb_angle = int_buffer[1];
    msg.drive_rf_angle = int_buffer[2];
    msg.drive_rb_angle = int_buffer[3];
    msg.drive_bl_angle = int_buffer[4];
    msg.drive_br_angle = int_buffer[5];
    msg.turret_angle = int_buffer[6];

    // Motor Velocities
    msg.drive_lf_velocity = float_buffer[0];
    msg.drive_lb_velocity = float_buffer[1];
    msg.drive_rf_velocity = float_buffer[2];
    msg.drive_rb_velocity = float_buffer[3];
    msg.drive_bl_velocity = float_buffer[4];
    msg.drive_br_velocity = float_buffer[5];
    msg.turret_velocity = float_buffer[6];

    // Drivetrain Steering Encoders
    msg.steering_left_angle = int_buffer[7];
    msg.steering_right_angle = int_buffer[8];
    msg.steering_back_angle = int_buffer[9];
    
    msg.steering_left_velocity = int_buffer[10];
    msg.steering_right_velocity = int_buffer[11];
    msg.steering_back_velocity = int_buffer[12];

    // Joystick Channels
    msg.joystick_left_x = int_buffer[13];
    msg.joystick_left_y = int_buffer[14];
    msg.joystick_right_x = int_buffer[15];
    msg.joystick_right_y = int_buffer[16];

    // Joystick Buttons
    msg.joystick_btn_a =        digital_states & 0x8000;
    msg.joystick_btn_b =        digital_states & 0x4000;
    msg.joystick_btn_x =        digital_states & 0x2000;
    msg.joystick_btn_y =        digital_states & 0x1000;
    msg.joystick_btn_up =       digital_states & 0x0800;
    msg.joystick_btn_down =     digital_states & 0x0400;
    msg.joystick_btn_left =     digital_states & 0x0200;
    msg.joystick_btn_right =    digital_states & 0x0100;
    msg.joystick_btn_l1 =       digital_states & 0x0080;
    msg.joystick_btn_l2 =       digital_states & 0x0040;
    msg.joystick_btn_r1 =       digital_states & 0x0020;
    msg.joystick_btn_r2 =       digital_states & 0x0010;

    // Competition state
    msg.is_disabled =           digital_states & 0x0008;
    msg.is_autonomous =         digital_states & 0x0004;
    msg.is_connected =          digital_states & 0x0002;

    // Digital Outputs
    msg.digital_out_1 = digital_outs & 0x80;
    msg.digital_out_2 = digital_outs & 0x40;
    msg.digital_out_3 = digital_outs & 0x20;
    msg.digital_out_4 = digital_outs & 0x10;
    msg.digital_out_5 = digital_outs & 0x08;
    msg.digital_out_6 = digital_outs & 0x04;
    msg.digital_out_7 = digital_outs & 0x02;
    msg.digital_out_8 = digital_outs & 0x01;

    sensor_update_pub_->publish(msg);
}

} // namespace ghost_serial