#include "ghost_ros/ros_nodes/jetson_v5_serial_node.hpp"
#include "ghost_ros/robot_config/v5_serial_msg_config.hpp"
#include "ghost_serial/serial_utils/bitmasks.hpp"

using std::placeholders::_1;
using namespace std::literals::chrono_literals;
namespace ghost_ros
{

    JetsonV5SerialNode::JetsonV5SerialNode() : Node("ghost_serial_node"), reader_thread_init_(false)
    {
        // Load ROS Params
        declare_parameter("using_reader_thread", true);
        using_reader_thread_ = get_parameter("using_reader_thread").as_bool();
        
        declare_parameter("use_checksum", true);
        bool use_checksum = get_parameter("use_checksum").as_bool();
        
        declare_parameter("verbose", false);
        verbose_ = get_parameter("verbose").as_bool();
        
        declare_parameter("read_msg_start_seq", "sout");
        std::string read_msg_start_seq = get_parameter("read_msg_start_seq").as_string();
        
        declare_parameter("write_msg_start_seq", "msg");
        std::string write_msg_start_seq = get_parameter("write_msg_start_seq").as_string();
        
        declare_parameter("port_name", "/dev/ttyACM1");
        std::string port_name = get_parameter("port_name").as_string();

        declare_parameter("backup_port_name", "/dev/ttyACM2");
        std::string backup_port_name = get_parameter("backup_port_name").as_string();

        // Calculate Msg Sizes based on robot configuration
        actuator_command_msg_len_ = 2 * 4 * ghost_v5_config::actuator_command_config.size() + 1;
        for (auto &pair : ghost_v5_config::actuator_command_config)
        {
            actuator_command_msg_len_ += (pair.second) ? 4 : 0; // Add four bytes for each motor using position control
        }
        actuator_command_msg_len_ += ghost_v5_config::actuator_cmd_extra_byte_count;

        sensor_update_msg_len_ = (ghost_v5_config::sensor_update_motor_config.size() * 4 + ghost_v5_config::sensor_update_sensor_config.size() * 2 ) * 4;
        sensor_update_msg_len_ += ghost_v5_config::sensor_update_extra_byte_count;

        int incoming_packet_len = sensor_update_msg_len_ +
                                  use_checksum +
                                  read_msg_start_seq.size() + 
                                  2; // Cobs Encoding adds two bytes

        RCLCPP_INFO(get_logger(), "Actuator Command Msg Length: %d", actuator_command_msg_len_);
        RCLCPP_INFO(get_logger(), "State Update Msg Length: %d", sensor_update_msg_len_);
        RCLCPP_INFO(get_logger(), "Incoming Packet Length: %d", incoming_packet_len);

        // Array to store latest incoming msg
        sensor_update_msg_ = std::vector<unsigned char>(sensor_update_msg_len_, 0);

        // Serial Interface
        serial_base_interface_ = std::make_shared<ghost_serial::JetsonSerialBase>(
            port_name,
            write_msg_start_seq,
            read_msg_start_seq,
            sensor_update_msg_len_,
            use_checksum,
            verbose_);

        backup_serial_interface = std::make_shared<ghost_serial::JetsonSerialBase>(
            backup_port_name,
            write_msg_start_seq,
            read_msg_start_seq,
            sensor_update_msg_len_,
            use_checksum,
            verbose_);

        // Sensor Update Msg Publisher
        sensor_update_pub_ = create_publisher<ghost_msgs::msg::V5SensorUpdate>("v5/sensor_update", 10);
        competition_state_pub_ = create_publisher<ghost_msgs::msg::V5CompetitionState>("v5/competition_state", 10);
        joystick_pub_ = create_publisher<ghost_msgs::msg::V5Joystick>("v5/joystick", 10);

        // Actuator Command Msg Subscriber
        actuator_command_sub_ = create_subscription<ghost_msgs::msg::V5ActuatorCommand>(
            "v5/actuator_commands",
            10,
            std::bind(&JetsonV5SerialNode::actuatorCommandCallback, this, _1));
    }

    JetsonV5SerialNode::~JetsonV5SerialNode()
    {
        if (reader_thread_init_)
        {
            reader_thread_.join();
        }
    }

    bool JetsonV5SerialNode::initSerialBlocking()
    {
        // Wait for serial to become available
        bool serial_open = false;
        bool backup = false;
        while (rclcpp::ok() && !serial_open)
        {
            if(backup){
                serial_open = backup_serial_interface->trySerialInit();
                
                // Hack
                if(serial_open){
                    serial_base_interface_ = backup_serial_interface;
                }
            }
            else{
                serial_open = serial_base_interface_->trySerialInit();
            }

            if(!serial_open){
                backup = !backup;
            }
            std::this_thread::sleep_for(10ms);
        }

        // Start Reader Thread
        if (rclcpp::ok() && using_reader_thread_)
        {
            reader_thread_ = std::thread(&JetsonV5SerialNode::readerLoop, this);
        }
        return serial_open;
    }

    void JetsonV5SerialNode::readerLoop()
    {
        RCLCPP_INFO(get_logger(), "Starting Reader Thread");

        reader_thread_init_ = true;
        while (rclcpp::ok())
        {
            try
            {
                int msg_len;
                bool msg_found = serial_base_interface_->readMsgFromSerial(sensor_update_msg_.data(), msg_len);

                if (msg_found)
                {
                    publishV5SensorUpdate(sensor_update_msg_.data());
                }
            }
            catch (std::exception &e)
            {
                std::cout << "Error: " << e.what() << std::endl;
            }
        }

        RCLCPP_INFO(get_logger(), "Ending Serial Reader Thread");
    }

    void JetsonV5SerialNode::actuatorCommandCallback(const ghost_msgs::msg::V5ActuatorCommand::SharedPtr msg)
    {
        if(verbose_){
            RCLCPP_INFO(get_logger(), "Received Actuator Command");
        }

        // Pack into single msg
        int buffer_index = 0;
        unsigned char msg_buffer[actuator_command_msg_len_] = {
            0,
        };

        // Assign motor commands
        for (auto motor_pair : ghost_v5_config::actuator_command_config)
        {
            memcpy(msg_buffer + 4 * (buffer_index++), &(msg->motor_commands[motor_pair.first].desired_voltage), 4);
            memcpy(msg_buffer + 4 * (buffer_index++), &(msg->motor_commands[motor_pair.first].desired_velocity), 4);
            if (motor_pair.second)
            {
                memcpy(msg_buffer + 4 * (buffer_index++), &(msg->motor_commands[motor_pair.first].desired_angle), 4);
            }
        }

        // Digital Outputs
        uint8_t digital_out_byte = 0;
        for (int i = 0; i < 7; i++)
        {
            digital_out_byte += msg->digital_out_vector[i];
			digital_out_byte <<= 1;
        }
        digital_out_byte += msg->digital_out_vector[7];
        memcpy(msg_buffer + 4 * (buffer_index), &digital_out_byte, 1);
        memcpy(msg_buffer + 4 * (buffer_index) + 1, &(msg->msg_id), 4);

        serial_base_interface_->writeMsgToSerial(msg_buffer, actuator_command_msg_len_);
    }

    void JetsonV5SerialNode::publishV5SensorUpdate(unsigned char buffer[])
    {
        if(verbose_){
            RCLCPP_INFO(get_logger(), "Publishing Sensor Update");
        }

        auto curr_ros_time = get_clock()->now();

        auto encoder_state_msg = ghost_msgs::msg::V5SensorUpdate{};
        encoder_state_msg.header.stamp = curr_ros_time - rclcpp::Duration(7.36ms);

        auto joystick_msg = ghost_msgs::msg::V5Joystick{};
        joystick_msg.header.stamp = curr_ros_time - rclcpp::Duration(7.36ms);

        auto competition_state_msg = ghost_msgs::msg::V5CompetitionState{};
        competition_state_msg.header.stamp = curr_ros_time - rclcpp::Duration(7.36ms);

        // Copy sensor device data to ros msg
        int buffer_index = 0;
        for (auto motor_id : ghost_v5_config::sensor_update_motor_config)
        {
            // Set Device Name from Config Enum ID
            encoder_state_msg.encoders[motor_id].device_name = ghost_v5_config::device_names.at(motor_id);
            encoder_state_msg.encoders[motor_id].device_id = motor_id;

            // Copy encoder angle
            float angle;
            memcpy(&angle, buffer + 4 * (buffer_index++), 4);
            encoder_state_msg.encoders[motor_id].angle_degrees = angle;

            // Copy encoder velocity
            float velocity;
            memcpy(&velocity, buffer + 4 * (buffer_index++), 4);
            encoder_state_msg.encoders[motor_id].velocity_rpm = velocity;

            // Copy motor voltage
            int32_t voltage;
            memcpy(&voltage, buffer + 4 * (buffer_index++), 4);
            encoder_state_msg.encoders[motor_id].voltage_mv = voltage;

            // Copy motor current
            float current;
            memcpy(&current, buffer + 4 * (buffer_index++), 4);
            encoder_state_msg.encoders[motor_id].current_ma = current;
        }

        for (auto sensor_id : ghost_v5_config::sensor_update_sensor_config)
        {
            // Set Device Name from Config Enum ID
            encoder_state_msg.encoders[sensor_id].device_name = ghost_v5_config::device_names.at(sensor_id);
            encoder_state_msg.encoders[sensor_id].device_id = sensor_id;

            // Copy encoder angle
            float angle;
            memcpy(&angle, buffer + 4 * (buffer_index++), 4);
            encoder_state_msg.encoders[sensor_id].angle_degrees = angle;

            // Copy encoder velocity
            float velocity;
            memcpy(&velocity, buffer + 4 * (buffer_index++), 4);
            encoder_state_msg.encoders[sensor_id].velocity_rpm = velocity;
        }

        // Joystick Channels
        memcpy(&(joystick_msg.joystick_left_x), buffer + 4 * (buffer_index++), 4);
        memcpy(&(joystick_msg.joystick_left_y), buffer + 4 * (buffer_index++), 4);
        memcpy(&(joystick_msg.joystick_right_x), buffer + 4 * (buffer_index++), 4);
        memcpy(&(joystick_msg.joystick_right_y), buffer + 4 * (buffer_index++), 4);

        // Buffers to store extracted V5 Msg
        uint16_t digital_states = 0;
        memcpy(&digital_states, buffer + 4 * buffer_index, 2);

        // Joystick Buttons
        joystick_msg.joystick_btn_a =        digital_states & 0x8000;
        joystick_msg.joystick_btn_b =        digital_states & 0x4000;
        joystick_msg.joystick_btn_x =        digital_states & 0x2000;
        joystick_msg.joystick_btn_y =        digital_states & 0x1000;
        joystick_msg.joystick_btn_up =       digital_states & 0x0800;
        joystick_msg.joystick_btn_down =     digital_states & 0x0400;
        joystick_msg.joystick_btn_left =     digital_states & 0x0200;
        joystick_msg.joystick_btn_right =    digital_states & 0x0100;
        joystick_msg.joystick_btn_l1 =       digital_states & 0x0080;
        joystick_msg.joystick_btn_l2 =       digital_states & 0x0040;
        joystick_msg.joystick_btn_r1 =       digital_states & 0x0020;
        joystick_msg.joystick_btn_r2 =       digital_states & 0x0010;

        // Competition state
        competition_state_msg.is_disabled = digital_states & 0x0008;
        competition_state_msg.is_autonomous = digital_states & 0x0004;
        competition_state_msg.is_connected = digital_states & 0x0002;

        // Device Connected Vector
        uint32_t device_connected_bit_vector = 0;
        memcpy(&device_connected_bit_vector, buffer + 4 * buffer_index + 2, 4);
        
        for (auto motor_id : ghost_v5_config::sensor_update_motor_config)
        {
            encoder_state_msg.encoders[motor_id].device_connected = device_connected_bit_vector & ghost_serial::BITMASK_ARR_32BIT[motor_id];
        }
        for (auto sensor_id : ghost_v5_config::sensor_update_sensor_config)
        {
            encoder_state_msg.encoders[sensor_id].device_connected = device_connected_bit_vector & ghost_serial::BITMASK_ARR_32BIT[sensor_id];
        }

        // Update Msg Sequence ID
        uint32_t msg_id;
        memcpy(&msg_id, buffer + 4 * buffer_index + 2 + 4, 4);

        encoder_state_msg.msg_id      = msg_id;
        joystick_msg.msg_id           = msg_id;
        competition_state_msg.msg_id  = msg_id;

        if(verbose_){
            RCLCPP_INFO(get_logger(), "Going to publish?");
        }

        // Publish update
        sensor_update_pub_->publish(encoder_state_msg);
        competition_state_pub_->publish(competition_state_msg);
        joystick_pub_->publish(joystick_msg);
    }

} // namespace ghost_ros

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);

    std::cout << "Start" << std::endl;
    auto serial_node = std::make_shared<ghost_ros::JetsonV5SerialNode>();
    serial_node->initSerialBlocking();
    std::cout << "Done" << std::endl;
    
    rclcpp::spin(serial_node);
    rclcpp::shutdown();
    return 0;
}