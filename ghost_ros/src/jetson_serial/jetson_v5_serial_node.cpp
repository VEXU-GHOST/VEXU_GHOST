
#include "ghost_ros/jetson_serial/jetson_v5_serial_node.hpp"

#include "ghost_ros/robot_config/v5_serial_msg_config.hpp"
#include "ghost_serial/serial_utils/bitmasks.hpp"

using std::placeholders::_1;
using namespace std::literals::chrono_literals;
namespace ghost_serial
{

    JetsonV5SerialNode::JetsonV5SerialNode(std::string config_file) : Node("ghost_serial_node"), reader_thread_init_(false)
    {
        // Load config file
        config_yaml_ = YAML::LoadFile(config_file);
        using_reader_thread_ = config_yaml_["using_reader_thread"].as<bool>();
        bool use_checksum = config_yaml_["use_checksum"].as<bool>();

        // Calculate Msg Sizes based on robot configuration
        actuator_command_msg_len_ = 2 * 4 * ghost_v5_config::actuator_command_config.size() + 1;
        for (auto &pair : ghost_v5_config::actuator_command_config)
        {
            actuator_command_msg_len_ += (pair.second) ? 4 : 0; // Add four bytes for each motor using position control
        }

        state_update_msg_len_ = (ghost_v5_config::state_update_motor_config.size() + ghost_v5_config::state_update_sensor_config.size()) * 2 * 4;
        state_update_msg_len_ += ghost_v5_config::state_update_extra_byte_count + use_checksum; // 4x int32 Joystick Channels, 2x bytes of btns/digital outs/competition modes, 1 byte checksum

        int incoming_packet_len = state_update_msg_len_ +
                                  use_checksum +
                                  config_yaml_["msg_start_seq"].as<std::string>().size() +
                                  2; // Cobs Encoding adds two bytes

        RCLCPP_INFO(get_logger(), "Actuator Command Msg Length: %d", actuator_command_msg_len_);
        RCLCPP_INFO(get_logger(), "State Update Msg Length: %d", state_update_msg_len_);
        RCLCPP_INFO(get_logger(), "Incoming Packet Length: %d", incoming_packet_len);

        // Array to store latest incoming msg
        robot_state_msg_ = std::vector<unsigned char>(state_update_msg_len_, 0);

        // Serial Interface
        serial_base_interface_ = std::make_shared<JetsonSerialBase>(
            config_yaml_["port_name"].as<std::string>(),
            config_yaml_["msg_start_seq"].as<std::string>(),
            state_update_msg_len_,
            use_checksum);

        // Sensor Update Msg Publisher
        state_update_pub_ = create_publisher<ghost_msgs::msg::RobotStateUpdate>("v5/state_update", 10);
        competition_state_pub_ = create_publisher<ghost_msgs::msg::V5CompetitionState>("v5/competition_state", 10);
        joystick_pub_ = create_publisher<ghost_msgs::msg::V5Joystick>("v5/joystick", 10);

        // Actuator Command Msg Subscriber
        actuator_command_sub_ = create_subscription<ghost_msgs::msg::RobotActuatorCommand>(
            "v5_actuator_commands",
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

    void JetsonV5SerialNode::initSerialBlocking()
    {
        // Wait for serial to become available
        bool serial_open = false;
        while (rclcpp::ok() && !serial_open)
        {
            serial_open = serial_base_interface_->trySerialInit();
            std::this_thread::sleep_for(10ms);
        }

        // Start Reader Thread
        if (rclcpp::ok() && using_reader_thread_)
        {
            reader_thread_ = std::thread(&JetsonV5SerialNode::readerLoop, this);
        }
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
                bool msg_found = serial_base_interface_->readMsgFromSerial(robot_state_msg_.data(), msg_len);

                if (msg_found)
                {
                    publishRobotStateUpdate(robot_state_msg_.data());
                }
            }
            catch (std::exception &e)
            {
                std::cout << "Error: " << e.what() << std::endl;
            }
        }

        RCLCPP_INFO(get_logger(), "Ending Serial Reader Thread");
    }

    void JetsonV5SerialNode::actuatorCommandCallback(const ghost_msgs::msg::RobotActuatorCommand::SharedPtr msg)
    {
        // RCLCPP_INFO(get_logger(), "Actuator Command");

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
        }
        digital_out_byte += msg->digital_out_vector[8];
        memcpy(msg_buffer + actuator_command_msg_len_, &digital_out_byte, 1);

        serial_base_interface_->writeMsgToSerial(msg_buffer, actuator_command_msg_len_);
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
    void JetsonV5SerialNode::publishRobotStateUpdate(unsigned char buffer[])
    {
        // RCLCPP_INFO(get_logger(), "Sensor Update");

        auto encoder_state_msg = ghost_msgs::msg::RobotStateUpdate{};
        encoder_state_msg.header.stamp = get_clock()->now() - rclcpp::Duration(7.36ms);

        auto joystick_msg = ghost_msgs::msg::V5Joystick{};
        joystick_msg.header.stamp = get_clock()->now() - rclcpp::Duration(7.36ms);

        auto competition_state_msg = ghost_msgs::msg::V5CompetitionState{};
        competition_state_msg.header.stamp = get_clock()->now() - rclcpp::Duration(7.36ms);

        // Copy sensor device data to ros msg
        int buffer_index = 0;
        for (auto motor_id : ghost_v5_config::state_update_motor_config)
        {
            // Set Device Name from Config Enum ID
            encoder_state_msg.encoders[motor_id].device_name = ghost_v5_config::device_names[motor_id];
            encoder_state_msg.encoders[motor_id].device_id = motor_id;

            // Copy encoder angle
            uint32_t angle;
            memcpy(&angle, buffer + 4 * (buffer_index++), 4);
            encoder_state_msg.encoders[motor_id].current_angle = angle;

            // Copy encoder velocity
            float velocity;
            memcpy(&velocity, buffer + 4 * (buffer_index++), 4);
            encoder_state_msg.encoders[motor_id].current_velocity = velocity;
        }

        for (auto sensor_id : ghost_v5_config::state_update_sensor_config)
        {
            // Set Device Name from Config Enum ID
            encoder_state_msg.encoders[sensor_id].device_name = ghost_v5_config::device_names[sensor_id];
            encoder_state_msg.encoders[sensor_id].device_id = sensor_id;

            // Copy encoder angle
            uint32_t angle;
            memcpy(&angle, buffer + 4 * (buffer_index++), 4);
            encoder_state_msg.encoders[sensor_id].current_angle = angle;

            // Copy encoder velocity
            float velocity;
            memcpy(&velocity, buffer + 4 * (buffer_index++), 4);
            encoder_state_msg.encoders[sensor_id].current_velocity = velocity;
        }

        // Joystick Channels
        memcpy(&(joystick_msg.joystick_left_x), buffer + 4 * (buffer_index++), 4);
        memcpy(&(joystick_msg.joystick_left_y), buffer + 4 * (buffer_index++), 4);
        memcpy(&(joystick_msg.joystick_right_x), buffer + 4 * (buffer_index++), 4);
        memcpy(&(joystick_msg.joystick_right_y), buffer + 4 * (buffer_index++), 4);

        // Buffers to store extracted V5 Msg
        uint8_t digital_outs = 0;
        uint16_t digital_states = 0;

        memcpy(&digital_states, buffer + 4 * buffer_index, 2);
        memcpy(&digital_outs, buffer + 4 * buffer_index + 2, 1);

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

        // Digital Outputs
        uint8_t bitmask[8] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};
        for (int i = 0; i < 8; i++)
        {
            encoder_state_msg.digital_out_vector[i] = digital_outs & bitmask[i];
        }

        // Device Connected Vector
        uint32_t device_connected_bit_vector = 0;
        memcpy(&device_connected_bit_vector, buffer + 4 * buffer_index + 3, 4);
        
        for (auto motor_id : ghost_v5_config::state_update_motor_config)
        {
            encoder_state_msg.encoders[motor_id].device_connected = device_connected_bit_vector & ghost_serial::BITMASK_ARR_32BIT[motor_id];
        }
        for (auto sensor_id : ghost_v5_config::state_update_sensor_config)
        {
            encoder_state_msg.encoders[sensor_id].device_connected = device_connected_bit_vector & ghost_serial::BITMASK_ARR_32BIT[sensor_id];
        }

        state_update_pub_->publish(encoder_state_msg);
        competition_state_pub_->publish(competition_state_msg);
        joystick_pub_->publish(joystick_msg);
    }

} // namespace ghost_serial