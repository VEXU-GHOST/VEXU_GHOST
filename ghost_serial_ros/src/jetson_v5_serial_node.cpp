#include "ghost_serial_ros/jetson_v5_serial_node.hpp"
#include "ghost_common/v5_robot_config_defs.hpp"
#include "ghost_serial/serial_utils/bitmasks.hpp"

using std::placeholders::_1;
using namespace std::literals::chrono_literals;
namespace ghost_serial_ros
{

    JetsonV5SerialNode::JetsonV5SerialNode() : Node("ghost_serial_node"), serial_open_(false), using_backup_port_(false)
    {
        // Load ROS Params
        declare_parameter("use_checksum", true);
        use_checksum_ = get_parameter("use_checksum").as_bool();

        declare_parameter("verbose", false);
        verbose_ = get_parameter("verbose").as_bool();

        declare_parameter("read_msg_start_seq", "sout");
        read_msg_start_seq_ = get_parameter("read_msg_start_seq").as_string();

        declare_parameter("write_msg_start_seq", "msg");
        write_msg_start_seq_ = get_parameter("write_msg_start_seq").as_string();

        declare_parameter("port_name", "/dev/ttyACM1");
        port_name_ = get_parameter("port_name").as_string();

        declare_parameter("backup_port_name", "/dev/ttyACM2");
        backup_port_name_ = get_parameter("backup_port_name").as_string();

        RCLCPP_DEBUG(get_logger(), "Port Name: %s", port_name_.c_str());
        RCLCPP_DEBUG(get_logger(), "Backup Port Name: %s", backup_port_name_.c_str());

        // Calculate Msg Sizes based on robot configuration
        actuator_command_msg_len_ = ghost_v5_config::get_actuator_command_msg_len();
        sensor_update_msg_len_ = ghost_v5_config::get_sensor_update_msg_len();

        int incoming_packet_len = sensor_update_msg_len_ +
                                  use_checksum_ +
                                  read_msg_start_seq_.size() +
                                  2; // Cobs Encoding adds two bytes

        RCLCPP_DEBUG(get_logger(), "Actuator Command Msg Length: %d", actuator_command_msg_len_);
        RCLCPP_DEBUG(get_logger(), "State Update Msg Length: %d", sensor_update_msg_len_);
        RCLCPP_DEBUG(get_logger(), "Incoming Packet Length: %d", incoming_packet_len);

        // Serial Interface
        serial_base_interface_ = std::make_shared<ghost_serial::JetsonSerialBase>(
            write_msg_start_seq_,
            read_msg_start_seq_,
            sensor_update_msg_len_,
            use_checksum_,
            verbose_);

        // Sensor Update Msg Publisher
        state_update_pub_ = create_publisher<ghost_msgs::msg::V5SensorUpdate>("v5/state_update", 10);

        // Actuator Command Msg Subscriber
        actuator_command_sub_ = create_subscription<ghost_msgs::msg::V5ActuatorCommand>(
            "v5/actuator_commands",
            10,
            std::bind(&JetsonV5SerialNode::actuatorCommandCallback, this, _1));

        // Start Serial Thread
        serial_thread_ = std::thread(&JetsonV5SerialNode::serialLoop, this);
        serial_timeout_thread_ = std::thread(&JetsonV5SerialNode::serialTimeoutLoop, this);
    }

    JetsonV5SerialNode::~JetsonV5SerialNode()
    {
        serial_thread_.join();
        serial_timeout_thread_.join();
    }

    bool JetsonV5SerialNode::initSerial()
    {
        // Wait for serial to become available
        try
        {
            if (!using_backup_port_)
            {
                RCLCPP_DEBUG(get_logger(), "Attempting to open %s", port_name_.c_str());
                serial_open_ = serial_base_interface_->trySerialInit(port_name_);
            }
            else
            {
                RCLCPP_DEBUG(get_logger(), "Attempting to open %s", backup_port_name_.c_str());
                serial_open_ = serial_base_interface_->trySerialInit(backup_port_name_);
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), e.what());
        }

        if (!serial_open_)
        {
            using_backup_port_ = !using_backup_port_;
        }
        return serial_open_;
    }

    void JetsonV5SerialNode::serialTimeoutLoop()
    {
        while (rclcpp::ok())
        {
            if (std::chrono::system_clock::now() - last_msg_time_ > 100ms && serial_open_)
            {
                // Acquire exclusive access to serial port, and then reset
                std::unique_lock<std::mutex> serial_lock(serial_reset_mutex_);
                serial_open_ = false;

                // Serial Interface
                serial_base_interface_ = std::make_shared<ghost_serial::JetsonSerialBase>(
                    write_msg_start_seq_,
                    read_msg_start_seq_,
                    sensor_update_msg_len_,
                    use_checksum_,
                    verbose_);

                serial_lock.unlock();
            }
        }
    }

    void JetsonV5SerialNode::serialLoop()
    {
        while (rclcpp::ok())
        {
            // Ensure serial timeout does not interrupt a msg read operation
            std::unique_lock<std::mutex> serial_lock(serial_reset_mutex_);
            if (serial_open_)
            {
                RCLCPP_DEBUG(get_logger(), "Serial Loop is Running");
                try
                {
                    int msg_len;
                    bool msg_found = serial_base_interface_->readMsgFromSerial(sensor_update_msg_.data(), msg_len);

                    if (msg_found)
                    {
                        RCLCPP_DEBUG(get_logger(), "Received new message over serial");
                        last_msg_time_ = std::chrono::system_clock::now();
                        publishV5SensorUpdate(sensor_update_msg_.data());
                    }
                }
                catch (std::exception &e)
                {
                    RCLCPP_ERROR(get_logger(), e.what());
                }
            }
            else
            {
                RCLCPP_DEBUG(get_logger(), "Initializing Serial");
                initSerial();
                std::this_thread::sleep_for(10ms);
            }
            serial_lock.unlock();
        }
    }

    void JetsonV5SerialNode::actuatorCommandCallback(const ghost_msgs::msg::V5ActuatorCommand::SharedPtr msg)
    {
        RCLCPP_DEBUG(get_logger(), "Received Actuator Command");

        if (!serial_open_)
        {
            RCLCPP_ERROR(get_logger(), "Cannot write to serial, port is not open");
            return;
        }

        // Pack into single msg
        int buffer_8bit_index = 0;
        unsigned char msg_buffer[actuator_command_msg_len_] = {
            0,
        };

        // Assign motor commands
        for (const auto &[motor_name, motor_config] : ghost_v5_config::motor_config_map)
        {
            auto motor_id = motor_config.port;
            memcpy(msg_buffer + buffer_8bit_index, &(msg->motor_commands[motor_id].current_limit), 4);
            buffer_8bit_index += 4;
            memcpy(msg_buffer + buffer_8bit_index, &(msg->motor_commands[motor_id].desired_position), 4);
            buffer_8bit_index += 4;
            memcpy(msg_buffer + buffer_8bit_index, &(msg->motor_commands[motor_id].desired_velocity), 4);
            buffer_8bit_index += 4;
            memcpy(msg_buffer + buffer_8bit_index, &(msg->motor_commands[motor_id].desired_voltage), 4);
            buffer_8bit_index += 4;
            memcpy(msg_buffer + buffer_8bit_index, &(msg->motor_commands[motor_id].desired_torque), 4);
            buffer_8bit_index += 4;

            // Pack actuator flags into one byte
            uint8_t actuator_flags_byte = 0;
            actuator_flags_byte += msg->motor_commands[motor_id].position_control;
            actuator_flags_byte <<= 1;
            actuator_flags_byte += msg->motor_commands[motor_id].velocity_control;
            actuator_flags_byte <<= 1;
            actuator_flags_byte += msg->motor_commands[motor_id].voltage_control;
            actuator_flags_byte <<= 1;
            actuator_flags_byte += msg->motor_commands[motor_id].torque_control;

            // Copy to msg buffer
            memcpy(msg_buffer + buffer_8bit_index, &actuator_flags_byte, 1);
            buffer_8bit_index++;
        }

        // Msg ID
        memcpy(msg_buffer + buffer_8bit_index, &(msg->msg_id), 4);
        buffer_8bit_index += 4;

        // Digital Outputs
        uint8_t digital_out_byte = 0;
        for (int i = 0; i < 7; i++)
        {
            digital_out_byte += msg->digital_port_vector[i];
            digital_out_byte <<= 1;
        }
        digital_out_byte += msg->digital_port_vector[7];
        memcpy(msg_buffer + buffer_8bit_index, &digital_out_byte, 1);

        serial_base_interface_->writeMsgToSerial(msg_buffer, actuator_command_msg_len_);
    }

    void JetsonV5SerialNode::publishV5SensorUpdate(unsigned char buffer[])
    {
        RCLCPP_DEBUG(get_logger(), "Publishing Sensor Update");

        auto curr_ros_time = get_clock()->now();

        auto state_update_msg = ghost_msgs::msg::V5SensorUpdate{};
        state_update_msg.header.stamp = curr_ros_time - rclcpp::Duration(7.36ms);

        // Copy sensor device data to ros msg
        int buffer_index = 0;
        for (const auto & [motor_name, motor_config] : ghost_v5_config::motor_config_map)
        {
            auto motor_id = motor_config.port;
            // Set Device Name from Config Enum ID
            state_update_msg.encoders[motor_id].device_name = motor_name;
            state_update_msg.encoders[motor_id].device_id = motor_id;

            // Copy encoder angle
            float position;
            memcpy(&position, buffer + 4 * (buffer_index++), 4);
            state_update_msg.encoders[motor_id].position_degrees = position;

            // Copy encoder velocity
            float velocity;
            memcpy(&velocity, buffer + 4 * (buffer_index++), 4);
            state_update_msg.encoders[motor_id].velocity_rpm = velocity;

            // Copy motor voltage
            float voltage;
            memcpy(&voltage, buffer + 4 * (buffer_index++), 4);
            state_update_msg.encoders[motor_id].voltage_mv = voltage;

            // Copy motor current
            float current;
            memcpy(&current, buffer + 4 * (buffer_index++), 4);
            state_update_msg.encoders[motor_id].current_ma = current;

            // Copy motor temp
            float temp;
            memcpy(&temp, buffer + 4 * (buffer_index++), 4);
            state_update_msg.encoders[motor_id].temp_c = temp;

            // Copy motor power
            float power;
            memcpy(&power, buffer + 4 * (buffer_index++), 4);
            state_update_msg.encoders[motor_id].power_w = power;
        }

        for (const auto &[sensor_name, sensor_config] : ghost_v5_config::encoder_config_map)
        {
            auto sensor_id = sensor_config.port;
            // Set Device Name from Config Enum ID
            state_update_msg.encoders[sensor_id].device_name = sensor_name;
            state_update_msg.encoders[sensor_id].device_id = sensor_id;

            // Copy encoder angle
            float position;
            memcpy(&position, buffer + 4 * (buffer_index++), 4);
            state_update_msg.encoders[sensor_id].position_degrees = position;

            // Copy encoder velocity
            float velocity;
            memcpy(&velocity, buffer + 4 * (buffer_index++), 4);
            state_update_msg.encoders[sensor_id].velocity_rpm = velocity;
        }

        // Joystick Channels
        memcpy(&(state_update_msg.joystick_left_x), buffer + 4 * (buffer_index++), 4);
        memcpy(&(state_update_msg.joystick_left_y), buffer + 4 * (buffer_index++), 4);
        memcpy(&(state_update_msg.joystick_right_x), buffer + 4 * (buffer_index++), 4);
        memcpy(&(state_update_msg.joystick_right_y), buffer + 4 * (buffer_index++), 4);

        // Buffers to store extracted V5 Msg
        uint16_t digital_states = 0;
        memcpy(&digital_states, buffer + 4 * buffer_index, 2);

        // Joystick Buttons
        state_update_msg.joystick_btn_a = digital_states & 0x8000;
        state_update_msg.joystick_btn_b = digital_states & 0x4000;
        state_update_msg.joystick_btn_x = digital_states & 0x2000;
        state_update_msg.joystick_btn_y = digital_states & 0x1000;
        state_update_msg.joystick_btn_up = digital_states & 0x0800;
        state_update_msg.joystick_btn_down = digital_states & 0x0400;
        state_update_msg.joystick_btn_left = digital_states & 0x0200;
        state_update_msg.joystick_btn_right = digital_states & 0x0100;
        state_update_msg.joystick_btn_l1 = digital_states & 0x0080;
        state_update_msg.joystick_btn_l2 = digital_states & 0x0040;
        state_update_msg.joystick_btn_r1 = digital_states & 0x0020;
        state_update_msg.joystick_btn_r2 = digital_states & 0x0010;

        // Competition state
        state_update_msg.is_disabled = digital_states & 0x0008;
        state_update_msg.is_autonomous = digital_states & 0x0004;
        state_update_msg.is_connected = digital_states & 0x0002;

        // Device Connected Vector
        uint32_t device_connected_bit_vector = 0;
        memcpy(&device_connected_bit_vector, buffer + 4 * buffer_index + 2, 4);

        for (const auto &[motor_name, motor_config]: ghost_v5_config::motor_config_map)
        {
            state_update_msg.encoders[motor_config.port].device_connected = device_connected_bit_vector & ghost_serial::BITMASK_ARR_32BIT[motor_config.port];
        }
        for (const auto &[sensor_name, sensor_config] : ghost_v5_config::encoder_config_map)
        {
            state_update_msg.encoders[sensor_config.port].device_connected = device_connected_bit_vector & ghost_serial::BITMASK_ARR_32BIT[sensor_config.port];
        }

        // Update Msg Sequence ID
        uint32_t msg_id;
        memcpy(&msg_id, buffer + 4 * buffer_index + 2 + 4, 4);

        state_update_msg.msg_id = msg_id;

        // Publish update
        state_update_pub_->publish(state_update_msg);
    }

} // namespace ghost_serial_ros

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto serial_node = std::make_shared<ghost_serial_ros::JetsonV5SerialNode>();
    rclcpp::spin(serial_node);
    rclcpp::shutdown();
    return 0;
}