#include "ghost_ros/ros_nodes/jetson_v5_serial_node.hpp"
#include "ghost_ros/robot_config/v5_serial_msg_config.hpp"
#include "ghost_serial/serial_utils/bitmasks.hpp"

using std::placeholders::_1;
using namespace std::literals::chrono_literals;
namespace ghost_ros
{

    JetsonV5SerialNode::JetsonV5SerialNode() : Node("ghost_serial_node"),
                                               serial_thread_init_(false),
                                               serial_open_(false)
    {
        // Load ROS Params
        declare_parameter("using_reader_thread", true);
        using_reader_thread_ = get_parameter("using_reader_thread").as_bool();

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

        RCLCPP_INFO(get_logger(), "Port Name: " + port_name_);
        RCLCPP_INFO(get_logger(), "Backup Port Name: " + backup_port_name_);

        // Calculate Msg Sizes based on robot configuration
        actuator_command_msg_len_ = ghost_v5_config::get_actuator_command_msg_len();

        sensor_update_msg_len_ = ghost_v5_config::get_sensor_update_msg_len();

        int incoming_packet_len = sensor_update_msg_len_ +
                                  use_checksum_ +
                                  read_msg_start_seq_.size() +
                                  2; // Cobs Encoding adds two bytes

        RCLCPP_INFO(get_logger(), "Actuator Command Msg Length: %d", actuator_command_msg_len_);
        RCLCPP_INFO(get_logger(), "State Update Msg Length: %d", sensor_update_msg_len_);
        RCLCPP_INFO(get_logger(), "Incoming Packet Length: %d", incoming_packet_len);

        // Array to store latest incoming msg
        sensor_update_msg_ = std::vector<unsigned char>(sensor_update_msg_len_, 0);

        // Serial Interface
        serial_base_interface_ = std::make_shared<ghost_serial::JetsonSerialBase>(
            write_msg_start_seq_,
            read_msg_start_seq_,
            sensor_update_msg_len_,
            use_checksum_,
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

        // Timer for Connection timeout
        port_timer_ = this->create_wall_timer(
            100ms,
            [this]()
            {
                if (std::chrono::system_clock::now() - this->last_msg_time_ > 100ms && this->serial_open_)
                {
                    RCLCPP_WARN(get_logger(), "Serial Timed Out");
                    // Acquire exclusive access to serial port, and then reset
                    std::unique_lock<std::mutex> serial_lock(this->serial_reset_mutex_);
                    serial_open_ = false;

                    // Serial Interface
                    serial_base_interface_ = std::make_shared<ghost_serial::JetsonSerialBase>(
                        this->write_msg_start_seq_,
                        this->read_msg_start_seq_,
                        this->sensor_update_msg_len_,
                        this->use_checksum_,
                        this->verbose_);

                    serial_lock.unlock();
                }
            });

        // Start Serial Thread
        serial_thread_ = std::thread(&JetsonV5SerialNode::serialLoop, this);
    }

    JetsonV5SerialNode::~JetsonV5SerialNode()
    {
        if (serial_thread_init_)
        {
            serial_thread_.join();
        }
    }

    bool JetsonV5SerialNode::initSerialBlocking()
    {
        // Wait for serial to become available
        bool backup = false;
        while (rclcpp::ok() && !serial_open_)
        {
            try{
                
                if (!backup)
                {
                    RCLCPP_INFO(get_logger(), "Attempting to open " + port_name_);
                    serial_open_ = serial_base_interface_->trySerialInit(port_name_);
                }
                else
                {
                    RCLCPP_INFO(get_logger(), "Attempting to open " + backup_port_name_);
                    serial_open_ = serial_base_interface_->trySerialInit(backup_port_name_);
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(get_logger(), e.what());
            }

            if (!serial_open_)
            {
                backup = !backup;
            }
            std::this_thread::sleep_for(10ms);
        }

        return serial_open_;
    }

    void JetsonV5SerialNode::serialLoop()
    {
        serial_thread_init_ = true;
        while (rclcpp::ok())
        {
            // Ensure serial timeout does not interrupt a msg read operation
            std::unique_lock<std::mutex> serial_lock(serial_reset_mutex_);
            if (serial_open_)
            {
                try
                {
                    int msg_len;
                    bool msg_found = serial_base_interface_->readMsgFromSerial(sensor_update_msg_.data(), msg_len);

                    if (msg_found)
                    {
                        last_msg_time_ = std::chrono::system_clock::now();
                        publishV5SensorUpdate(sensor_update_msg_.data());
                    }
                }
                catch (std::exception &e)
                {
                    RCLCPP_ERROR(get_logger(), std::string(e.what()));
                }
            }
            else
            {
                initSerialBlocking();
            }
            serial_lock.unlock();
        }
    }

    void JetsonV5SerialNode::actuatorCommandCallback(const ghost_msgs::msg::V5ActuatorCommand::SharedPtr msg)
    {
        if (verbose_)
        {
            RCLCPP_INFO(get_logger(), "Received Actuator Command");
        }

        if (!serial_open_)
        {
            RCLCPP_ERROR(get_logger(), "Cannot write to serial, port is not open");
            return;
        }

        // Pack into single msg
        int buffer_index = 0;
        unsigned char msg_buffer[actuator_command_msg_len_] = {
            0,
        };

        // Motor Active States
        uint32_t actuator_active_vector = 0;
        for (int i = 0; i < ghost_v5_config::actuator_command_config.size() - 1; i++)
        {
            actuator_active_vector += msg->motor_commands[ghost_v5_config::actuator_command_config[i].first].active;
			actuator_active_vector <<= 1;
        }
            actuator_active_vector += msg->motor_commands[ghost_v5_config::actuator_command_config.back().first].active;

        memcpy(msg_buffer + 4 * (buffer_index++), &actuator_active_vector, 4);

        // Assign motor commands
        for (auto motor_pair : ghost_v5_config::actuator_command_config)
        {
            memcpy(msg_buffer + 4 * (buffer_index++), &(msg->motor_commands[motor_pair.first].current_limit), 4);
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
        if (verbose_){
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
            float voltage;
            memcpy(&voltage, buffer + 4 * (buffer_index++), 4);
            encoder_state_msg.encoders[motor_id].voltage_mv = voltage;

            // Copy motor current
            float current;
            memcpy(&current, buffer + 4 * (buffer_index++), 4);
            encoder_state_msg.encoders[motor_id].current_ma = current;

            // Copy motor temp
            float temp;
            memcpy(&temp, buffer + 4 * (buffer_index++), 4);
            encoder_state_msg.encoders[motor_id].temp_c = temp;

            // Copy motor power
            float power;
            memcpy(&power, buffer + 4 * (buffer_index++), 4);
            encoder_state_msg.encoders[motor_id].power_w = power;
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

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto serial_node = std::make_shared<ghost_ros::JetsonV5SerialNode>();
    rclcpp::spin(serial_node);
    rclcpp::shutdown();
    return 0;
}