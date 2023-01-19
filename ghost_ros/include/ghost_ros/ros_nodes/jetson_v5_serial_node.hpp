
#ifndef GHOST_SERIAL__JETSON_V5_SERIAL_NODE_HPP
#define GHOST_SERIAL__JETSON_V5_SERIAL_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include <yaml-cpp/yaml.h>

#include "ghost_msgs/msg/v5_actuator_command.hpp"
#include "ghost_msgs/msg/v5_sensor_update.hpp"
#include "ghost_msgs/msg/v5_competition_state.hpp"
#include "ghost_msgs/msg/v5_joystick.hpp"

#include "ghost_serial/base_interfaces/jetson_serial_base.hpp"

namespace ghost_ros
{

    class JetsonV5SerialNode : public rclcpp::Node
    {
    public:
        JetsonV5SerialNode();
        ~JetsonV5SerialNode();

        bool initSerialBlocking();

    private:
        // Process incoming/outgoing msgs w/ ROS
        void actuatorCommandCallback(const ghost_msgs::msg::V5ActuatorCommand::SharedPtr msg);
        void publishV5SensorUpdate(unsigned char buffer[]);

        // Background thread loop for processing serial reads
        void readerLoop();

        // Config Params
        YAML::Node config_yaml_;
        int max_msg_len_;
        bool using_reader_thread_;
        bool verbose_;

        // ROS Topics
        rclcpp::Subscription<ghost_msgs::msg::V5ActuatorCommand>::SharedPtr actuator_command_sub_;
        rclcpp::Publisher<ghost_msgs::msg::V5SensorUpdate>::SharedPtr sensor_update_pub_;
        rclcpp::Publisher<ghost_msgs::msg::V5Joystick>::SharedPtr joystick_pub_;
        rclcpp::Publisher<ghost_msgs::msg::V5CompetitionState>::SharedPtr competition_state_pub_;

        // Serial Interface
        std::shared_ptr<ghost_serial::JetsonSerialBase> serial_base_interface_;
        std::shared_ptr<ghost_serial::JetsonSerialBase> backup_serial_interface;
        std::vector<unsigned char> sensor_update_msg_;

        // Reader Thread
        std::thread reader_thread_;
        std::atomic_bool reader_thread_init_;

        // Msg Config
        int actuator_command_msg_len_;
        int sensor_update_msg_len_;
    };

} // namespace ghost_serial
#endif // GHOST_SERIAL__JETSON_V5_SERIAL_NODE_HPP