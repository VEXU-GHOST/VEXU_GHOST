
#ifndef GHOST_SERIAL__GHOST_SERIAL_NODE_HPP
#define GHOST_SERIAL__GHOST_SERIAL_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include <yaml-cpp/yaml.h>

#include "ghost_msgs/msg/actuator_commands.hpp"
#include "ghost_msgs/msg/sensor_update.hpp"

#include "serial_interface/serial_interface.hpp"

namespace ghost_serial
{

    class V5SerialNode : public rclcpp::Node
    {
    public:
        V5SerialNode(std::string config_file);
        ~V5SerialNode();

        // Starts
        void initSerialInterfaceBlocking();

    private:
        // Process incoming/outgoing msgs w/ ROS
        void actuatorCommandCallback(const ghost_msgs::msg::ActuatorCommands::SharedPtr msg);
        void publishSensorUpdate(unsigned char buffer[]);

        // Background thread loop for processing serial reads
        void readerLoop();

        // Config Params
        YAML::Node config_yaml_;
        int msg_len_;
        bool using_reader_thread_;
        bool verbose_;

        // ROS Topics
        rclcpp::Subscription<ghost_msgs::msg::ActuatorCommands>::SharedPtr actuator_command_sub_;
        rclcpp::Publisher<ghost_msgs::msg::SensorUpdate>::SharedPtr sensor_update_pub_;

        // Serial Interface
        std::shared_ptr<SerialInterface> serial_interface_;
        std::vector<unsigned char> new_msg_;

        // Reader Thread
        std::thread reader_thread_;
        std::atomic_bool reader_thread_init_;
    };

} // namespace ghost_serial
#endif