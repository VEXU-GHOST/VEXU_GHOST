
#ifndef GHOST_SERIAL__GHOST_SERIAL_NODE_HPP
#define GHOST_SERIAL__GHOST_SERIAL_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include <yaml-cpp/yaml.h>

#include "ghost_msgs/msg/actuator_commands.hpp"
#include "ghost_msgs/msg/sensor_update.hpp"

#include "serial_interface/serial_interface.hpp"

namespace ghost_serial
{

    class GhostSerialNode : public rclcpp::Node
    {
    public:
        GhostSerialNode(std::string config_file);
        ~GhostSerialNode();
        void initSerial();

        void startReaderThread();

    private:
        // Process incoming/outgoing msgs
        void actuatorCommandCallback(const ghost_msgs::msg::ActuatorCommands::SharedPtr msg);
        void publishSensorUpdate();

        void readerLoop();

        // Config Params
        YAML::Node config_yaml_;
        int msg_len_;

        // ROS Topics
        rclcpp::Subscription<ghost_msgs::msg::ActuatorCommands>::SharedPtr actuator_command_sub_;
        rclcpp::Publisher<ghost_msgs::msg::SensorUpdate>::SharedPtr sensor_update_pub_;

        // Serial Interface
        std::unique_ptr<SerialInterface> serial_interface_;
        std::vector<unsigned char> new_msg_;
        std::thread reader_thread_;
    };

} // namespace ghost_serial
#endif