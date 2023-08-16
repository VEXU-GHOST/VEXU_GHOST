#ifndef GHOST_ROS__FOXGLOVE_DIAGNOSTICS_NODE_HPP
#define GHOST_ROS__FOXGLOVE_DIAGNOSTICS_NODE_HPP

#include <map>

#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

#include "rclcpp/rclcpp.hpp"
#include "ghost_msgs/msg/v5_sensor_update.hpp"
#include "ghost_msgs/msg/v5_actuator_command.hpp"

namespace ghost_ros
{
    class FoxgloveDiagnosticsNode : public rclcpp::Node
    {
    public:
        FoxgloveDiagnosticsNode();

    private:
        void V5SensorUpdateCallback(const ghost_msgs::msg::V5SensorUpdate::SharedPtr msg);
        void V5ActuatorCommandCallback(const ghost_msgs::msg::V5ActuatorCommand::SharedPtr msg);

        // Subscriptions
        rclcpp::Subscription<ghost_msgs::msg::V5SensorUpdate>::SharedPtr v5_sensor_update_sub_;
        rclcpp::Subscription<ghost_msgs::msg::V5ActuatorCommand>::SharedPtr v5_actuator_command_sub_;

        // Publishers
        std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr> motor_state_pubs_;
        std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr> motor_setpoint_pubs_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr port_status_pub_;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_connected_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_autonomous_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_teleop_pub_;

        rclcpp::TimerBase::SharedPtr port_timer_;
        std::chrono::time_point<std::chrono::system_clock> last_port_update_time_;
        std::chrono::time_point<std::chrono::system_clock> last_heartbeat_update_time_;
        bool v5_hearbeat_toggle_;

        std::vector<std::string> motor_names_;
    };
}

#endif // GHOST_ROS__FOXGLOVE_DIAGNOSTICS_NODE_HPP