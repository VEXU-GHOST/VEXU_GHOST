#ifndef GHOST_SIM__V5_SIM_NODE_HPP
#define GHOST_SIM__V5_SIM_NODE_HPP

#include <map>

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "ghost_msgs/msg/v5_actuator_command.hpp"
#include "ghost_msgs/msg/v5_sensor_update.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace v5_sim_node{

class V5SimNode : public rclcpp::Node {
  public:

    V5SimNode(std::string config_file);

    // Topic callback functions
    void JoystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void ActuatorCmdCallback(const ghost_msgs::msg::V5ActuatorCommand::SharedPtr msg);
    
    // Topic publish functions
    void PublishStateUpdate();
  
  private:

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<ghost_msgs::msg::V5ActuatorCommand>::SharedPtr actuator_cmd_sub_;
    
    // Publishers
    rclcpp::Publisher<ghost_msgs::msg::V5SensorUpdate>::SharedPtr sensor_update_pub_;
    std::map<std::string, rclcpp::Publisher<ghost_msgs::msg::V5MotorCommand>::SharedPtr> motor_pubs_;

    // Configuration
    YAML::Node v5_sim_config_yaml_;
    std::vector<std::string> motor_names_;

};
} // namespace v5_sim_node

#endif