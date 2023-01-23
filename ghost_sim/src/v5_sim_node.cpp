#include "v5_sim_node.hpp"

using std::placeholders::_1;

namespace v5_sim_node{

  // Constructor
  V5SimNode::V5SimNode(std::string config_file): Node("v5_sim_node"){

    // Fetch motor names as a string vector from given YAML config file
    v5_sim_config_yaml_ = YAML::LoadFile(config_file);
    motor_names_ = v5_sim_config_yaml_["motor_names"].as<std::vector<std::string>>();

    // Initialize motor publishers map with each motor name (key) and its corresponding ROS topic (value)
    for(std::string &name : motor_names_){

      std::string nameLower = name;

      // Convert motor name to lowercase for ROS topic naming convention
      for(char &c : nameLower){
        c = tolower(c);
      }
      
      motor_pubs_[name] = this->create_publisher<ghost_msgs::msg::V5MotorCommand>(nameLower, 10);
    }


    // Create Subscriptions
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&V5SimNode::JoystickCallback, this, _1));
    actuator_cmd_sub_ = this->create_subscription<ghost_msgs::msg::RobotActuatorCommand>("actuator_cmd", 10, std::bind(&V5SimNode::ActuatorCmdCallback, this, _1));

    // Create Publishers
    state_update_pub_ = this->create_publisher<ghost_msgs::msg::RobotStateUpdate>("state_update", 10);

  }

  void V5SimNode::JoystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg){

  }
  
  // Given a RobotActuatorCommand message, echoes all valid motor commands on their own topics (valid, i.e. a motor name that exists within the publisher map).
  void V5SimNode::ActuatorCmdCallback(const ghost_msgs::msg::RobotActuatorCommand::SharedPtr msg){

    for(ghost_msgs::msg::V5MotorCommand cmd : msg->motor_commands){

      if(motor_pubs_.find(cmd.motor_name) != motor_pubs_.end()){

        motor_pubs_[cmd.motor_name]->publish(cmd);

      }

    }

  }

  void V5SimNode::PublishStateUpdate(){

  }

} // namespace v5_sim_node