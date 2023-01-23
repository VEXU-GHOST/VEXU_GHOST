#include <chrono>
#include <thread>

#include "v5_sim_node.hpp"

int main(int argc, char* argv[]){

  rclcpp::init(argc, argv);

  // Fetch motor names as a string vector from v5_sim_node config file
  auto repo_base_dir = std::string(getenv("HOME")) + "/VEXU_GHOST/";
  auto v5_sim_config_yaml_ = YAML::LoadFile(repo_base_dir + "ghost_sim/config/v5_sim_config.yaml");
  auto motor_names_ = v5_sim_config_yaml_["motor_names"].as<std::vector<std::string>>();

  // Create RobotActuatorCommand message and fill it with sample commands for each given motor name
  auto actuator_cmd_msg = ghost_msgs::msg::RobotActuatorCommand{};
  auto motor_cmd_msg = ghost_msgs::msg::V5MotorCommand{};

  int cur_motor_num = 1;

  for(auto &motor_name : motor_names_){

    motor_cmd_msg.motor_name = motor_name;

    motor_cmd_msg.device_id        = cur_motor_num;
    motor_cmd_msg.desired_angle    = cur_motor_num;
    motor_cmd_msg.desired_velocity = cur_motor_num;
    motor_cmd_msg.desired_voltage  = cur_motor_num;
    
    motor_cmd_msg.use_position_control = true;

    actuator_cmd_msg.motor_commands[cur_motor_num++] = motor_cmd_msg;

  }

  // Create RobotActuatorCommand publisher and publish test message every second until program interruption or termination
  auto actuator_cmd_pub_ = rclcpp::Node("actuator_cmd_node").create_publisher<ghost_msgs::msg::RobotActuatorCommand>("actuator_cmd", 10);

  while(rclcpp::ok()){
   actuator_cmd_pub_->publish(actuator_cmd_msg);
   std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}