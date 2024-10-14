#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "ghost_msgs/msg/v5_actuator_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// Used to test diff drive state space. Input control is motor voltages to drive train

class MtrVltgPub : public rclcpp::Node
{
public:
  MtrVltgPub()
  : Node("mtr_vltg_pub"), count_(0)
  {
    publisher_ = this->create_publisher<ghost_msgs::msg::V5ActuatorCommand>(
      "v5_act_cmd",
      10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MtrVltgPub::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto cmd = std::make_shared<ghost_msgs::msg::V5ActuatorCommand>();
    auto l_mtr_cmd = std::make_shared<ghost_msgs::msg::V5MotorCommand>();
    auto r_mtr_cmd = std::make_shared<ghost_msgs::msg::V5MotorCommand>();

    l_mtr_cmd->position_command = 0.0;
    l_mtr_cmd->velocity_command = 0.0;
    l_mtr_cmd->torque_command = 0.0;
    l_mtr_cmd->voltage_command = 1.0;
    l_mtr_cmd->current_limit = 0.0;

    l_mtr_cmd->position_control = false;
    l_mtr_cmd->velocity_control = false;
    l_mtr_cmd->torque_control = false;
    l_mtr_cmd->voltage_control = true;


    r_mtr_cmd->position_command = 0.0;
    r_mtr_cmd->velocity_command = 0.0;
    r_mtr_cmd->torque_command = 0.0;
    r_mtr_cmd->voltage_command = 1.0;
    r_mtr_cmd->current_limit = 0.0;

    r_mtr_cmd->position_control = false;
    r_mtr_cmd->velocity_control = false;
    r_mtr_cmd->torque_control = false;
    r_mtr_cmd->voltage_control = true;
    // publisher_->publish(cmd);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ghost_msgs::msg::V5ActuatorCommand>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MtrVltgPub>());
  rclcpp::shutdown();
  return 0;
}
