#include <chrono>

#include "ghost_ros/robot_config/v5_port_config.hpp"
#include "ghost_ros/robot_config/v5_serial_msg_config.hpp"

#include "ghost_ros/ros_nodes/robot_state_machine_node.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace ghost_ros
{

    RobotStateMachineNode::RobotStateMachineNode(std::string config_file) : 
        rclcpp::Node("ghost_state_machine"),
        curr_robot_state_{robot_state_e::DISABLED},
        curr_pose_msg_id_{0},
        curr_joystick_msg_id_{0},
        curr_comp_state_msg_id_{0}
    {
        actuator_command_pub_ = create_publisher<ghost_msgs::msg::V5ActuatorCommand>(
            "v5/actuator_commands",
            10);

        competition_state_sub_ = create_subscription<ghost_msgs::msg::V5CompetitionState>(
            "v5/competition_state",
            10,
            [this](const ghost_msgs::msg::V5CompetitionState::SharedPtr msg)
            {
                curr_comp_state_msg_id_ = msg->msg_id;
                curr_comp_state_msg_ = msg;

                if (msg->is_autonomous && curr_robot_state_ != robot_state_e::AUTONOMOUS)
                {
                    curr_robot_state_ = robot_state_e::AUTONOMOUS;
                    auton_start_time_ = std::chrono::system_clock::now();
                }
                else if (msg->is_disabled)
                {
                    curr_robot_state_ = robot_state_e::DISABLED;
                }
                else if(!msg->is_autonomous && !msg->is_disabled)
                {
                    curr_robot_state_ = robot_state_e::TELEOP;
                }

                updateController();
            });

        v5_joystick_sub_ = create_subscription<ghost_msgs::msg::V5Joystick>(
            "v5/joystick",
            10,
            [this](const ghost_msgs::msg::V5Joystick::SharedPtr msg)
            {
                curr_joystick_msg_id_ = msg->msg_id;
                curr_joystick_msg_ = msg;
                updateController();
            });

        robot_pose_sub_ = create_subscription<ghost_msgs::msg::GhostRobotState>(
            "estimation/robot_pose",
            10,
            [this](const ghost_msgs::msg::GhostRobotState::SharedPtr msg)
            {
                curr_pose_msg_id_ = msg->msg_id;
                curr_pose_msg_ = msg;
                updateController();
            });
    }

    void RobotStateMachineNode::updateController()
    {
        // Comparing msg ids ensures only one control update is performed for a given sensor update
        if ((curr_joystick_msg_id_ == curr_comp_state_msg_id_)) //&& (curr_joystick_msg_id_ == curr_pose_msg_id_))
        {
            switch(curr_robot_state_){
                case robot_state_e::AUTONOMOUS:

                break;

                case robot_state_e::TELEOP:
                    teleop();
                break;

                case robot_state_e::DISABLED:

                break;
            }
        }
    }

    void RobotStateMachineNode::teleop(){
        auto actuator_cmd_msg = ghost_msgs::msg::V5ActuatorCommand{};
        actuator_cmd_msg.header.stamp = get_clock()->now();
        actuator_cmd_msg.msg_id = curr_joystick_msg_id_;

        auto motor_cmd_msg = ghost_msgs::msg::V5MotorCommand{};

        int motor_index = 0;

        if (curr_joystick_msg_->joystick_btn_a)
        {
            actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_LEFT_FRONT_MOTOR].desired_velocity = curr_joystick_msg_->joystick_right_y * 600.0;
        }
        else if (curr_joystick_msg_->joystick_btn_b)
        {
            actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_LEFT_BACK_MOTOR].desired_velocity = curr_joystick_msg_->joystick_right_y * 600.0;
        }
        else if (curr_joystick_msg_->joystick_btn_x)
        {
            actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_RIGHT_FRONT_MOTOR].desired_velocity = curr_joystick_msg_->joystick_right_y * 600.0;
        }
        else if (curr_joystick_msg_->joystick_btn_y)
        {
            actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_RIGHT_BACK_MOTOR].desired_velocity = curr_joystick_msg_->joystick_right_y * 600.0;
        }
        else if (curr_joystick_msg_->joystick_btn_right)
        {
            actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_BACK_RIGHT_1_MOTOR].desired_velocity = curr_joystick_msg_->joystick_right_y * 600.0;
            actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_BACK_RIGHT_2_MOTOR].desired_velocity = curr_joystick_msg_->joystick_right_y * 600.0;
        }
        else if (curr_joystick_msg_->joystick_btn_left)
        {
            actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_BACK_LEFT_1_MOTOR].desired_velocity = curr_joystick_msg_->joystick_right_y * 600.0;
            actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_BACK_LEFT_2_MOTOR].desired_velocity = curr_joystick_msg_->joystick_right_y * 600.0;
        }

        actuator_command_pub_->publish(actuator_cmd_msg);
    }

} // namespace ghost_ros