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
        curr_x_{0.0},
        curr_y_{0.0},
        curr_theta_{0.0},
        des_x_vel_{0.0},
        des_y_vel_{0.0},
        des_theta_vel_{0.0},
        curr_robot_state_{robot_state_e::DISABLED}
    {
        actuator_command_pub_ = create_publisher<ghost_msgs::msg::RobotActuatorCommand>(
            "v5/actuator_commands",
            10);

        competition_state_sub_ = create_subscription<ghost_msgs::msg::V5CompetitionState>(
            "v5/competition_state",
            10,
            std::bind(&RobotStateMachineNode::competitionStateCallback, this, _1));

        v5_joystick_sub_ = create_subscription<ghost_msgs::msg::V5Joystick>(
            "v5/joystick",
            10,
            std::bind(&RobotStateMachineNode::v5JoystickCallback ,this, _1));

        robot_pose_sub_ = create_subscription<ghost_msgs::msg::Pose2D>(
            "estimation/robot_pose",
            10,
            std::bind(&RobotStateMachineNode::robotPoseCallback, this, _1));
    }

    void RobotStateMachineNode::competitionStateCallback(const ghost_msgs::msg::V5CompetitionState::SharedPtr msg){
        if(msg->is_disabled){
            curr_robot_state_ = robot_state_e::DISABLED;
        }
        else if(msg->is_autonomous){
            curr_robot_state_ = robot_state_e::AUTONOMOUS;
        }
        else{
            curr_robot_state_ = robot_state_e::TELEOP;
        }
    }

    void RobotStateMachineNode::v5JoystickCallback(const ghost_msgs::msg::V5Joystick::SharedPtr msg){
        if(curr_robot_state_ == robot_state_e::TELEOP){
            auto actuator_cmd_msg = ghost_msgs::msg::RobotActuatorCommand{};
            actuator_cmd_msg.header.stamp = get_clock()->now() - rclcpp::Duration(7.36ms);
            
            auto motor_cmd_msg = ghost_msgs::msg::V5MotorCommand{};

            int motor_index = 0;

            if(msg->joystick_btn_a){
                actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_LEFT_FRONT_MOTOR].desired_velocity = msg->joystick_right_y*600.0;
            }
            else if(msg->joystick_btn_b){
                actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_LEFT_BACK_MOTOR].desired_velocity = msg->joystick_right_y*600.0;
            }
            else if(msg->joystick_btn_x){
                actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_RIGHT_FRONT_MOTOR].desired_velocity = msg->joystick_right_y*600.0;
            }
            else if(msg->joystick_btn_y){
                actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_RIGHT_BACK_MOTOR].desired_velocity = msg->joystick_right_y*600.0;
            }
            else if(msg->joystick_btn_right){
                actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_BACK_RIGHT_1_MOTOR].desired_velocity = msg->joystick_right_y*600.0;
                actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_BACK_RIGHT_2_MOTOR].desired_velocity = msg->joystick_right_y*600.0;
            }
            else if(msg->joystick_btn_up){
            }
            else if(msg->joystick_btn_left){
                actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_BACK_LEFT_1_MOTOR].desired_velocity = msg->joystick_right_y*600.0;
                actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_BACK_LEFT_2_MOTOR].desired_velocity = msg->joystick_right_y*600.0;
            }
            else if(msg->joystick_btn_down){
            }
            
            actuator_command_pub_->publish(actuator_cmd_msg);
        }
    }

    void RobotStateMachineNode::robotPoseCallback(const ghost_msgs::msg::Pose2D::SharedPtr msg){
        curr_x_     = msg->x;
        curr_y_     = msg->y;
        curr_theta_ = msg->theta;
    }


} // namespace ghost_ros