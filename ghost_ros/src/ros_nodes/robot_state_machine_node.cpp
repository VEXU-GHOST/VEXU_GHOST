#include <chrono>
#include <math>
#include "eigen3/Eigen/Geometry"

#include "ghost_ros/robot_config/v5_port_config.hpp"
#include "ghost_ros/robot_config/v5_serial_msg_config.hpp"

#include "ghost_ros/ros_nodes/robot_state_machine_node.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace ghost_ros
{

    RobotStateMachineNode::RobotStateMachineNode() : 
        rclcpp::Node("ghost_state_machine"),
        curr_robot_state_{robot_state_e::DISABLED},
        curr_pose_msg_id_{0},
        curr_joystick_msg_id_{0},
        curr_comp_state_msg_id_{0}
    {
        declare_parameter("max_linear_vel", 0.0);
        declare_parameter("max_angular_vel", 0.0);
        max_linear_vel_ = get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = get_parameter("max_angular_vel").as_double();

        declare_parameter("left_wheel_x",   0.0);
        declare_parameter("left_wheel_y",   0.0);
        declare_parameter("right_wheel_x",  0.0);
        declare_parameter("right_wheel_y",  0.0);
        declare_parameter("back_wheel_x",   0.0);
        declare_parameter("back_wheel_y",   0.0);

        left_wheel_pos_ = Eigen::Vector2f(
            get_parameter("left_wheel_x").as_double(),
            get_parameter("left_wheel_y").as_double(),
            );

        right_wheel_pos_ = Eigen::Vector2f(
            get_parameter("right_wheel_x").as_double(), 
            get_parameter("right_wheel_y").as_double(), 
            );

        back_wheel_pos_ = Eigen::Vector2f(
            get_parameter("back_wheel_x").as_double(), 
            get_parameter("back_wheel_y").as_double(), 
            );
        
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

        // Convert joystick to robot twist command
        Eigen::Vector2f xy_vel_cmd(curr_joystick_msg_->joystick_left_x, curr_joystick_msg_->joystick_left_y);
        float angular_vel_cmd = std::clamp<float>(curr_joystick_msg_->joystick_right_x, -1.0, 1.0) * max_angular_vel_;
        float linear_vel_cmd = std::clamp<float>(xy_vel_cmd.norm(), -1.0, 1.0) * max_linear_vel_;
        float linear_vel_dir = xy_vel_cmd / xy_vel_cmd.norm();

        // Swerve Drive Setpoints
        std::vector<float> steering_angle_cmd_{3, 0.0};
        std::vector<float> steering_velocity_cmd_{3, 0.0};
        std::vector<float> steering_voltage_cmd_{3, 0.0};

        std::vector<float> wheel_velocity_cmd_{3, 0.0};
        std::vector<float> wheel_voltage_cmd_{3, 0.0};

        // Zero motors is vel cmd is less than 1%
        if(fabs(linear_vel_cmd) > max_linear_vel_ * 0.01){
            Eigen::Vector2f xy_vel_dir = xy_vel_vector/xy_vel_vector.norm();
            Eigen::Vector2f icr

            // Pure translation for negligable angular velocity command (<1%)
            if(fabs(angular_vel_cmd) < max_angular_vel_ * 0.01){
                auto angle = atan2(linear_vel_dir.y(), linear_vel_dir.x());

            }

        }
        else{

        }

        auto motor_cmd_msg = ghost_msgs::msg::V5MotorCommand{};

        Eigen::Vector3f steering

        // Zero motor commands if linear velocity is less than 1% of max
        if(fabs(linear_vel_cmd) > (max_linear_vel_ * 0.01)){
            Eigen::Vector2f xy_vel_dir = xy_vel_vector/xy_vel_vector.norm();

            // Special case for pure translation (less than 1% of max angular velocity)
            if(fabs(angular_vel_cmd) < (max_angular_vel_ * 0.01)){
                
            }
            else{
                
            }

            Eigen::Vector2f desired_icr(-linear_vel_dir.y(), linear_vel_dir.x());
            desired_icr *= (linear_vel_cmd / angular_vel_cmd) * (max_angular_vel_ / max_linear_vel_);
        }

        // Convert steering and wheel commands to actuator space
        std::vector<float> motor_speed_cmds{8, 0.0};
        std::vector<float> motor_voltage_cmds{8, 0.0};

        publishMotorCommand(motor_speed_cmds, motor_voltager_cmds);
    }

    void RobotStateMachineNode::publishMotorCommand(std::vector<float> speed_cmd_array, std::vector<float> voltage_cmd_array){
        // Generate actuator command msg
        auto actuator_cmd_msg = ghost_msgs::msg::V5ActuatorCommand{};
        actuator_cmd_msg.header.stamp = get_clock()->now();
        actuator_cmd_msg.msg_id = curr_joystick_msg_id_;

        // Update velocity commands
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_LEFT_FRONT_MOTOR].desired_velocity   = voltage_cmd_array[0];
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_LEFT_BACK_MOTOR].desired_velocity    = voltage_cmd_array[1];
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_RIGHT_FRONT_MOTOR].desired_velocity  = voltage_cmd_array[2];
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_RIGHT_BACK_MOTOR].desired_velocity   = voltage_cmd_array[3];
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_BACK_RIGHT_1_MOTOR].desired_velocity = voltage_cmd_array[4];
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_BACK_RIGHT_2_MOTOR].desired_velocity = voltage_cmd_array[5];
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_BACK_LEFT_1_MOTOR].desired_velocity  = voltage_cmd_array[6];
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_BACK_LEFT_2_MOTOR].desired_velocity  = voltage_cmd_array[7];

        // Update voltage commands
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_LEFT_FRONT_MOTOR].desired_voltage    = speed_cmd_array[0];
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_LEFT_BACK_MOTOR].desired_voltage     = speed_cmd_array[1];
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_RIGHT_FRONT_MOTOR].desired_voltage   = speed_cmd_array[2];
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_RIGHT_BACK_MOTOR].desired_voltage    = speed_cmd_array[3];
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_BACK_RIGHT_1_MOTOR].desired_voltage  = speed_cmd_array[4];
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_BACK_RIGHT_2_MOTOR].desired_voltage  = speed_cmd_array[5];
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_BACK_LEFT_1_MOTOR].desired_voltage   = speed_cmd_array[6];
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_BACK_LEFT_2_MOTOR].desired_voltage   = speed_cmd_array[7];

        actuator_command_pub_->publish(actuator_cmd_msg);
    }

} // namespace ghost_ros

int main(int argc, char* argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ghost_ros::RobotStateMachineNode>());
  rclcpp::shutdown();
  return 0;
}