#ifndef GHOST_ROS__ROBOT_STATE_MACHINE_NODE_HPP
#define GHOST_ROS__ROBOT_STATE_MACHINE_NODE_HPP

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "ghost_msgs/msg/v5_competition_state.hpp"
#include "ghost_msgs/msg/v5_joystick.hpp"
#include "ghost_msgs/msg/pose2_d.hpp"
#include "ghost_msgs/msg/robot_actuator_command.hpp"

namespace ghost_ros
{

    enum robot_state_e{
        DISABLED            = 0,
        TELEOP              = 1,
        AUTONOMOUS          = 2,
    };

    class RobotStateMachineNode : public rclcpp::Node
    {
    public:
        RobotStateMachineNode(std::string config_file);

    private:
        void competitionStateCallback(const ghost_msgs::msg::V5CompetitionState::SharedPtr msg);
        void v5JoystickCallback(const ghost_msgs::msg::V5Joystick::SharedPtr msg);
        void robotPoseCallback(const ghost_msgs::msg::Pose2D::SharedPtr msg);

        // Publishers
        rclcpp::Publisher<ghost_msgs::msg::RobotActuatorCommand>::SharedPtr actuator_command_pub_;

        // Subscriptions
        rclcpp::Subscription<ghost_msgs::msg::V5CompetitionState>::SharedPtr competition_state_sub_;
        rclcpp::Subscription<ghost_msgs::msg::V5Joystick>::SharedPtr v5_joystick_sub_;
        rclcpp::Subscription<ghost_msgs::msg::Pose2D>::SharedPtr robot_pose_sub_;

        // Robot States
        robot_state_e curr_robot_state_;
        float curr_x_;
        float curr_y_;
        float curr_theta_;

        // Last Command
        float des_x_vel_;
        float des_y_vel_;
        float des_theta_vel_;
    };

} // namespace ghost_ros
#endif // GHOST_ROS__ROBOT_STATE_MACHINE_NODE_HPP