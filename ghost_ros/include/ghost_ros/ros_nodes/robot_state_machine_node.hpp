#ifndef GHOST_ROS__ROBOT_STATE_MACHINE_NODE_HPP
#define GHOST_ROS__ROBOT_STATE_MACHINE_NODE_HPP

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "ghost_msgs/msg/v5_competition_state.hpp"
#include "ghost_msgs/msg/v5_joystick.hpp"
#include "ghost_msgs/msg/ghost_robot_state.hpp"
#include "ghost_msgs/msg/v5_actuator_command.hpp"

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
        void updateController();
        void teleop();
        // Publishers
        rclcpp::Publisher<ghost_msgs::msg::V5ActuatorCommand>::SharedPtr actuator_command_pub_;

        // Subscriptions
        rclcpp::Subscription<ghost_msgs::msg::V5CompetitionState>::SharedPtr competition_state_sub_;
        rclcpp::Subscription<ghost_msgs::msg::V5Joystick>::SharedPtr v5_joystick_sub_;
        rclcpp::Subscription<ghost_msgs::msg::GhostRobotState>::SharedPtr robot_pose_sub_;

        // Robot States
        robot_state_e curr_robot_state_;
        std::chrono::time_point<std::chrono::system_clock> auton_start_time_;

        // Latest Msgs
        ghost_msgs::msg::V5Joystick::SharedPtr curr_joystick_msg_;
        ghost_msgs::msg::V5CompetitionState::SharedPtr curr_comp_state_msg_;
        ghost_msgs::msg::GhostRobotState::SharedPtr curr_pose_msg_;

        uint32_t curr_pose_msg_id_;
        uint32_t curr_joystick_msg_id_;
        uint32_t curr_comp_state_msg_id_;
    };

} // namespace ghost_ros
#endif // GHOST_ROS__ROBOT_STATE_MACHINE_NODE_HPP