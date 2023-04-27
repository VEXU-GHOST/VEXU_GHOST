#ifndef GHOST_ROS__ROBOT_STATE_MACHINE_NODE_HPP
#define GHOST_ROS__ROBOT_STATE_MACHINE_NODE_HPP

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "eigen3/Eigen/Geometry"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "ghost_msgs/msg/v5_competition_state.hpp"
#include "ghost_msgs/msg/v5_joystick.hpp"
#include "ghost_msgs/msg/ghost_robot_state.hpp"
#include "ghost_msgs/msg/v5_actuator_command.hpp"
#include "ghost_msgs/msg/v5_sensor_update.hpp"

namespace ghost_ros
{

    enum robot_state_e
    {
        DISABLED = 0,
        TELEOP = 1,
        AUTONOMOUS = 2,
    };

    enum teleop_mode_e
    {
        SHOOTER_MODE,
        INTAKE_MODE,
        EJECT_MODE,
        TILT_MODE,
        EXPANSION_MODE
    };

    class RobotStateMachineNode : public rclcpp::Node
    {
    public:
        RobotStateMachineNode();

    private:
        void updateController();
        void teleop();
        void resetPose();

        void updateSwerveCommandsFromTwist(float angular_velocity, float x_velocity, float y_velocity);
        void updateSwerveVoltageCommandsFromTwist(float angular_velocity, float x_velocity, float y_velocity);

        void publishSwerveKinematicsVisualization(
            const Eigen::Vector2f &left_wheel_cmd,
            const Eigen::Vector2f &right_wheel_cmd,
            const Eigen::Vector2f &back_wheel_cmd);

        // Publishers
        rclcpp::Publisher<ghost_msgs::msg::V5ActuatorCommand>::SharedPtr actuator_command_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_viz_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
        

        // Subscriptions
        rclcpp::Subscription<ghost_msgs::msg::V5CompetitionState>::SharedPtr competition_state_sub_;
        rclcpp::Subscription<ghost_msgs::msg::V5Joystick>::SharedPtr v5_joystick_sub_;
        rclcpp::Subscription<ghost_msgs::msg::GhostRobotState>::SharedPtr robot_state_sub_;
        rclcpp::Subscription<ghost_msgs::msg::V5SensorUpdate>::SharedPtr v5_sensor_update_sub_;

        // Robot States
        robot_state_e curr_robot_state_;
        std::chrono::time_point<std::chrono::system_clock> auton_start_time_;

        // Latest Msgs
        ghost_msgs::msg::V5Joystick::SharedPtr curr_joystick_msg_;
        ghost_msgs::msg::V5SensorUpdate::SharedPtr curr_encoder_msg_;
        ghost_msgs::msg::V5CompetitionState::SharedPtr curr_comp_state_msg_;
        ghost_msgs::msg::GhostRobotState::SharedPtr curr_robot_state_msg_;

        uint32_t curr_robot_state_msg_id_;
        uint32_t curr_encoder_msg_id_;
        uint32_t curr_joystick_msg_id_;
        uint32_t curr_comp_state_msg_id_;

        float max_linear_vel_;
        float max_angular_vel_;
        float max_steering_angular_vel_;

        float steering_kp_;
        float translate_kp_;
        float translate_kd_;
        float rotate_kp_;
        float rotate_kd_;
        float translation_slew_;
        float rotation_slew_;
        float translation_tolerance_;
        float max_motor_rpm_true_;

        float last_ang_vel_cmd_;
        Eigen::Vector2f last_xy_vel_cmd_;

        double pose_reset_x_;
        double pose_reset_y_;
        double pose_reset_theta_;

        float x_goal_;
        float y_goal_;

        Eigen::Vector2f left_wheel_pos_;
        Eigen::Vector2f right_wheel_pos_;
        Eigen::Vector2f back_wheel_pos_;

        ghost_msgs::msg::V5ActuatorCommand actuator_cmd_msg_;

        // Flywheel teleop modes
        bool r1_pressed_;
        teleop_mode_e teleop_mode;
    };

} // namespace ghost_ros
#endif // GHOST_ROS__ROBOT_STATE_MACHINE_NODE_HPP