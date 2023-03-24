#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ghost_msgs/msg/ghost_robot_state.hpp"

class SentryControl : public rclcpp::Node {
    public:

        SentryControl();

    private:
        rclcpp::Subscription<ghost_msgs::msg::GhostRobotState>::SharedPtr robot_pose_sub_; // /estimation/robot_pose [ghost_msgs/msg/GhostRobotState]
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_; // /goal_pose [geometry_msgs/msg/PoseStamped]
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_velocity_pub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

        Pose robot_pose;
        Pose goal_pose;
        float speed;

        void update_command_velocity();
        void update_goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_pose_msg);
        void update_robot_pose_callback(const ghost_msgs::msg::GhostRobotState::SharedPtr robot_pose_msg);
};