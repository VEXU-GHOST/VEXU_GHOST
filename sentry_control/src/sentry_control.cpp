#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "ghost_msgs/msg/ghost_robot_state.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;
using ghost_msgs::msg::GhostRobotState;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Quaternion;
using geometry_msgs::msg::Vector3;

class SentryControl : public rclcpp::Node {
  public:
    SentryControl() : Node("sentry_control") {

      robot_pose_sub_ = this->create_subscription<ghost_msgs::msg::GhostRobotState>(
        "estimation/robot_pose", 
        10, 
        std::bind(&SentryControl::update_robot_pose_callback, this, _1));

      goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 
        10, 
        std::bind(&SentryControl::update_goal_pose_callback, this, _1));

      command_velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

      odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    }

    private:
      Pose robot_pose;
      Pose goal_pose;
      float speed = 1;

      rclcpp::Subscription<ghost_msgs::msg::GhostRobotState>::SharedPtr robot_pose_sub_;
      rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_velocity_pub_;
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

      void update_command_velocity()
      {
        Vector3 desired_velocity;
        desired_velocity.x = goal_pose.position.x - robot_pose.position.x;
        desired_velocity.y = goal_pose.position.y - robot_pose.position.y;
        desired_velocity.z = goal_pose.position.z - robot_pose.position.z;

        float magnitude = sqrt(
          (desired_velocity.x * desired_velocity.x) +
          (desired_velocity.y * desired_velocity.y) +
          (desired_velocity.z * desired_velocity.z)
        );
        // normalize velocity
        desired_velocity.x = speed * desired_velocity.x / magnitude;
        desired_velocity.y = speed * desired_velocity.y / magnitude;
        desired_velocity.z = speed * desired_velocity.z / magnitude;

        Twist desired_twist;
        desired_twist.linear = desired_velocity;

        command_velocity_pub_->publish(desired_twist);
      }

      void update_goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_pose_msg)
      {
        goal_pose = goal_pose_msg->pose;
        update_command_velocity();
      }

      void update_robot_pose_callback(const ghost_msgs::msg::GhostRobotState::SharedPtr robot_pose_msg)
      {
        Point robot_point;
        robot_point.x = robot_pose_msg->x;
        robot_point.y = robot_pose_msg->y;

        Quaternion robot_rotation;
        robot_rotation.z = sin((robot_pose_msg->theta)/2);
        robot_rotation.w = cos((robot_pose_msg->theta)/2);

        robot_pose.position = robot_point;
        robot_pose.orientation = robot_rotation;

        update_command_velocity();
      }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SentryControl>());
    rclcpp::shutdown();
    return 0;
}