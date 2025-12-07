// A_robot_publisher.cpp

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ghost_ros_interfaces/v5_robot_base.hpp>

class TeamTransmitter : public ghost_ros_interfaces::V5RobotBase
{
public:
  void teleop(double current_time) override
  {
    static rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;

    if (!pub)
    {
      pub = this->node_->create_publisher<geometry_msgs::msg::Twist>("/team_drive", 10);
    }

    auto joy = rhi_ptr_->getMainJoystickData();

    geometry_msgs::msg::Twist msg;

    // Convert joystick values (-127 to 127) to -1.0 to 1.0
    msg.linear.x  = joy->left_y  / 127.0;
    msg.angular.z = joy->right_x / 127.0;

    pub->publish(msg);
  }
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(TeamTransmitter, ghost_ros_interfaces::V5RobotBase)