// B_robot_subscriber.cpp

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ghost_ros_interfaces/v5_robot_base.hpp>

class TeamReceiver : public ghost_ros_interfaces::V5RobotBase
{
public:
  void initialize() override
  {
    sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "/team_drive",
      10,
      std::bind(&TeamReceiver::driveCallback, this, std::placeholders::_1));
  }

  void driveCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double forward = msg->linear.x;   // -1.0 to 1.0
    double turn    = msg->angular.z;  // -1.0 to 1.0

    double left  = forward - turn;
    double right = forward + turn;

    // Send to VEX motors through radio
    rhi_ptr_->setMotorVoltageCommandPercent("left_motor",  left);
    rhi_ptr_->setMotorVoltageCommandPercent("right_motor", right);

    rhi_ptr_->setMotorCurrentLimitMilliAmps("left_motor",  2500);
    rhi_ptr_->setMotorCurrentLimitMilliAmps("right_motor", 2500);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(TeamReceiver, ghost_ros_interfaces::V5RobotBase)