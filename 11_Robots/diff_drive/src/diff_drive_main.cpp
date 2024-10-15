#include "diff_drive_main.hpp"

using std::placeholders::_1;

namespace diff_drive
{

DiffDriveNode::DiffDriveNode()
: Node("diff_drive_node")
{
  ss = std::make_unique<diff_drive::DiffDriveSS>();
  // Conversion
  const float IN_TO_M = 1 / 39.37;

  // Declare parameters
  declare_parameter("n_dim", 5);
  n_dim_ = get_parameter("n_dim").as_int();

  declare_parameter("m_dim", 2);
  m_dim_ = get_parameter("m_dim").as_int();

  declare_parameter("r_wheel", 0);
  r_wheel_ = get_parameter("r_wheel").as_double();

  // Initialize A_ and B_ size
  this->A_(n_dim_, n_dim_);
  this->B_(n_dim_, m_dim_);
  this->init_state_(n_dim_);
  this->init_state_ << 0.0, 0.0, 0.0, 1.0, 1.0;
  this->state_(n_dim_);


// Subscribers
  mtr_vltg_sub_ = this->create_subscription<ghost_msgs::msg::V5ActuatorCommand>(
    "/v5_actuator_cmd",
    10,
    std::bind(&DiffDriveNode::MtrVltgCallback, this, _1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom",
    10,
    std::bind(&DiffDriveNode::OdomCallback, this, _1));

// Publishers
  diff_drive_ss_pub_ = this->create_publisher<ghost_msgs::msg::DiffDriveState>(
    "/diff_drive_ss", 10);

}

// Callback when map ekf publishes odom
void DiffDriveNode::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  auto odom_msg = *msg;
  double x = odom_msg.pose.pose.position.x;
  double y = odom_msg.pose.pose.position.y;
  double z = odom_msg.pose.pose.position.z;

  Eigen::Vector2d v_map = Eigen::Vector2d(
    odom_msg.twist.twist.linear.x,
    odom_msg.twist.twist.linear.y);
  // Get rotation from map to base frame
  double tht = ghost_util::quaternionToYawRad(
    odom_msg.pose.pose.orientation.w,
    odom_msg.pose.pose.orientation.x,
    odom_msg.pose.pose.orientation.y,
    odom_msg.pose.pose.orientation.z);
  auto map_to_base_R = Eigen::Rotation2D<double>(-tht).toRotationMatrix();

  // Convert map velocity to base left and right wheel velocity
  Eigen::VectorXd v_b = map_to_base_R * v_map;

  this->state_(0) = x;
  this->state_(1) = y;
  this->state_(2) = z;
}


// Callback for input motor voltage command
void DiffDriveNode::MtrVltgCallback(const ghost_msgs::msg::V5ActuatorCommand::SharedPtr msg)
{
  // check if number of input motor voltages match mdim
  if (msg->motor_commands.size() == m_dim_) {
    ss->VltgToVelocity(msg->motor_commands);
  } else {
    RCLCPP_WARN(
      this->get_logger(), "The number of motor commands %ld does not match the number of specified control inputs %ld",
      msg->motor_commands.size(), m_dim_);
  }
}


} // namespace diff_drive


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<diff_drive::DiffDriveNode>());
  rclcpp::shutdown();
  return 0;
}
