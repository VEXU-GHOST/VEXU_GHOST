#include "diff_drive_ss.hpp"

using std::placeholders::_1;


namespace diff_drive_ss
{

DiffDriveSS::DiffDriveSS()
: Node("diff_drive_ss")
{
// Declare parameters
  declare_parameter("n_dim", 5);
  n_dim_ = get_parameter("n_dim").as_int();

  declare_parameter("m_dim", 2);
  m_dim_ = get_parameter("m_dim").as_int();

// Subscribers
  mtr_vltg_sub_ = this->create_subscription<ghost_msgs::msg::V5ActuatorCommand>(
    "/v5_actuator_cmd",
    10,
    std::bind(&DiffDriveSS::MtrVltgCallback, this, _1));

// Publishers
  diff_drive_ss_pub_ = this->create_publisher<ghost_msgs::msg::DiffDriveState>(
    "/diff_drive_ss", 10);

}

void DiffDriveSS::MtrVltgCallback(const ghost_msgs::msg::V5ActuatorCommand::SharedPtr msg)
{
  // check if number of input motor voltages match mdim
  if (msg->motor_commands.size() == m_dim_) {

  } else {
    RCLCPP_WARN(
      this->get_logger(), "The number of motor commands %ld does not match the number of specified control inputs %ld",
      msg->motor_commands.size(), m_dim_);
  }
}

} // namespace diff_drive_ss

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<diff_drive_ss::DiffDriveSS>());
  rclcpp::shutdown();
  return 0;
}
