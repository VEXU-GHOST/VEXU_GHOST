#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "ghost_msgs/msg/v5_actuator_command.hpp"
#include "ghost_msgs/msg/diff_drive_state.hpp"
#include "geometry_msgs/msg/pose.hpp"


namespace diff_drive_ss
{

class DiffDriveSS : public rclcpp::Node
{
public:
  DiffDriveSS();

  // Subscribers
  rclcpp::Subscription<ghost_msgs::msg::V5ActuatorCommand>::SharedPtr mtr_vltg_sub_;


  // Publishers
  rclcpp::Publisher<ghost_msgs::msg::DiffDriveState>::SharedPtr diff_drive_ss_pub_;

  void MtrVltgCallback(const ghost_msgs::msg::V5ActuatorCommand::SharedPtr msg);

private:
  unsigned long int n_dim_;
  unsigned long int m_dim_;


};

}
// namespace diff_drive_ss
