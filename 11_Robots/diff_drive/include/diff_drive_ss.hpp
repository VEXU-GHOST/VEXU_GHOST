#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "ghost_msgs/msg/v5_actuator_command.hpp"
#include "ghost_msgs/msg/diff_drive_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"

/*
Matrix dimensions are static! A is 5x5
*/


namespace diff_drive
{
class DiffDriveSS
{
public:
  DiffDriveSS();
  void getA();
  void VltgToVelocity(
    std::vector<ghost_msgs::msg::V5MotorCommand,
    std::allocator<ghost_msgs::msg::V5MotorCommand>> & mtr_cmds);


};


}
