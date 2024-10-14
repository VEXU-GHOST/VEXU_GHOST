#include "diff_drive_ss.hpp"

namespace diff_drive
{
DiffDriveSS::DiffDriveSS()
{

}


void DiffDriveSS::VltgToVelocity(
  std::vector<ghost_msgs::msg::V5MotorCommand,
  std::allocator<ghost_msgs::msg::V5MotorCommand>> & mtr_cmds)
{
  Eigen::Vector2f v_in(mtr_cmds[0].voltage_command, mtr_cmds[1].voltage_command);
}


void DiffDriveSS::getA()
{


}
}
// namespace diff_drive
