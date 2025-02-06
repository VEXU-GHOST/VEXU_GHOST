//header file for PD Control class, to use angular and linear PD control.
#include <vector>
#include <iostream>
#include <eigen3/Eigen/Core>

namespace ghost_tank
{

class PDControl
{


public:
  //constructor
  PDControl();
  Eigen::Vector2d tank_pid(Eigen::Vector3d cur_pos, Eigen::Vector2d end_pos, float time);

private:
  float prev_time_;
  float prev_error_xy_;
  float prev_error_theta_;
};

} // namespace ghost_tank