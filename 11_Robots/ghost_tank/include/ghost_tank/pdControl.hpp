//header file for PD Control class, to use angular and linear PD control.
#include <vector>
#include <iostream>
#include <../include/ghost_tank/tank_model.hpp>

class pdControl
{
private:
  ;
  double prev_time;
  double cur_time;
  double delta_time = prev_time - cur_time;

  std::vector<std::string> motor_list = {
    "drive_ltr",
    "drive_lbr",
    "drive_ltf",
    "drive_lbf",
    "drive_lttf",
    "indexer_right",
    "indexer_left",
    "drive_rttf",
    "drive_rtr",
    "drive_rbr",
    "drive_rtf",
    "drive_rbf"
  };

public:
  ;

//constructor
  pdControl(float time, std::shared_ptr<ghost_tank::TankModel> m_tank_model_ptr);

  void linear_pid(float st_x, float st_y, float end_x, float end_y);
  void angular_pid(float st_theta, float end_theta);


};
