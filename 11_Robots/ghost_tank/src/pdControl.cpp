#include <iostream>
#include <../include/ghost_tank/tank_model.hpp>
#include <../include/tank_robot_plugin.hpp>
#include <ghost_util/angle_util.hpp>
#include <ghost_util/unit_conversion_utils.hpp>
#include <../include/ghost_tank/tank_model.hpp>
//#include <pluginlib/class_list_macros.hpp>

#include "ghost_tank/pdControl.hpp";

#include <cmath>


pdControl::pdControl(double time, std::shared_ptr<ghost_tank::TankModel> m_tank_model_ptr)
{
  this->cur_time = time;

  //initiallize current for all motors
  for (const auto motor_name: motor_list) {
    rhi_ptr_->setMotorCurrentLimitMilliAmps(motor_name, 2500);
  }
}

void pdControl::linear_pid(float end_x, float end_y)
{
  m_tank_model_ptr = std::make_shared<TankModel>(node_ptr_, rhi_ptr_, tank_model_config);
  auto cur_pos = m_tank_model_ptr->getWorldPose();

  float error_prior_x = end_x - cur_pos.x; //in meters
  float error_prior_y = end_y - cur_pos.y; //in meters

  float hyp = sqrt(pow(error_prior_x, 2) + pow(error_prior_y, 2));

  float error_prior = hyp;
  float Ce = 1; //Error Constant
  float Cd = 1; //Derivative Constant
  float bias = 0;
  float derivative;

  float error;
  while (error < 0.1) {
    error = sqrt(pow(end_x - cur_pos.x, 2) + pow(end_y - cur_pos.y, 2));
    derivative = (error_prior - error) / this->prev_time;
    float output = Ce * error + Cd * derivative + bias;
    if (output > 1.00) {
      output = 1.00;
    } else if (output < 0.00) {
      output = 0.00;
    }
    driveCommand(output, 0);
    error_prior = error;
  }

  void pdControl::angular_pid(float end_theta)
  {
    m_tank_model_ptr = std::make_shared<TankModel>(node_ptr_, rhi_ptr_, tank_model_config);
    auto cur_pos = m_tank_model_ptr->getWorldPose();
    float error_prior = end_theta - cur_pos.z; //in radians
    float max_error = error_prior;

    float Ce = 1; //Error Constant
    float Cd = 1; //Derivative Constant
    float bias = 0;
    float derivative;

    float error;

    int dir;
    if (cur_pos.z - end_theta >= end_theta - cur_pos.z) {
      dir = 1;
    } else {
      dir = -1;
    }

    while (error < 0.1) {
      error = end_theta - cur_pos.z;
      derivative = (error_prior - error) / delta_time;
      float output = Ce * max_error + Cd * derivative + bias;
      if (output > 1.00) {
        output = 1.00;
      } else if (output < 0.00) {
        output = 0.00;
      }
      driveCommand(0, dir * output);
      error_prior = error;
    }

  }

}
