#include "ghost_tank/trapezoid_motion_planner.hpp"

namespace ghost_tank
{

using ghost_ros_interfaces::msg_helpers::toROSMsg;
using std::placeholders::_1;

void TrapezoidMotionPlanner::initialize()
{
  RCLCPP_INFO(node_ptr_->get_logger(), "initializing");

  node_ptr_->declare_parameter("acceleration", 1.0);
  acceleration_ = node_ptr_->get_parameter("acceleration").as_double();

  node_ptr_->declare_parameter("badCase_timeout", 5.0);
  badCase_timeout_ = node_ptr_->get_parameter("badCase_timeout").as_double();

  final_time_ = 1.0;

  RCLCPP_INFO(node_ptr_->get_logger(), "accel: %f", acceleration_);
  RCLCPP_INFO(node_ptr_->get_logger(), "timeout: %f", badCase_timeout_);
}


void TrapezoidMotionPlanner::generateMotionPlan(
  const ghost_msgs::msg::DrivetrainCommand::SharedPtr cmd)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Generating tank Motion Plan");

  double theta_f = ghost_util::quaternionToYawRad(
    cmd->pose.pose.orientation.w,
    cmd->pose.pose.orientation.x,
    cmd->pose.pose.orientation.y,
    cmd->pose.pose.orientation.z);

  // find position/velocities
  std::vector<double> xpos0({current_x_, current_x_vel_});
  std::vector<double> xposf({cmd->pose.pose.position.x, cmd->twist.twist.linear.x});
  std::vector<double> ypos0({current_y_, current_y_vel_});
  std::vector<double> yposf({cmd->pose.pose.position.y, cmd->twist.twist.linear.y});
  std::vector<double> thetapos0({current_theta_rad_, 0.0});
  std::vector<double> thetaposf({current_theta_rad_ +
      ghost_util::SmallestAngleDistRad(theta_f, current_theta_rad_), 0.0});
  double pos_threshold = cmd->pose.pose.position.z;
  double theta_threshold = cmd->twist.twist.angular.x;

  RCLCPP_INFO(node_ptr_->get_logger(), "current x: %f, current x_vel: %f", xpos0[0], xpos0[1]);
  RCLCPP_INFO(node_ptr_->get_logger(), "current y: %f, current x_vel: %f", ypos0[0], ypos0[1]);
  RCLCPP_INFO(
    node_ptr_->get_logger(), "current theta: %f, current theta_vel: %f", thetapos0[0],
    thetapos0[1]);
  RCLCPP_INFO(node_ptr_->get_logger(), "final x: %f, final x_vel: %f", xposf[0], xposf[1]);
  RCLCPP_INFO(node_ptr_->get_logger(), "final y: %f, final x_vel: %f", yposf[0], yposf[1]);
  RCLCPP_INFO(
    node_ptr_->get_logger(), "final theta: %f, final theta_vel: %f", thetaposf[0], thetaposf[1]);

  // find final time
  double v_max = cmd->speed;
  auto dist_vector = Eigen::Vector2d(xposf[0] - xpos0[0], yposf[0] - ypos0[0]);
  auto vel_vector0 = Eigen::Vector2d(xpos0[1], ypos0[1]);
  auto vel_vectorf = Eigen::Vector2d(xposf[1], yposf[1]);
  double dist = dist_vector.norm();
  double vel0 = vel_vector0.norm();
  double velf = vel_vectorf.norm();
  double vel_direction = atan2(dist_vector.y(), dist_vector.x());

  std::vector<double> pos0({0.0, vel0});
  std::vector<double> posf({dist, velf});
  auto trajectory = std::make_shared<RobotTrajectory>();
  auto pos_trajectory = getTrapezoidTraj(acceleration_, v_max, xpos0, xposf);

  for (int i = 0; i < pos_trajectory.time_vector.size(); i++) {
    trajectory->x_trajectory.position_vector.push_back(pos_trajectory.position_vector[i] *
      cos(vel_direction) + xpos0[0]);
    trajectory->x_trajectory.velocity_vector.push_back(pos_trajectory.velocity_vector[i] *
      cos(vel_direction));
    trajectory->y_trajectory.position_vector.push_back(pos_trajectory.position_vector[i] *
      sin(vel_direction) + ypos0[0]);
    trajectory->y_trajectory.velocity_vector.push_back(pos_trajectory.velocity_vector[i] *
      sin(vel_direction));
  }
  trajectory->x_trajectory.time_vector = pos_trajectory.time_vector;
  trajectory->x_trajectory.threshold = pos_threshold;
  trajectory->y_trajectory.time_vector = pos_trajectory.time_vector;
  trajectory->y_trajectory.threshold = pos_threshold;

  if (ghost_util::SmallestAngleDistRad(theta_f, current_theta_rad_) < 0.0) {
    thetapos0[0] = 0.0;
    thetaposf[0] = -ghost_util::SmallestAngleDistRad(theta_f, current_theta_rad_);
    trajectory->theta_trajectory = getTrapezoidTraj(acceleration_, v_max, thetapos0, thetaposf);
    for (int i = 0; i < trajectory->theta_trajectory.time_vector.size(); i++) {
      double pos = trajectory->theta_trajectory.position_vector[i];
      trajectory->theta_trajectory.position_vector[i] = current_theta_rad_ - pos;
      trajectory->theta_trajectory.velocity_vector[i] *= -1;
    }
  } else {
    trajectory->theta_trajectory = getTrapezoidTraj(acceleration_, v_max, thetapos0, thetaposf);
  }
  trajectory->theta_trajectory.threshold = theta_threshold;

  ghost_msgs::msg::RobotTrajectory trajectory_msg;

  toROSMsg(*trajectory, trajectory_msg);

  RCLCPP_INFO(node_ptr_->get_logger(), "Generated tank Motion Plan");
  trajectory_pub_->publish(trajectory_msg);
}

void TrapezoidMotionPlanner::computeTrapezoidFunction(double accel, double v_max,
  std::vector<double> vec_q0,
  std::vector<double> vec_qf)
{
  bool bad_case = false;
  std::string error = "";
  double pos_initial = vec_q0[0];
  double pos_final = vec_qf[0];
  double vel_initial = vec_q0[1];
  double vel_final = vec_qf[1];

  if (accel <= 0) {
    error = "acceleration";
    bad_case = true;
  } else if (v_max <= 0) {
    error = "v_max";
    bad_case = true;
  } else if (pos_final - pos_initial < 0) {
    error = "position";
    bad_case = true;
  } else if (vel_final > v_max) {
    // error = "vel_final";
    // bad_case = true;
    vel_final = v_max;
  }
  if (bad_case) {
    badCase(pos_final, vel_final, error);
  }
  // i think this case is handled below
  // if((vel_final - vel_initial) / accel < 1002131){ //diff in velocity is too high?
  //    vel_final = v_max;
  // }

  double accel_initial = (v_max - vel_initial > 0.0) ? accel : -accel;
  double accel_final = (vel_final - v_max > 0.0) ? accel : -accel;
  double dist = pos_final - pos_initial;
  double time_accel_initial = abs(v_max - vel_initial) / accel;
  double time_accel_final = abs(v_max - vel_final) / accel;
  double time_1 = time_accel_initial;
  double time_2 =
    ((v_max - vel_initial) * time_accel_initial / 2 - (vel_final + v_max) * time_accel_final / 2 +
    dist) / v_max;
  double time_3 = time_2 + time_accel_final;

  if (time_2 < time_1) {    // doesnt reach max velocity
    if (abs(time_accel_final) <= 0.001) {          // divide by zero
      computeLinearFunction(pos_initial, pos_final, vel_initial, vel_final, accel_initial);
    } else {
      computeTriangleFunction(pos_initial, pos_final, vel_initial, vel_final, accel_initial,
        accel_final);
    }
  } else {
    // coefficients for vel
    double va1 = accel_initial;
    double vb1 = vel_initial;
    double va2 = 0.0;
    double vb2 = v_max;
    double va3 = accel_final;
    double vb3 = -accel_final * time_2 + v_max;

    // coefficients for pos
    double pa1 = accel_initial / 2.0;
    double pb1 = vel_initial;
    double pc1 = pos_initial;
    double pa2 = 0.0;
    double pb2 = v_max;
    double pc2 = pos_initial + (vel_initial - v_max) * time_1 / 2.0;
    double pa3 = accel_final / 2.0;
    double pb3 = v_max - (accel_final * time_2);
    double pc3 = accel_final * time_2 * time_2 / 2.0 + (vel_initial - v_max) * time_1 / 2.0 +
      pos_initial;

    trapezoidVelocityFunc_ =
      [time_1, time_2, time_3, va1, vb1, va2, vb2, va3, vb3, vel_final](double time) {
        if (time <= time_1) {
          return va1 * time +
                 vb1;
        } else if (time <= time_2) {
          return va2 * time +
                 vb2;
        } else if (time <= time_3) {
          return va3 * time +
                 vb3;
        }
        return vel_final;
      };
    trapezoidPositionFunc_ =
      [time_1, time_2, time_3, pa1, pb1, pc1, pa2, pb2, pc2, pa3, pb3, pc3,
        pos_final](double time) {
        if (time <= time_1) {
          return pa1 * time * time +
                 pb1 * time +
                 pc1;
        } else if (time <= time_2) {
          return pa2 * time * time +
                 pb2 * time +
                 pc2;
        } else if (time <= time_3) {
          return pa3 * time * time +
                 pb3 * time +
                 pc3;
        }
        return pos_final;
      };
    final_time_ = time_3;
  }
}

void TrapezoidMotionPlanner::computeTriangleFunction(
  double pos_initial, double pos_final,
  double vel_initial, double vel_final,
  double accel_initial, double accel_final)
{
  double dist = pos_final - pos_initial;
  double vel_diff = (vel_final - vel_initial);
  double accel_diff = (accel_final - accel_initial);
  double a = accel_diff / 2.0 - accel_diff * accel_diff / (2.0 * accel_final);
  double b = vel_initial * accel_diff / accel_final;
  double c = vel_diff * vel_diff / (2.0 * accel_final) + vel_initial * vel_diff / accel_final -
    dist;
  double determinant = b * b - 4.0 * a * c;
  if (determinant < 0.0) {
    computeLinearFunction(pos_initial, pos_final, vel_initial, vel_final, accel_initial);
  } else {
    // time points
    double time_1 = (-b + sqrt(determinant)) / (2.0 * a);
    double time_2 = (vel_diff + accel_diff * time_1) / accel_final;

    // coefficients for vel
    double va1 = accel_initial;
    double vb1 = vel_initial;
    double va2 = accel_final;
    double vb2 = (accel_initial - accel_final) * time_1 + vel_initial;

    // coefficients for pos
    double pa1 = accel_initial / 2.0;
    double pb1 = vel_initial;
    double pc1 = pos_initial;
    double pa2 = accel_final / 2.0;
    double pb2 = vel_initial - (accel_final - accel_initial) * time_1;
    double pc2 = pos_initial + (accel_final - accel_initial) * time_1 * time_1 / 2.0;

    trapezoidVelocityFunc_ = [time_1, time_2, va1, vb1, va2, vb2, vel_final](double time) {
        if (time <= time_1) {
          return va1 * time +
                 vb1;
        } else if (time <= time_2) {
          return va2 * time +
                 vb2;
        }
        return vel_final;
      };
    trapezoidPositionFunc_ =
      [time_1, time_2, pa1, pb1, pc1, pa2, pb2, pc2, pos_final](double time) {
        if (time <= time_1) {
          return pa1 * time * time +
                 pb1 * time +
                 pc1;
        } else if (time <= time_2) {
          return pa2 * time * time +
                 pb2 * time +
                 pc2;
        }
        return pos_final;
      };
    final_time_ = time_2;
  }
}

void TrapezoidMotionPlanner::computeLinearFunction(
  double pos_initial, double pos_final,
  double vel_initial, double vel_final,
  double accel_initial)
{
  double dist = pos_final - pos_initial;
  double a = accel_initial / 2.0;
  double b = vel_initial;
  double c = -dist;
  double determinant = b * b - 4.0 * a * c;
  if (determinant < 0.0) {
    badCase(pos_final, vel_final, "invalid motion");
  } else {
    // velocity coeff
    double va = accel_initial;
    double vb = vel_initial;

    // position coeff
    double pa = accel_initial / 2.0;
    double pb = vel_initial;
    double pc = pos_initial;

    // time points
    double time_final = (-b + sqrt(determinant)) / (2.0 * a);

    trapezoidVelocityFunc_ = [time_final, va, vb, vel_final](double time) {
        if (time <= time_final) {
          return va * time + vb;
        }
        return vel_final;
      };
    trapezoidPositionFunc_ = [time_final, pa, pb, pc, pos_final](double time) {
        if (time <= time_final) {
          return pa * time * time +
                 pb * time +
                 pc;
        }
        return pos_final;
      };
    final_time_ = time_final;
  }
}

void TrapezoidMotionPlanner::badCase(double pos_final, double vel_final, std::string error)
{
  RCLCPP_WARN(node_ptr_->get_logger(), "Warning: Invalid Trajectory Parameters: %s", error.c_str());
  trapezoidVelocityFunc_ = [vel_final](double time) {
      return vel_final;
    };
  trapezoidPositionFunc_ = [pos_final](double time) {
      return pos_final;
    };
  final_time_ = badCase_timeout_;
}

RobotTrajectory::Trajectory TrapezoidMotionPlanner::getTrapezoidTraj(
  double accel, double v_max,
  std::vector<double> vec_q0,
  std::vector<double> vec_qf)
{
  computeTrapezoidFunction(accel, v_max, vec_q0, vec_qf);

  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> time;
  double step = 0.01;
  for (double t = 0; t < final_time_; t += step) {
    position.push_back(trapezoidPositionFunc_(t));
    velocity.push_back(trapezoidVelocityFunc_(t));
    time.push_back(t);
  }
  auto trajectory = RobotTrajectory::Trajectory();
  trajectory.position_vector = position;
  trajectory.velocity_vector = velocity;
  trajectory.time_vector = time;
  return trajectory;
}

} // namespace ghost_tank

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto trapezoid_motion_planner = std::make_shared<ghost_tank::TrapezoidMotionPlanner>();
  trapezoid_motion_planner->configure("trapezoid_motion_planner");
  auto trapezoid_motion_planner_node = trapezoid_motion_planner->getROSNodePtr();
  rclcpp::spin(trapezoid_motion_planner_node);
  rclcpp::shutdown();
  return 0;
}
