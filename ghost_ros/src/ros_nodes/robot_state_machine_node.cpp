#include <chrono>
#include <cmath>
#include <stdexcept>

#include "eigen3/Eigen/Geometry"

#include "math/math_util.h"
#include "ghost_util/angle_util.hpp"

#include "ghost_ros/robot_config/v5_port_config.hpp"
#include "ghost_ros/robot_config/v5_serial_msg_config.hpp"
#include "ghost_ros/ros_nodes/robot_state_machine_node.hpp"


using namespace std::chrono_literals;
using namespace math_util;
using std::placeholders::_1;

namespace ghost_ros
{

    RobotStateMachineNode::RobotStateMachineNode() : rclcpp::Node("ghost_state_machine"),
                                                     curr_robot_state_{robot_state_e::DISABLED},
                                                     curr_robot_state_msg_{0},
                                                     curr_joystick_msg_id_{0},
                                                     curr_comp_state_msg_id_{0},
                                                     r1_pressed_{false},
                                                     teleop_mode{INTAKE_MODE},
                                                     last_ang_vel_cmd_{0.0}
                                                     
    {
        declare_parameter("max_linear_vel", 0.0);
        max_linear_vel_ = get_parameter("max_linear_vel").as_double();

        declare_parameter("left_wheel_x", 0.0);
        declare_parameter("left_wheel_y", 0.0);
        declare_parameter("right_wheel_x", 0.0);
        declare_parameter("right_wheel_y", 0.0);
        declare_parameter("back_wheel_x", 0.0);
        declare_parameter("back_wheel_y", 0.0);

        left_wheel_pos_ = Eigen::Vector2f(get_parameter("left_wheel_x").as_double(), get_parameter("left_wheel_y").as_double());
        right_wheel_pos_ = Eigen::Vector2f(get_parameter("right_wheel_x").as_double(), get_parameter("right_wheel_y").as_double());
        back_wheel_pos_ = Eigen::Vector2f(get_parameter("back_wheel_x").as_double(), get_parameter("back_wheel_y").as_double());

        declare_parameter("steering_kp", 0.0);
        steering_kp_ = get_parameter("steering_kp").as_double();

        declare_parameter("translate_kp", 0.0);
        declare_parameter("translate_kd", 0.0);
        declare_parameter("rotate_kp", 0.0);
        declare_parameter("rotate_kd", 0.0);
        translate_kp_ = get_parameter("translate_kp").as_double();
        translate_kd_ = get_parameter("translate_kd").as_double();
        rotate_kp_ = get_parameter("rotate_kp").as_double();
        rotate_kd_ = get_parameter("rotate_kd").as_double();

        declare_parameter("translation_slew", 0.0);
        translation_slew_ = get_parameter("translation_slew").as_double();

        declare_parameter("rotation_slew", 0.0);
        rotation_slew_ = get_parameter("rotation_slew").as_double();

        declare_parameter("translation_tolerance", 0.0);
        translation_tolerance_ = get_parameter("translation_tolerance").as_double();

        declare_parameter("x_goal", 0.0);
        declare_parameter("y_goal", 0.0);
        x_goal_ = get_parameter("x_goal").as_double();
        y_goal_ = get_parameter("y_goal").as_double();

        declare_parameter("max_motor_rpm_true", 0.0);
        max_motor_rpm_true_ = get_parameter("max_motor_rpm_true").as_double();

        if (fabs(back_wheel_pos_.norm()) != 0.0)
        {
            max_angular_vel_ = max_linear_vel_ / back_wheel_pos_.norm();
        }
        else
        {
            throw std::invalid_argument("Wheel Geometry has not been set.");
        }

        declare_parameter("pose_reset_x", 0.0);
        declare_parameter("pose_reset_y", 0.0);
        declare_parameter("pose_reset_theta", 0.0);

        pose_reset_x_ = get_parameter("pose_reset_x").as_double();
        pose_reset_y_ = get_parameter("pose_reset_y").as_double();
        pose_reset_theta_ = get_parameter("pose_reset_theta").as_double();

        last_xy_vel_cmd_ = Eigen::Vector2f(0.0, 0.0);

        actuator_command_pub_ = create_publisher<ghost_msgs::msg::V5ActuatorCommand>(
            "v5/actuator_commands",
            10);
        debug_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "controller_debug",
            10);

        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initial_pose",10);

        competition_state_sub_ = create_subscription<ghost_msgs::msg::V5CompetitionState>(
            "v5/competition_state",
            10,
            [this](const ghost_msgs::msg::V5CompetitionState::SharedPtr msg)
            {
                curr_comp_state_msg_id_ = msg->msg_id;
                curr_comp_state_msg_ = msg;

                if (msg->is_autonomous && curr_robot_state_ != robot_state_e::AUTONOMOUS)
                {
                    curr_robot_state_ = robot_state_e::AUTONOMOUS;
                    auton_start_time_ = std::chrono::system_clock::now();
                }
                else if (msg->is_disabled)
                {
                    curr_robot_state_ = robot_state_e::DISABLED;
                }
                else if (!msg->is_autonomous && !msg->is_disabled)
                {
                    curr_robot_state_ = robot_state_e::TELEOP;
                }

                updateController();
            });

        v5_joystick_sub_ = create_subscription<ghost_msgs::msg::V5Joystick>(
            "v5/joystick",
            10,
            [this](const ghost_msgs::msg::V5Joystick::SharedPtr msg)
            {
                curr_joystick_msg_id_ = msg->msg_id;
                curr_joystick_msg_ = msg;
                updateController();
            });

        v5_sensor_update_sub_ = create_subscription<ghost_msgs::msg::V5SensorUpdate>(
            "v5/sensor_update",
            10,
            [this](const ghost_msgs::msg::V5SensorUpdate::SharedPtr msg)
            {
                curr_encoder_msg_id_ = msg->msg_id;
                curr_encoder_msg_ = msg;
                updateController();
            });

        robot_state_sub_ = create_subscription<ghost_msgs::msg::GhostRobotState>(
            "/estimation/robot_state",
            10,
            [this](const ghost_msgs::msg::GhostRobotState::SharedPtr msg)
            {
                curr_robot_state_msg_id_ = msg->msg_id;
                curr_robot_state_msg_ = msg;
                updateController();
            });
    }

    void RobotStateMachineNode::updateController()
    {
        // Comparing msg ids ensures only one control update is performed for a given sensor update
        if (
            (curr_joystick_msg_id_ == curr_comp_state_msg_id_) &&
            (curr_joystick_msg_id_ == curr_encoder_msg_id_) &&
            (curr_joystick_msg_id_ == curr_robot_state_msg_id_))
        {
            // Clear actuator command msg
            actuator_cmd_msg_ = ghost_msgs::msg::V5ActuatorCommand{};

            // Calculate robot actuator command dependent on competition state
            switch (curr_robot_state_)
            {
            case robot_state_e::AUTONOMOUS:

                break;

            case robot_state_e::TELEOP:
                teleop();
                break;

            case robot_state_e::DISABLED:

                break;
            }

            // Publish actuator command
            actuator_cmd_msg_.header.stamp = get_clock()->now();
            actuator_cmd_msg_.msg_id = curr_joystick_msg_id_;
            actuator_command_pub_->publish(actuator_cmd_msg_);
        }
    }

    void RobotStateMachineNode::resetPose(){
        geometry_msgs::msg::PoseWithCovarianceStamped msg{};

        msg.header.frame_id = "world";
        msg.header.stamp = this->get_clock()->now();

        msg.pose.pose.position.x = pose_reset_x_;
        msg.pose.pose.position.y = pose_reset_y_;
        msg.pose.pose.orientation.w = cos(pose_reset_theta_ * M_PI/180.0 * 0.5);
        msg.pose.pose.orientation.z = sin(pose_reset_theta_ * M_PI/180.0 * 0.5);

        initial_pose_pub_->publish(msg);
    }

    void RobotStateMachineNode::teleop()
    {
        updateSwerveCommandsFromTwist(
            -curr_joystick_msg_->joystick_right_x,
            curr_joystick_msg_->joystick_left_y,
            -curr_joystick_msg_->joystick_left_x);

        // Intake Control
        if(curr_joystick_msg_->joystick_btn_r2){
            float intake_cmd = 750;
            actuator_cmd_msg_.motor_commands[ghost_v5_config::INTAKE_MOTOR_1].desired_velocity = intake_cmd;
            actuator_cmd_msg_.motor_commands[ghost_v5_config::INTAKE_MOTOR_1].desired_voltage = 1.0;
            actuator_cmd_msg_.motor_commands[ghost_v5_config::INTAKE_MOTOR_1].active = true;
            actuator_cmd_msg_.motor_commands[ghost_v5_config::INTAKE_MOTOR_1].current_limit = 2500;

            actuator_cmd_msg_.motor_commands[ghost_v5_config::INTAKE_MOTOR_2].desired_velocity = intake_cmd;
            actuator_cmd_msg_.motor_commands[ghost_v5_config::INTAKE_MOTOR_2].desired_voltage = 1.0;
            actuator_cmd_msg_.motor_commands[ghost_v5_config::INTAKE_MOTOR_2].active = true;
            actuator_cmd_msg_.motor_commands[ghost_v5_config::INTAKE_MOTOR_2].current_limit = 2500;   
        }
        else if(curr_joystick_msg_->joystick_btn_down){
            float intake_cmd = -750;
            actuator_cmd_msg_.motor_commands[ghost_v5_config::INTAKE_MOTOR_1].desired_velocity = intake_cmd;
            actuator_cmd_msg_.motor_commands[ghost_v5_config::INTAKE_MOTOR_1].desired_voltage = -1.0;
            actuator_cmd_msg_.motor_commands[ghost_v5_config::INTAKE_MOTOR_1].active = true;
            actuator_cmd_msg_.motor_commands[ghost_v5_config::INTAKE_MOTOR_1].current_limit = 2500;

            actuator_cmd_msg_.motor_commands[ghost_v5_config::INTAKE_MOTOR_2].desired_velocity = intake_cmd;
            actuator_cmd_msg_.motor_commands[ghost_v5_config::INTAKE_MOTOR_2].desired_voltage = -1.0;
            actuator_cmd_msg_.motor_commands[ghost_v5_config::INTAKE_MOTOR_2].active = true;
            actuator_cmd_msg_.motor_commands[ghost_v5_config::INTAKE_MOTOR_2].current_limit = 2500;
        }

        if(curr_joystick_msg_->joystick_btn_up){
            actuator_cmd_msg_.motor_commands[ghost_v5_config::ENDGAME_MOTOR].desired_voltage = 1.0;
            actuator_cmd_msg_.motor_commands[ghost_v5_config::ENDGAME_MOTOR].active = true;
            actuator_cmd_msg_.motor_commands[ghost_v5_config::ENDGAME_MOTOR].current_limit = 2500;
        }

        // Toggle shooter mode
        if(curr_joystick_msg_->joystick_btn_r1 && !r1_pressed_){
            if(teleop_mode == INTAKE_MODE){
                teleop_mode = SHOOTER_MODE;
            }
            else if(teleop_mode == SHOOTER_MODE){
                teleop_mode = INTAKE_MODE;
            }
            r1_pressed_ = true;
        }
        else if(!curr_joystick_msg_->joystick_btn_r1){
            r1_pressed_ = false;
        }

        if(curr_joystick_msg_->joystick_btn_b && teleop_mode == SHOOTER_MODE){
            teleop_mode = INTAKE_MODE;
        }

        if(curr_joystick_msg_->joystick_btn_left){
            resetPose();
        }

        float des_angle = M_PI/180.0*ghost_util::WrapAngle360(atan2(y_goal_ - curr_robot_state_msg_->y, x_goal_ - curr_robot_state_msg_->x)*180.0/M_PI);
        float des_x = 0.0;
        float des_y = 0.0;

        if(curr_joystick_msg_->joystick_btn_l1){

            float ang_vel_cmd = 
                ghost_util::SmallestAngleDistDeg(des_angle * 180 / M_PI, curr_robot_state_msg_->theta * 180 / M_PI) * M_PI/180.0 * rotate_kp_ -
                curr_robot_state_msg_->theta_vel * rotate_kd_;

            float x_vel_cmd =
                (des_x - curr_robot_state_msg_->x) * translate_kp_ -
                curr_robot_state_msg_->x_vel * translate_kd_;
            
            float y_vel_cmd =
                (des_y - curr_robot_state_msg_->y) * translate_kp_ -
                curr_robot_state_msg_->y_vel * translate_kd_;

            if(sqrt(
                fabs(des_x - curr_robot_state_msg_->x)*fabs(des_x - curr_robot_state_msg_->x) +
                fabs(des_y - curr_robot_state_msg_->y)*fabs(des_y - curr_robot_state_msg_->y)) < translation_tolerance_){
                    x_vel_cmd = 0.0;
                    y_vel_cmd = 0.0;
                }

            auto rotate_base_to_world = Eigen::Rotation2D<float>(-curr_robot_state_msg_->theta).toRotationMatrix();
            Eigen::Vector2f xy_vel = rotate_base_to_world * Eigen::Vector2f(x_vel_cmd, y_vel_cmd);

            x_vel_cmd = xy_vel.x();
            y_vel_cmd = xy_vel.y();

            if((fabs(x_vel_cmd) - fabs(last_xy_vel_cmd_.x())) > translation_slew_){
                x_vel_cmd = last_xy_vel_cmd_.x() + Sign(x_vel_cmd - last_xy_vel_cmd_.x()) * translation_slew_;
            }
            if((fabs(y_vel_cmd) - fabs(last_xy_vel_cmd_.y())) > translation_slew_){
                y_vel_cmd = last_xy_vel_cmd_.y() + Sign(y_vel_cmd - last_xy_vel_cmd_.y()) * translation_slew_;
            }
            last_xy_vel_cmd_ = Eigen::Vector2f(x_vel_cmd, y_vel_cmd);
            
            if((fabs(ang_vel_cmd) - fabs(last_ang_vel_cmd_)) > rotation_slew_){
                ang_vel_cmd = last_ang_vel_cmd_ + Sign(ang_vel_cmd - last_ang_vel_cmd_) * rotation_slew_;
            }
            last_ang_vel_cmd_ = ang_vel_cmd;

            updateSwerveCommandsFromTwist(
                ang_vel_cmd,  // Angular Velocity
                x_vel_cmd,    // X Velocity (Forward)
                y_vel_cmd);  // Y Velocity (Left)

            // updateSwerveCommandsFromTwist(
            //     ang_vel_cmd,  // Angular Velocity
            //     curr_joystick_msg_->joystick_left_y,    // X Velocity (Forward)
            //     -curr_joystick_msg_->joystick_left_x);  // Y Velocity (Left)
        }

        switch(teleop_mode){
            case SHOOTER_MODE:
                actuator_cmd_msg_.motor_commands[ghost_v5_config::SHOOTER_LEFT_MOTOR].desired_velocity = 2800;
                actuator_cmd_msg_.motor_commands[ghost_v5_config::SHOOTER_RIGHT_MOTOR].desired_velocity = 1200;
                
                actuator_cmd_msg_.motor_commands[ghost_v5_config::SHOOTER_LEFT_MOTOR].current_limit = 2500;
                actuator_cmd_msg_.motor_commands[ghost_v5_config::SHOOTER_RIGHT_MOTOR].current_limit = 2500;

                actuator_cmd_msg_.motor_commands[ghost_v5_config::SHOOTER_LEFT_MOTOR].active = true;
                actuator_cmd_msg_.motor_commands[ghost_v5_config::SHOOTER_RIGHT_MOTOR].active = true;

                if(curr_joystick_msg_->joystick_btn_l2){
                actuator_cmd_msg_.motor_commands[ghost_v5_config::INDEXER_MOTOR].desired_voltage = 1.0;
                actuator_cmd_msg_.motor_commands[ghost_v5_config::INDEXER_MOTOR].current_limit = 2500;
                actuator_cmd_msg_.motor_commands[ghost_v5_config::INDEXER_MOTOR].active = true;
                }

            break;

            case INTAKE_MODE:
            break;
        }
    }

    void RobotStateMachineNode::updateSwerveCommandsFromTwist(
        float angular_velocity,
        float x_velocity,
        float y_velocity)
    {
        // Convert joystick to robot twist command
        Eigen::Vector2f xy_vel_cmd(x_velocity, y_velocity);
        float angular_vel_cmd = std::clamp<float>(angular_velocity, -1.0, 1.0) * max_angular_vel_;
        float linear_vel_cmd = std::clamp<float>(xy_vel_cmd.norm(), -1.0, 1.0) * max_linear_vel_;

        // Zero commands under 1%
        linear_vel_cmd = (fabs(linear_vel_cmd) > max_linear_vel_ * 0.01) ? linear_vel_cmd : 0.0;
        angular_vel_cmd = (fabs(angular_vel_cmd) > max_angular_vel_ * 0.01) ? angular_vel_cmd : 0.0;

        // Calculate Linear Velocity Direction w/ error checking
        Eigen::Vector2f linear_vel_dir(0.0, 0.0);
        if (xy_vel_cmd.norm() != 0)
        {
            linear_vel_dir = xy_vel_cmd / xy_vel_cmd.norm();
        }

        // Calculate Wheel and Steering Setpoints
        std::vector<float> steering_angles = {
            ghost_util::WrapAngle360(curr_encoder_msg_->encoders[ghost_v5_config::STEERING_LEFT_ENCODER].angle_degrees),
            ghost_util::WrapAngle360(curr_encoder_msg_->encoders[ghost_v5_config::STEERING_RIGHT_ENCODER].angle_degrees),
            ghost_util::WrapAngle360(curr_encoder_msg_->encoders[ghost_v5_config::STEERING_BACK_ENCODER].angle_degrees),
        };

        // Swerve Drive Setpoints
        std::vector<float> steering_angle_cmd_(3, 0.0);
        std::vector<float> steering_velocity_cmd_(3, 0.0);
        std::vector<float> steering_voltage_cmd_(3, 0.0);

        std::vector<float> wheel_velocity_cmd_(3, 0.0);
        std::vector<float> wheel_voltage_cmd_(3, 0.0);

        std::vector<Eigen::Vector2f> wheel_vel_vectors(3, Eigen::Vector2f(0, 0));
        std::vector<Eigen::Vector2f> wheel_positions = {left_wheel_pos_, right_wheel_pos_, back_wheel_pos_};
        for (int wheel_id = 0; wheel_id < 3; wheel_id++)
        {
            // Calculate linear velocity vector at wheel
            Eigen::Vector2f vel_vec(-wheel_positions[wheel_id].y(), wheel_positions[wheel_id].x());
            vel_vec *= angular_vel_cmd;
            vel_vec += linear_vel_cmd * linear_vel_dir;
            wheel_vel_vectors[wheel_id] = vel_vec;

            // Calculate naive steering angle and wheel velocity setpoints
            steering_angle_cmd_[wheel_id] = ghost_util::WrapAngle360(atan2(vel_vec.y(), vel_vec.x()) * 180.0 / M_PI);   // Converts rad/s to degrees
            wheel_velocity_cmd_[wheel_id] = vel_vec.norm() * 100 / 2.54 / (2.75 * M_PI) * 60; // Convert m/s to RPM

            // Calculate angle error and then use direction of smallest error
            float steering_error  = ghost_util::SmallestAngleDistDeg(steering_angle_cmd_[wheel_id], steering_angles[wheel_id]);

            // It is faster to reverse wheel direction and steer to opposite angle
            if(fabs(steering_error) > 90.0){
                // Flip commands
                wheel_velocity_cmd_[wheel_id] *= -1.0;
                steering_angle_cmd_[wheel_id] = ghost_util::FlipAngle180(steering_angle_cmd_[wheel_id]);

                // Recalculate error
                steering_error = ghost_util::SmallestAngleDistDeg(steering_angle_cmd_[wheel_id], steering_angles[wheel_id]);
            }

            // Set steering voltage using position control law
            steering_voltage_cmd_[wheel_id] = steering_error * steering_kp_;
        }
        publishSwerveKinematicsVisualization(wheel_vel_vectors[0], wheel_vel_vectors[1], wheel_vel_vectors[2]);

        // Calculate Actuator Commands
        Eigen::Matrix2f diff_swerve_jacobian;
        Eigen::Matrix2f diff_swerve_jacobian_inverse;
        diff_swerve_jacobian << 13.0 / 18.0 / 2.0, -13.0 / 18.0 / 2.0, 13.0 / 45.0 / 2.0, 13.0 / 45.0 / 2.0;
        diff_swerve_jacobian_inverse << 18.0 / 13.0, 45.0 / 13.0, -18.0 / 13.0, 45.0 / 13.0;

        // Convert steering and wheel commands to actuator space
        std::vector<float> motor_speed_cmds(6, 0.0);
        std::vector<float> motor_voltage_cmds(6, 0.0);

        // Normalize velocities based on vel saturation, transform velocities to actuator space
        for (int wheel_id = 0; wheel_id < 3; wheel_id++)
        {
            // Transform velocity setpoints to actuator space
            Eigen::Vector2f wheel_module_vel_cmd(wheel_velocity_cmd_[wheel_id], steering_velocity_cmd_[wheel_id]);
            Eigen::Vector2f actuator_vel_cmd = diff_swerve_jacobian_inverse * wheel_module_vel_cmd;

            motor_speed_cmds[2 * wheel_id] = actuator_vel_cmd[0];
            motor_speed_cmds[2 * wheel_id + 1] = actuator_vel_cmd[1];

            // Transform position control output to actuator space
            Eigen::Vector2f wheel_module_voltage_cmd(wheel_voltage_cmd_[wheel_id], steering_voltage_cmd_[wheel_id]);
            Eigen::Vector2f actuator_voltage_cmd = diff_swerve_jacobian.transpose() * wheel_module_voltage_cmd;
            motor_voltage_cmds[2 * wheel_id] = actuator_voltage_cmd[0];
            motor_voltage_cmds[2 * wheel_id + 1] = actuator_voltage_cmd[1];
        }

        // Get largest speed magnitude
        double max_speed_val = 0;
        for (float &speed : motor_speed_cmds)
        {
            max_speed_val = std::max(fabs(speed), max_speed_val);
        }

        // Normalize if any motor exceeds maximum possible speed
        if (max_speed_val > max_motor_rpm_true_)
        {
            for (float &motor_speed : motor_speed_cmds)
            {
                motor_speed *= (max_motor_rpm_true_ / max_speed_val);
            }
        }

        // If we don't receive non-zero user input, zero everything
        if ((fabs(linear_vel_cmd) < 1e-5) && (fabs(angular_vel_cmd) < 1e-5))
        {
            for (int motor_id = 0; motor_id < 6; motor_id++)
            {
                motor_speed_cmds[motor_id] = 0.0;
                motor_voltage_cmds[motor_id] = 0.0;
            }
        }

        Eigen::Vector2f wheel_mod_speed = diff_swerve_jacobian * Eigen::Vector2f(
            curr_encoder_msg_->encoders[ghost_v5_config::DRIVE_BACK_RIGHT_REAR_MOTOR].velocity_rpm,
            curr_encoder_msg_->encoders[ghost_v5_config::DRIVE_BACK_LEFT_REAR_MOTOR].velocity_rpm);

        // Update velocity commands
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_LEFT_FRONT_MOTOR].desired_velocity =    motor_speed_cmds[0];
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_LEFT_BACK_MOTOR].desired_velocity =     motor_speed_cmds[1];
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_RIGHT_FRONT_MOTOR].desired_velocity =   motor_speed_cmds[2];
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_RIGHT_BACK_MOTOR].desired_velocity =    motor_speed_cmds[3];
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_BACK_RIGHT_REAR_MOTOR].desired_velocity =  motor_speed_cmds[4];
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_BACK_RIGHT_FRONT_MOTOR].desired_velocity =  motor_speed_cmds[4];
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_BACK_LEFT_REAR_MOTOR].desired_velocity =   motor_speed_cmds[5];
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_BACK_LEFT_FRONT_MOTOR].desired_velocity =   motor_speed_cmds[5];

        // Update voltage commands
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_LEFT_FRONT_MOTOR].desired_voltage =     motor_voltage_cmds[0];
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_LEFT_BACK_MOTOR].desired_voltage =      motor_voltage_cmds[1];
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_RIGHT_FRONT_MOTOR].desired_voltage =    motor_voltage_cmds[2];
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_RIGHT_BACK_MOTOR].desired_voltage =     motor_voltage_cmds[3];
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_BACK_RIGHT_REAR_MOTOR].desired_voltage =   motor_voltage_cmds[4];
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_BACK_RIGHT_FRONT_MOTOR].desired_voltage =   motor_voltage_cmds[4];
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_BACK_LEFT_REAR_MOTOR].desired_voltage =    motor_voltage_cmds[5];
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_BACK_LEFT_FRONT_MOTOR].desired_voltage =    motor_voltage_cmds[5];

        // Set Current Limits
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_LEFT_FRONT_MOTOR].current_limit         = 2500;
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_LEFT_BACK_MOTOR].current_limit          = 2500;
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_RIGHT_FRONT_MOTOR].current_limit        = 2500;
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_RIGHT_BACK_MOTOR].current_limit         = 2500;
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_BACK_RIGHT_REAR_MOTOR].current_limit    = 2500;
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_BACK_RIGHT_FRONT_MOTOR].current_limit   = 2500;
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_BACK_LEFT_REAR_MOTOR].current_limit     = 2500;
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_BACK_LEFT_FRONT_MOTOR].current_limit    = 2500;

        // Set motors to be active
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_LEFT_FRONT_MOTOR].active         = true;
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_LEFT_BACK_MOTOR].active          = true;
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_RIGHT_FRONT_MOTOR].active        = true;
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_RIGHT_BACK_MOTOR].active         = true;
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_BACK_RIGHT_REAR_MOTOR].active    = true;
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_BACK_RIGHT_FRONT_MOTOR].active   = true;
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_BACK_LEFT_REAR_MOTOR].active     = true;
        actuator_cmd_msg_.motor_commands[ghost_v5_config::DRIVE_BACK_LEFT_FRONT_MOTOR].active    = true;

        // Set Motor Names and Device IDs
        for (auto pair : ghost_v5_config::actuator_command_config){
            int dev_id = pair.first;
            actuator_cmd_msg_.motor_commands[dev_id].motor_name =   ghost_v5_config::device_names.at(dev_id);
            actuator_cmd_msg_.motor_commands[dev_id].device_id =    dev_id;
        }
    }

    void RobotStateMachineNode::publishSwerveKinematicsVisualization(
        const Eigen::Vector2f &left_wheel_cmd,
        const Eigen::Vector2f &right_wheel_cmd,
        const Eigen::Vector2f &back_wheel_cmd)
    {
        auto debug_viz_msg = visualization_msgs::msg::MarkerArray{};
        auto marker_msgs = std::vector<visualization_msgs::msg::Marker>(3);

        // Initialize each arrow marker msg
        for (int i = 0; i < 3; i++)
        {
            marker_msgs[i].header.frame_id = "base_link";
            marker_msgs[i].header.stamp = get_clock()->now();
            marker_msgs[i].ns = "controller";
            marker_msgs[i].id = i;
            marker_msgs[i].type = visualization_msgs::msg::Marker::ARROW;
            marker_msgs[i].action = visualization_msgs::msg::Marker::ADD;
            marker_msgs[i].scale.x = 0.01;
            marker_msgs[i].scale.y = 0.025;
            marker_msgs[i].scale.z = 0.025;
            marker_msgs[i].color.a = 1.0;
            marker_msgs[i].color.r = 1.0;
        }

        std::vector<Eigen::Vector2f> points = {
            left_wheel_pos_,
            left_wheel_pos_ + left_wheel_cmd / 10.0,
            right_wheel_pos_,
            right_wheel_pos_ + right_wheel_cmd / 10.0,
            back_wheel_pos_,
            back_wheel_pos_ + back_wheel_cmd / 10.0,
        };

        // Assign the three marker msgs their corresponding start/end points
        for (int i = 0; i < 3; i++)
        {
            for (int k = 0; k < 2; k++)
            {
                auto point_msg = geometry_msgs::msg::Point{};
                point_msg.x = points[2 * i + k].x();
                point_msg.y = points[2 * i + k].y();
                point_msg.z = 0.0;
                marker_msgs[i].points.push_back(point_msg);
            }
        }

        // Add all markers to the debug message
        for (auto &msg : marker_msgs)
        {
            debug_viz_msg.markers.push_back(msg);
        }

        debug_viz_pub_->publish(debug_viz_msg);
    }

} // namespace ghost_ros

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ghost_ros::RobotStateMachineNode>());
    rclcpp::shutdown();
    return 0;
}