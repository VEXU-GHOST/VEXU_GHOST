#include <chrono>
#include <cmath>
#include <stdexcept>

#include "eigen3/Eigen/Geometry"

#include "ghost_ros/robot_config/v5_port_config.hpp"
#include "ghost_ros/robot_config/v5_serial_msg_config.hpp"
#include "ghost_ros/ros_nodes/robot_state_machine_node.hpp"

#include "math/math_util.h"

using namespace std::chrono_literals;
using namespace math_util;
using std::placeholders::_1;

namespace ghost_ros
{

    RobotStateMachineNode::RobotStateMachineNode() : rclcpp::Node("ghost_state_machine"),
                                                     curr_robot_state_{robot_state_e::DISABLED},
                                                     curr_robot_state_msg_{0},
                                                     curr_joystick_msg_id_{0},
                                                     curr_comp_state_msg_id_{0}
    {
        declare_parameter("max_linear_vel", 0.0);
        declare_parameter("max_steering_angular_vel", 0.0);
        max_linear_vel_ = get_parameter("max_linear_vel").as_double();
        max_steering_angular_vel_ = get_parameter("max_steering_angular_vel").as_double();

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

        actuator_command_pub_ = create_publisher<ghost_msgs::msg::V5ActuatorCommand>(
            "v5/actuator_commands",
            10);
        debug_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "controller_debug",
            10);

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
            "estimation/robot_pose",
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
            (curr_joystick_msg_id_ == curr_encoder_msg_id_)) // &&
        // (curr_joystick_msg_id_ == curr_robot_state_msg_))
        {
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
        }
    }

    void RobotStateMachineNode::teleop()
    {
        // Convert joystick to robot twist command
        Eigen::Vector2f xy_vel_cmd(curr_joystick_msg_->joystick_left_y, -curr_joystick_msg_->joystick_left_x);
        float angular_vel_cmd = std::clamp<float>(-curr_joystick_msg_->joystick_right_x, -1.0, 1.0) * max_angular_vel_;
        float linear_vel_cmd = std::clamp<float>(xy_vel_cmd.norm(), -1.0, 1.0) * max_linear_vel_;

        // Swerve Drive Setpoints
        std::vector<float> steering_angle_cmd_(3, 0.0);
        std::vector<float> steering_velocity_cmd_(3, 0.0);
        std::vector<float> steering_voltage_cmd_(3, 0.0);

        std::vector<float> wheel_velocity_cmd_(3, 0.0);
        std::vector<float> wheel_voltage_cmd_(3, 0.0);

        // Convert steering and wheel commands to actuator space
        std::vector<float> motor_speed_cmds(6, 0.0);
        std::vector<float> motor_voltage_cmds(6, 0.0);

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
        std::vector<Eigen::Vector2f> wheel_vel_vectors(3, Eigen::Vector2f(0, 0));
        std::vector<Eigen::Vector2f> wheel_positions = {left_wheel_pos_, right_wheel_pos_, back_wheel_pos_};
        for (int wheel_id = 0; wheel_id < 3; wheel_id++)
        {
            Eigen::Vector2f vel_vec(-wheel_positions[wheel_id].y(), wheel_positions[wheel_id].x());

            vel_vec *= angular_vel_cmd;
            vel_vec += linear_vel_cmd * linear_vel_dir;

            wheel_vel_vectors[wheel_id] = vel_vec;

            steering_angle_cmd_[wheel_id] = atan2(vel_vec.y(), vel_vec.x()) * 180.0 / M_PI;   // Converts rad/s to degrees
            wheel_velocity_cmd_[wheel_id] = vel_vec.norm() * 100 / 2.54 / (2.75 * M_PI) * 60; // Convert m/s to RPM
            wheel_velocity_cmd_[wheel_id] = 0.0;                                              // TEMP
            }

        publishSwerveKinematicsVisualization(wheel_vel_vectors[0], wheel_vel_vectors[1], wheel_vel_vectors[2]);

        // Apply Steering Control
        float left_steering_error =
            steering_angle_cmd_[0] - curr_encoder_msg_->encoders[ghost_v5_config::STEERING_LEFT_ENCODER].current_angle;
        float right_steering_error =
            steering_angle_cmd_[1] - curr_encoder_msg_->encoders[ghost_v5_config::STEERING_RIGHT_ENCODER].current_angle;
        float back_steering_error =
            steering_angle_cmd_[2] - curr_encoder_msg_->encoders[ghost_v5_config::STEERING_BACK_ENCODER].current_angle;

        // Wrap angles to 180
        left_steering_error -= float(180.0) * rint(left_steering_error / float(180));
        right_steering_error -= float(180.0) * rint(right_steering_error / float(180));
        back_steering_error -= float(180.0) * rint(back_steering_error / float(180));

        steering_voltage_cmd_[0] = left_steering_error * steering_kp_;
        steering_voltage_cmd_[1] = right_steering_error * steering_kp_;
        steering_voltage_cmd_[2] = back_steering_error * steering_kp_;

        // Calculate Actuator Commands
        Eigen::Matrix2f diff_swerve_jacobian;
        Eigen::Matrix2f diff_swerve_jacobian_inverse;
        diff_swerve_jacobian << 5.0 / 6.0, -5.0 / 6.0, 1.0 / 6.0, 1.0 / 6.0;
        diff_swerve_jacobian_inverse << 0.6, 3.0, -0.6, 3.0;

        // Normalize velocities based on vel saturation, transform velocities to actuator space
        for (int wheel_id = 0; wheel_id < 3; wheel_id++)
        {
            // Transform velocity setpoints to actuator space
            Eigen::Vector2f wheel_module_vel_cmd(wheel_velocity_cmd_[wheel_id], steering_velocity_cmd_[wheel_id]);
            Eigen::Vector2f actuator_vel_cmd = diff_swerve_jacobian_inverse * wheel_module_vel_cmd;

            motor_speed_cmds[2 * wheel_id] = actuator_vel_cmd.x();
            motor_speed_cmds[2 * wheel_id + 1] = actuator_vel_cmd.y();

            // Transform position control output to actuator space
            Eigen::Vector2f wheel_module_voltage_cmd(0.0, steering_voltage_cmd_[wheel_id]);
            Eigen::Vector2f actuator_voltage_cmd = diff_swerve_jacobian.transpose() * wheel_module_voltage_cmd;
            motor_voltage_cmds[2 * wheel_id] = actuator_voltage_cmd.x();
            motor_voltage_cmds[2 * wheel_id + 1] = actuator_voltage_cmd.y();
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

        std::vector<bool> digital_outs(8, false);
        if(curr_joystick_msg_->joystick_btn_a){
            digital_outs[0] = true;
        }
        else{
            digital_outs[0] = false;
        }

        if(curr_joystick_msg_->joystick_btn_b){
            digital_outs[1] = true;
        }
        else{
            digital_outs[1] = false;
        }

        publishActuatorCommand(motor_speed_cmds, motor_voltage_cmds, digital_outs);
    }

    void RobotStateMachineNode::publishActuatorCommand(
        const std::vector<float> &speed_cmd_array,
        const std::vector<float> &voltage_cmd_array,
        const std::vector<bool> &digital_outs)
    {
        // Generate actuator command msg
        auto actuator_cmd_msg = ghost_msgs::msg::V5ActuatorCommand{};
        actuator_cmd_msg.header.stamp = get_clock()->now();
        actuator_cmd_msg.msg_id = curr_joystick_msg_id_;

        // Update velocity commands
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_LEFT_FRONT_MOTOR].desired_velocity = speed_cmd_array[0];
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_LEFT_BACK_MOTOR].desired_velocity = speed_cmd_array[1];
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_RIGHT_FRONT_MOTOR].desired_velocity = speed_cmd_array[2];
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_RIGHT_BACK_MOTOR].desired_velocity = speed_cmd_array[3];
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_BACK_RIGHT_1_MOTOR].desired_velocity = speed_cmd_array[4];
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_BACK_RIGHT_2_MOTOR].desired_velocity = speed_cmd_array[4];
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_BACK_LEFT_1_MOTOR].desired_velocity = speed_cmd_array[5];
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_BACK_LEFT_2_MOTOR].desired_velocity = speed_cmd_array[5];

        // Update voltage commands
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_LEFT_FRONT_MOTOR].desired_voltage = voltage_cmd_array[0];
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_LEFT_BACK_MOTOR].desired_voltage = voltage_cmd_array[1];
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_RIGHT_FRONT_MOTOR].desired_voltage = voltage_cmd_array[2];
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_RIGHT_BACK_MOTOR].desired_voltage = voltage_cmd_array[3];
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_BACK_RIGHT_1_MOTOR].desired_voltage = voltage_cmd_array[4];
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_BACK_RIGHT_2_MOTOR].desired_voltage = voltage_cmd_array[4];
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_BACK_LEFT_1_MOTOR].desired_voltage = voltage_cmd_array[5];
        actuator_cmd_msg.motor_commands[ghost_v5_config::DRIVE_BACK_LEFT_2_MOTOR].desired_voltage = voltage_cmd_array[5];

        for(int i = 0; i < 8; i++){
            actuator_cmd_msg.digital_out_vector[i] = digital_outs[i];
        }

        actuator_command_pub_->publish(actuator_cmd_msg);
    }

    void RobotStateMachineNode::publishSwerveKinematicsVisualization(
        const Eigen::Vector2f &left_wheel_cmd,
        const Eigen::Vector2f &right_wheel_cmd,
        const Eigen::Vector2f &back_wheel_cmd)
    {
        auto debug_viz_msg = visualization_msgs::msg::MarkerArray{};
        auto marker_msgs = std::vector<visualization_msgs::msg::Marker>{3};

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