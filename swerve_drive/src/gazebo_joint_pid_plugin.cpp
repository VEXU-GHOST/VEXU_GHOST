/*
 * Filename: gazebo_joint_pid_plugin
 * Created Date: Sunday August 7th 2022
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 * 
 * Last Modified: Sunday August 7th 2022 2:00:44 pm
 * Modified By: Maxx Wilson
 */

#include "gazebo_joint_pid_plugin.hpp"
#include "dc_motor_model.hpp"

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <math.h>
#include <vector>
#include <memory>

namespace gazebo_joint_pid_plugin
{
/// Class to hold private data members (PIMPL pattern)
class GazeboJointPIDPluginPrivate
{
public:
  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_;
  
  // Gazebo Ptrs
  gazebo::physics::ModelPtr model_;
  gazebo::physics::JointPtr joint_;
  gazebo::physics::LinkPtr link_;

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr input_setpoint_sub_;

  /// Publishers
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr output_pub_;

  // Plugin params
  std::string joint_name_;
  std::string link_name_;
  
  // Motor Parameters
  dc_motor_model::DCMotorModel motor_model_;
  double free_speed_;
  double stall_torque_;
  double free_current_;
  double stall_current_;
  double nominal_voltage_;
  double gear_ratio_;

  // Controller Parameters
  double pos_gain_;
  double vel_gain_;
  double accel_gain_;

  // Controller Setpoints
  double angle_setpoint_;
  double vel_setpoint_;
  double accel_setpoint_;
};

GazeboJointPIDPlugin::GazeboJointPIDPlugin()
: impl_(std::make_unique<GazeboJointPIDPluginPrivate>())
{
}

GazeboJointPIDPlugin::~GazeboJointPIDPlugin()
{
}

void GazeboJointPIDPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Get ROS Node and Gazebo Model Ptr
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
  auto logger = impl_->ros_node_->get_logger();
  impl_->model_= model;

  // Check for valid plugin config
  std::vector<std::string> params{
    "joint_name",
    "link_name",
    "position_gain",
    "vel_gain",
    "accel_gain",
    "free_speed",
    "stall_torque",
    "free_current",
    "stall_current",
    "nominal_voltage",
    "gear_ratio",
    };
    
  for(std::string& param: params){
    if(!sdf->HasElement(param)){
      RCLCPP_ERROR(logger, "Motor plugin missing <" + param + ">, cannot proceed");
      return;
    }
  }

  // Get plugin config
  impl_->joint_name_ = sdf->GetElement("joint_name")->Get<std::string>();
  impl_->link_name_ = sdf->GetElement("link_name")->Get<std::string>();

  impl_->pos_gain_ = sdf->GetElement("position_gain")->Get<double>();
  impl_->vel_gain_ = sdf->GetElement("vel_gain")->Get<double>();
  impl_->accel_gain_ = sdf->GetElement("accel_gain")->Get<double>();
  
  impl_->free_speed_ = sdf->GetElement("free_speed")->Get<double>();
  impl_->stall_torque_ = sdf->GetElement("stall_torque")->Get<double>();
  impl_->free_current_ = sdf->GetElement("free_current")->Get<double>();
  impl_->stall_current_ = sdf->GetElement("stall_current")->Get<double>();
  impl_->nominal_voltage_ = sdf->GetElement("nominal_voltage")->Get<double>();
  impl_->gear_ratio_ = sdf->GetElement("gear_ratio")->Get<double>();

  // Initialize Subscriptions
  impl_->input_setpoint_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Vector3>(
    "motors/" + impl_->link_name_ + "/setpoint",
    10,
    [this](const geometry_msgs::msg::Vector3::SharedPtr msg){
        impl_->angle_setpoint_ = msg->x;
        impl_->vel_setpoint_ = msg->y;
        impl_->accel_setpoint_ = msg->z;
    }
    );

  // Initialize Publisher
  impl_->output_pub_ = impl_->ros_node_->create_publisher<geometry_msgs::msg::Vector3>(
    "motors/" + impl_->link_name_ + "/output",
    10);

  // Initalize motor models
  impl_->motor_model_ = dc_motor_model::DCMotorModel(
    impl_->free_speed_,
    impl_->stall_torque_,
    impl_->free_current_,
    impl_->stall_current_,
    impl_->nominal_voltage_
    );

  // Set Gear Ratios
  impl_->motor_model_.setGearRatio(impl_->gear_ratio_);

  // Joint and Link Ptrs
  impl_->joint_ = impl_->model_->GetJoint(impl_->joint_name_);
  impl_->link_ = impl_->model_->GetLink(impl_->link_name_);

  // Initialize setpoints
  impl_->angle_setpoint_ = 0.0;
  impl_->vel_setpoint_ = 0.0;
  impl_->accel_setpoint_ = 0.0;

  // Create a connection so the OnUpdate function is called at every simulation
  // iteration. Remove this call, the connection and the callback if not needed.
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboJointPIDPlugin::OnUpdate, this));
}

void GazeboJointPIDPlugin::OnUpdate()
{
  // // Get joint velocity
  auto joint_vel_rpm = impl_->joint_->GetVelocity(2)*60/(2*3.14159);
  auto joint_angle_deg = fmod(impl_->joint_->Position(2)*180/3.14159, 360);

  // // Wrap angle error
  double angle_error = fmod(impl_->angle_setpoint_, 360) - joint_angle_deg;
  double angle_sign = std::abs(angle_error)/angle_error;
  angle_error = std::abs(angle_error) > 180 ? angle_sign*360 - angle_error : angle_error; 

  // // Update motor
  impl_->motor_model_.setMotorSpeedRPM(joint_vel_rpm);
  impl_->motor_model_.setMotorEffort(impl_->accel_gain_ * impl_->accel_setpoint_ + impl_->vel_gain_ * (impl_->vel_setpoint_ - joint_vel_rpm) + impl_->pos_gain_ * angle_error);
  
  auto motor_torque = impl_->motor_model_.getTorqueOutput();
  // For simulatilon stability, only apply torque if larger than a threshhold
  if(std::abs(motor_torque) > 1e-5){
    impl_->link_->AddRelativeTorque(ignition::math::v6::Vector3d(0.0, 0.0, motor_torque));
  }

  // Publish current joint torque
  auto output_msg = geometry_msgs::msg::Vector3{};
  output_msg.x = joint_angle_deg; // Angle in degrees
  output_msg.y = joint_vel_rpm;   // Velocity in RPM
  output_msg.z = motor_torque;    // Torque in N-m
  impl_->output_pub_->publish(output_msg);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboJointPIDPlugin)
}  // namespace gazebo_swerve_plugin