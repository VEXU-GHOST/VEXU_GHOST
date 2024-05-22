/*
 * Filename: gazebo_joint_pid_plugin
 * Created Date: Sunday August 7th 2022
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 *
 * Last Modified: Saturday September 10th 2022 10:50:34 am
 * Modified By: Maxx Wilson
 */

#include "gazebo_joint_pid_plugin.hpp"

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
  rclcpp::Subscription<ghost_msgs::msg::V5ActuatorCommand>::SharedPtr v5_actuator_cmd_sub_;

  /// Publishers
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr output_pub_;

  // Plugin params
  std::string joint_name_;
  std::string link_name_;
  std::vector<double> actuator_jacobian_;

  // Motor Parameters
  ghost_control::DCMotorModel motor_model_;
  double free_speed_;
  double stall_torque_;
  double free_current_;
  double stall_current_;
  double nominal_voltage_;
  double gear_ratio_;

  // Controller Parameters
  double position_gain_;
  double velocity_gain_;
  double feedforward_velocity_gain_;
  double feedforward_voltage_gain_;

  // Controller Setpoints
  double angle_setpoint_;
  double velocity_setpoint_;
  double voltage_setpoint_;
  double torque_setpoint_;
  double current_limit_;
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
  impl_->model_ = model;

  // Check for valid plugin config
  std::vector<std::string> params{
    "joint_name",
    "link_name",
    "position_gain",
    "velocity_gain",
    "feedforward_velocity_gain",
    "feedforward_voltage_gain",
    "free_speed",
    "stall_torque",
    "free_current",
    "stall_current",
    "nominal_voltage",
    "gear_ratio",
  };

  for (std::string & param : params) {
    if (!sdf->HasElement(param)) {
      RCLCPP_ERROR(logger, "Motor plugin missing <%s>, cannot proceed", param.c_str());
      return;
    }
  }

  // Get plugin config
  impl_->joint_name_ = sdf->GetElement("joint_name")->Get<std::string>();
  impl_->link_name_ = sdf->GetElement("link_name")->Get<std::string>();
  //   impl_-> actuator_jacobian_ = sdf->GetElement("actuator_jacobian")->Get<std::vector<double>>();
  impl_->position_gain_ = sdf->GetElement("position_gain")->Get<double>();
  impl_->velocity_gain_ = sdf->GetElement("velocity_gain")->Get<double>();
  impl_->feedforward_velocity_gain_ = sdf->GetElement("feedforward_velocity_gain")->Get<double>();
  impl_->feedforward_voltage_gain_ = sdf->GetElement("feedforward_voltage_gain")->Get<double>();

  impl_->free_speed_ = sdf->GetElement("free_speed")->Get<double>();
  impl_->stall_torque_ = sdf->GetElement("stall_torque")->Get<double>();
  impl_->free_current_ = sdf->GetElement("free_current")->Get<double>();
  impl_->stall_current_ = sdf->GetElement("stall_current")->Get<double>();
  impl_->nominal_voltage_ = sdf->GetElement("nominal_voltage")->Get<double>();
  impl_->gear_ratio_ = sdf->GetElement("gear_ratio")->Get<double>();

  // Initialize Subscriptions
  impl_->v5_actuator_cmd_sub_ =
    impl_->ros_node_->create_subscription<ghost_msgs::msg::V5ActuatorCommand>(
    "/v5actuator/setpoint", 10,
    [this](const ghost_msgs::msg::V5ActuatorCommand::SharedPtr msg) {
      this->GazeboJointPIDPlugin::v5ActuatorCallback(msg);
    });

  // Initialize Publisher
  //   impl_->output_pub_ = impl_->ros_node_->create_publisher<geometry_msgs::msg::Vector3>(
  //     "v5actuator/output",
  //     10);

  // Initalize motor models
  impl_->motor_model_ = ghost_control::DCMotorModel(
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
  impl_->velocity_setpoint_ = 0.0;
  impl_->voltage_setpoint_ = 0.0;

  // Create a connection so the OnUpdate function is called at every simulation
  // iteration. Remove this call, the connection and the callback if not needed.
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboJointPIDPlugin::OnUpdate, this));
}

void GazeboJointPIDPlugin::OnUpdate()
{
  // Get joint velocity
  auto joint_vel_rpm = impl_->joint_->GetVelocity(2) * 60 / (2 * 3.14159);
  auto joint_angle_deg = fmod(impl_->joint_->Position(2) * 180 / 3.14159, 360);

  // Wrap angle error
  double angle_error = fmod(impl_->angle_setpoint_, 360) - joint_angle_deg;
  double angle_sign = std::abs(angle_error) / angle_error;
  angle_error = std::abs(angle_error) > 180 ? angle_sign * 360 - angle_error : angle_error;

  // Update motor
  impl_->motor_model_.setMotorSpeedRPM(joint_vel_rpm);

  // Calculate control inputs
  float voltage_feedforward = impl_->voltage_setpoint_ * impl_->nominal_voltage_ * 1000 *
    impl_->feedforward_voltage_gain_;
  float velocity_feedforward = impl_->motor_model_.getVoltageFromVelocityMillivolts(
    impl_->velocity_setpoint_) * impl_->feedforward_velocity_gain_;
  float velocity_feedback = (impl_->velocity_setpoint_ - joint_vel_rpm) * impl_->velocity_gain_;
  float position_feedback = (impl_->angle_setpoint_ - joint_angle_deg) * impl_->position_gain_;

  impl_->motor_model_.setMotorEffort(
    voltage_feedforward + velocity_feedforward + velocity_feedback + position_feedback);

  auto motor_torque = impl_->motor_model_.getTorqueOutput();
  // For simulatilon stability, only apply torque if larger than a threshhold
  if (std::abs(motor_torque) > 1e-5) {
    impl_->link_->AddRelativeTorque(ignition::math::v6::Vector3d(0.0, 0.0, motor_torque));
  }

  // Publish current joint torque
  auto output_msg = geometry_msgs::msg::Vector3{};
  output_msg.x = joint_angle_deg;       // Angle in degrees
  output_msg.y = joint_vel_rpm;         // Velocity in RPM
  output_msg.z = motor_torque;          // Torque in N-m
  impl_->output_pub_->publish(output_msg);
}

void GazeboJointPIDPlugin::v5ActuatorCallback(
  const ghost_msgs::msg::V5ActuatorCommand::SharedPtr msg)
{
  // Parse V5ActuatorCommand.msg for corresponding motor command
  // TODO: change V5MotorCommand device_id to correspond to joint_name_ if not already
  for (ghost_msgs::msg::V5MotorCommand cmd : msg->motor_commands) {
    if (cmd.motor_name == impl_->joint_name_) {
      // int32 desired_position         # Degrees
      // float32 desired_velocity    # RPM
      // float32 desired_torque      # N-m
      // float32 desired_voltage     # Normalized (-1.0 -> 1.0)
      // int32 current_limit         # milliAmps

      // bool position_control
      // bool velocity_control
      // bool torque_control
      // bool voltage_control
      // Update setpoints
      if (cmd.position_control) {
        impl_->angle_setpoint_ = cmd.desired_position;
        RCLCPP_INFO(
          impl_->ros_node_->get_logger(),
          "angle_setpoint_: %f", impl_->angle_setpoint_);
      }
      if (cmd.velocity_control) {
        impl_->velocity_setpoint_ = cmd.desired_velocity;
      }
      if (cmd.torque_control) {
        impl_->torque_setpoint_ = cmd.desired_torque;
      }
      if (cmd.voltage_control) {
        impl_->voltage_setpoint_ = cmd.desired_voltage;
      }

      impl_->current_limit_ = cmd.current_limit;
    }
  }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboJointPIDPlugin)

}  // namespace gazebo_swerve_plugin
