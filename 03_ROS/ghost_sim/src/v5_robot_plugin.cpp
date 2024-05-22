/*
 * Filename: swerve_odometry_plugin
 * Created Date: Monday October 24th 2022
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 *
 * Last Modified: Monday October 24th 2022 2:19:23 pm
 * Modified By: Maxx Wilson
 */

#include <iostream>
#include <mutex>
#include <unordered_map>

#include "eigen3/Eigen/Geometry"

#include "ghost_msgs/msg/v5_actuator_command.hpp"
#include "ghost_msgs/msg/v5_encoder_state.hpp"
#include "ghost_msgs/msg/v5_sensor_update.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "ghost_common/util/angle_util.hpp"
#include "ghost_common/util/parsing_util.hpp"

#include "ghost_common/v5_robot_config_defs.hpp"

#include "ghost_control/models/dc_motor_model.hpp"
#include "ghost_control/motor_controller.hpp"

#include "ghost_sim/v5_robot_plugin.hpp"

using Eigen::Dynamic;
using Eigen::Matrix;
using Eigen::RowMajor;

using ghost_control::DCMotorModel;
using ghost_control::MotorController;

namespace v5_robot_plugin
{

// Class to hold private data members (PIMPL pattern)
class V5RobotPluginPrivate
{
public:
  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_;

  // Gazebo Ptrs
  gazebo::physics::ModelPtr model_;

  // Time object
  rclcpp::Clock clock_;

  // Parameter Vectors
  std::vector<std::string> motor_names_;
  std::vector<std::string> encoder_names_;
  std::vector<std::string> joint_names_;

  // Incoming joint state data
  Eigen::VectorXd joint_angles_;
  Eigen::VectorXd joint_velocities_;
  Eigen::VectorXd joint_efforts_;
  Eigen::VectorXd joint_cmd_torques_;

  // Incoming high level motor state data
  Eigen::VectorXd motor_positions_;
  Eigen::VectorXd motor_velocities_;
  Eigen::VectorXd motor_efforts_;

  // Outgoing encoder data
  Eigen::VectorXd encoder_positions_;
  Eigen::VectorXd encoder_velocities_;

  // Motor Controller Gains
  double feedforward_voltage_gain_ = 1.0;
  double feedforward_velocity_gain_ = 1.0;
  double velocity_gain_ = 15.0;
  double position_gain_ = 0.0;
  double nominal_voltage_;

  // Note that the float32 torque_nm parameter of this msg type is ignored since
  // torques can only be applied not measured in Gazebo
  std::array<ghost_msgs::msg::V5EncoderState, 21> encoder_msg_;
  ghost_msgs::msg::V5SensorUpdate sensor_msg_;

  std::unordered_map<std::string, std::shared_ptr<DCMotorModel>> motor_model_map_;
  std::unordered_map<std::string, std::shared_ptr<MotorController>> motor_controller_map_;

  Eigen::MatrixXd actuator_jacobian_;
  Eigen::MatrixXd sensor_jacobian_;

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<ghost_msgs::msg::V5ActuatorCommand>::SharedPtr actuator_command_sub_;
  rclcpp::Publisher<ghost_msgs::msg::V5SensorUpdate>::SharedPtr sensor_update_pub_;

  std::mutex actuator_update_callback_mutex;

  // node for simulator joystick commands
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_command_sub_;
  ghost_msgs::msg::V5Joystick joystick_msg_;

  // Constants
  // float DEG_TO_RAD = M_PI / 180;
  // float RAD_TO_DEG = 180 / M_PI;
  float RADS_TO_RPM = (0.5 * 60) / (M_PI);
};

V5RobotPlugin::V5RobotPlugin()
: impl_(std::make_unique<V5RobotPluginPrivate>())
{
}

V5RobotPlugin::~V5RobotPlugin()
{
}

void V5RobotPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Get ROS Node and Gazebo Model Ptr
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
  auto logger = impl_->ros_node_->get_logger();
  impl_->model_ = model;

  // Initialize ROS Subscriptions
  impl_->actuator_command_sub_ =
    impl_->ros_node_->create_subscription<ghost_msgs::msg::V5ActuatorCommand>(
    "v5/actuator_command",
    10,
    [this](const ghost_msgs::msg::V5ActuatorCommand::SharedPtr msg) {
      std::unique_lock update_lock(impl_->actuator_update_callback_mutex);
      for (const std::string & name : impl_->motor_names_) {
        // Get V5 Motor Command msg, Motor Model, and Motor Controller for each motor
        const auto & motor_cmd_msg =
        msg->motor_commands[ghost_v5_config::motor_config_map.at(name).port];
        auto motor_model_ptr = impl_->motor_model_map_.at(name);
        auto motor_controller_ptr = impl_->motor_controller_map_.at(name);

        // Update Controller Setpoints
        motor_controller_ptr->setMotorCommand(
          motor_cmd_msg.desired_position,
          motor_cmd_msg.desired_velocity,
          motor_cmd_msg.desired_voltage,
          motor_cmd_msg.desired_torque);

        // Update Control Mode
        motor_controller_ptr->setControlMode(
          motor_cmd_msg.position_control,
          motor_cmd_msg.velocity_control,
          motor_cmd_msg.voltage_control,
          motor_cmd_msg.torque_control);

        // Update Current Limit
        motor_model_ptr->setMaxCurrent(motor_cmd_msg.current_limit);
      }
    });

  impl_->joystick_command_sub_ = impl_->ros_node_->create_subscription<sensor_msgs::msg::Joy>(
    "/joy",
    10,
    [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
      impl_->joystick_msg_.joystick_left_x = msg->axes[0];
      impl_->joystick_msg_.joystick_left_y = msg->axes[1];
      impl_->joystick_msg_.joystick_right_x = msg->axes[2];
      impl_->joystick_msg_.joystick_right_y = msg->axes[3];

      impl_->joystick_msg_.btn_x = msg->buttons[0];
      impl_->joystick_msg_.btn_a = msg->buttons[1];
      impl_->joystick_msg_.btn_b = msg->buttons[2];
      impl_->joystick_msg_.btn_y = msg->buttons[3];
      impl_->joystick_msg_.btn_l1 = msg->buttons[4];
      impl_->joystick_msg_.btn_r1 = msg->buttons[5];
      impl_->joystick_msg_.btn_l2 = msg->buttons[6];
      impl_->joystick_msg_.btn_r2 = msg->buttons[7];

      impl_->joystick_msg_.btn_left = std::max(0, int(msg->axes[4]));
      impl_->joystick_msg_.btn_right = std::max(0, -int(msg->axes[4]));
      impl_->joystick_msg_.btn_up = std::max(0, int(msg->axes[5]));
      impl_->joystick_msg_.btn_down = std::max(0, -int(msg->axes[5]));
    });

  // Initialize ROS Publishers
  impl_->sensor_update_pub_ = impl_->ros_node_->create_publisher<ghost_msgs::msg::V5SensorUpdate>(
    "v5/sensor_update",
    10);

  // Check for required parameters
  std::vector<std::string> params{
    "joint_names",
    "motor_names",
    "encoder_names",
    "actuator_jacobian",
    "sensor_jacobian"};

  for (std::string & param : params) {
    if (!sdf->HasElement(param)) {
      std::string err_string = "[V5 Robot Plugin] Missing <" + param + ">, cannot proceed";
      RCLCPP_ERROR(logger, err_string.c_str());
      return;
    }
  }

  // Parse input plugin parameters
  impl_->joint_names_ = ghost_common::getVectorFromString<std::string>(
    sdf->GetElement(
      "joint_names")->Get<std::string>(), ' ');
  impl_->motor_names_ = ghost_common::getVectorFromString<std::string>(
    sdf->GetElement(
      "motor_names")->Get<std::string>(), ' ');
  impl_->encoder_names_ =
    ghost_common::getVectorFromString<std::string>(
    sdf->GetElement(
      "encoder_names")->Get<std::string>(), ' ');

  // Define eigen vector sizes using number of joints
  impl_->joint_angles_.resize(impl_->joint_names_.size());
  impl_->joint_velocities_.resize(impl_->joint_names_.size());
  impl_->joint_efforts_.resize(impl_->joint_names_.size());
  impl_->encoder_velocities_.resize(impl_->encoder_names_.size());

  std::vector<double> actuator_jacobian_temp = ghost_common::getVectorFromString<double>(
    sdf->GetElement(
      "actuator_jacobian")->Get<std::string>(), ' ');
  std::vector<double> sensor_jacobian_temp = ghost_common::getVectorFromString<double>(
    sdf->GetElement(
      "sensor_jacobian")->Get<std::string>(), ' ');

  // Input Validation
  if (actuator_jacobian_temp.size() != impl_->motor_names_.size() * impl_->joint_names_.size()) {
    std::string err_string =
      "[V5 Robot Plugin], Actuator Jacobian is incorrect size, cannot proceed!";
    RCLCPP_ERROR(logger, err_string.c_str());
    return;
  }

  if (sensor_jacobian_temp.size() != impl_->encoder_names_.size() * impl_->joint_names_.size()) {
    std::string err_string =
      "[V5 Robot Plugin], Sensor Jacobian is incorrect size, cannot proceed!";
    RCLCPP_ERROR(logger, err_string.c_str());
    return;
  }

  // Populate Eigen Matrices for each jacobian
  impl_->actuator_jacobian_ = Eigen::Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(
    actuator_jacobian_temp.data(), impl_->joint_names_.size(), impl_->motor_names_.size());
  impl_->sensor_jacobian_ = Eigen::Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(
    sensor_jacobian_temp.data(), impl_->joint_names_.size(), impl_->encoder_names_.size());

  std::cout << "[V5 Robot Plugin] Actuator Jacobian: \n"
            << impl_->actuator_jacobian_ << std::endl;
  std::cout << "[V5 Robot Plugin] Sensor Jacobian: \n"
            << impl_->sensor_jacobian_ << std::endl;


  std::cout << "reading motor names in constructor" << std::endl;
  // // Instantiate Motor Models and Motor Controllers for each motor
  for (const std::string & name : impl_->motor_names_) {
    //     // Get config from global definitions
    try {
      ghost_v5_config::MotorConfigStruct config =
        (ghost_v5_config::motor_config_map.at(name)).config;

      impl_->motor_model_map_[name] = std::make_shared<DCMotorModel>(
        config.motor__nominal_free_speed,
        config.motor__stall_torque,
        config.motor__free_current,
        config.motor__stall_current,
        config.motor__max_voltage,                         // TODO: max voltage is nominal voltage?
        config.motor__gear_ratio);

      impl_->motor_controller_map_[name] = std::make_shared<MotorController>(config);

      impl_->nominal_voltage_ = config.motor__max_voltage;
    } catch (const std::exception & e) {
      throw(std::runtime_error("[V5 Robot Plugin] Error: Motor Name <" + name + ">"));
    }
  }

  // Create a connection so the OnUpdate function is called at every simulation
  // iteration. Remove this call, the connection and the callback if not needed.
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&V5RobotPlugin::OnUpdate, this));

  // initialize joystick msg
  impl_->sensor_msg_.joystick_msg.joystick_left_x = 0.0;
  impl_->sensor_msg_.joystick_msg.joystick_left_y = 0.0;
  impl_->sensor_msg_.joystick_msg.joystick_right_x = 0.0;
  impl_->sensor_msg_.joystick_msg.joystick_right_y = 0.0;

  impl_->sensor_msg_.joystick_msg.btn_a = false;
  impl_->sensor_msg_.joystick_msg.btn_b = false;
  impl_->sensor_msg_.joystick_msg.btn_x = false;
  impl_->sensor_msg_.joystick_msg.btn_y = false;
  impl_->sensor_msg_.joystick_msg.btn_up = false;
  impl_->sensor_msg_.joystick_msg.btn_down = false;
  impl_->sensor_msg_.joystick_msg.btn_left = false;
  impl_->sensor_msg_.joystick_msg.btn_right = false;
  impl_->sensor_msg_.joystick_msg.btn_l1 = false;
  impl_->sensor_msg_.joystick_msg.btn_l2 = false;
  impl_->sensor_msg_.joystick_msg.btn_r1 = false;
  impl_->sensor_msg_.joystick_msg.btn_r2 = false;

  impl_->joystick_msg_ = impl_->sensor_msg_.joystick_msg;
}

void V5RobotPlugin::jointToEncoderTransform()
{
  impl_->encoder_positions_ =
    impl_->sensor_jacobian_.completeOrthogonalDecomposition().pseudoInverse() *
    impl_->joint_angles_;
  impl_->encoder_velocities_ =
    impl_->sensor_jacobian_.completeOrthogonalDecomposition().pseudoInverse() *
    impl_->joint_velocities_;
}

Eigen::VectorXd V5RobotPlugin::motorToJointTransform(const Eigen::VectorXd & motor_data)
{
  // Actuator Jacobian 6X8 rad/s
  // Motor Data 6X1 RPM
  auto joint_data = impl_->actuator_jacobian_.transpose() * motor_data * impl_->RADS_TO_RPM;

  return joint_data;
}

Eigen::VectorXd V5RobotPlugin::jointToMotorTransform(const Eigen::VectorXd & joint_data)
{
  // Actuator Jacobian 6X8
  // Joint Data 6X1
  auto motor_data = impl_->actuator_jacobian_.inverse() * joint_data;
  return motor_data;
}

void V5RobotPlugin::wrapEncoderMsg()
{
  int col_index = 0;

  for_each(
    begin(impl_->encoder_names_), end(impl_->encoder_names_), [&](
      const std::string & encoder_name) {
      auto encoder_vel_data = impl_->encoder_velocities_;
      auto encoder_pos_data = impl_->encoder_positions_;
      impl_->encoder_msg_[col_index].device_name = encoder_name;
      impl_->encoder_msg_[col_index].device_id = 1;
      impl_->encoder_msg_[col_index].device_connected = true;
      impl_->encoder_msg_[col_index].position_degrees = encoder_pos_data(col_index);
      impl_->encoder_msg_[col_index].velocity_rpm = encoder_vel_data(col_index);
      impl_->encoder_msg_[col_index].torque_nm = 0.0;
      impl_->encoder_msg_[col_index].voltage_mv = 0.0;
      impl_->encoder_msg_[col_index].current_ma = 0.0;
      impl_->encoder_msg_[col_index].power_w = 0.0;
      impl_->encoder_msg_[col_index].temp_c = 0.0;

      col_index++;
    });
}

void V5RobotPlugin::populateSensorMsg()
{
  impl_->sensor_msg_.header.stamp = impl_->clock_.now();
  impl_->sensor_msg_.header.frame_id = "base";
  impl_->sensor_msg_.msg_id = 1;
  impl_->sensor_msg_.encoders = impl_->encoder_msg_;

  // hardware parameters
  impl_->sensor_msg_.digital_port_vector[8] =
    (false, false, false, false, false, false, false, false);

  impl_->sensor_msg_.joystick_msg = impl_->joystick_msg_;
}

// Preserves order of joints listed in xacro
void V5RobotPlugin::updateJointStates()
{
  int index = 0;
  // Lamda for-each expression to get encoder values for every joint
  for_each(
    begin(impl_->joint_names_), end(impl_->joint_names_), [&](const std::string & joint_name) {
      try {
        impl_->joint_angles_(index) =
        fmod(impl_->model_->GetJoint(joint_name)->Position(2) * ghost_common::RAD_TO_DEG, 360);
        impl_->joint_velocities_(index) = impl_->model_->GetJoint(joint_name)->GetVelocity(2);
        index++;
      } catch (const std::exception & e) {
        // What type exception does GetJoint return
        throw(std::runtime_error(
          "[V5 Robot Plugin] Error: GetJoint passed non-existent joint name <" + joint_name + ">."));
      }
    });
}

void V5RobotPlugin::updateMotorController()
{
  int motor_index = 0;
  // Defined to avoid out of bounds indexing
  Eigen::VectorXd motor_torques = Eigen::VectorXd::Zero(impl_->motor_names_.size());
  Eigen::VectorXd motor_angles = this->jointToMotorTransform(impl_->joint_angles_);

  Eigen::VectorXd motor_velocities = impl_->RADS_TO_RPM * this->jointToMotorTransform(
    impl_->joint_velocities_);
  for (const std::string & name : impl_->motor_names_) {
    // Get V5 Motor Motor Model, and Motor Controller for each motor
    auto motor_model_ptr = impl_->motor_model_map_.at(name);
    auto motor_controller_ptr = impl_->motor_controller_map_.at(name);

    // Wrap angle error
    double angle_error = fmod(motor_controller_ptr->getPositionCommand(), 360) - motor_angles(
      motor_index);
    double angle_sign = std::abs(angle_error) / angle_error;
    angle_error = std::abs(angle_error) > 180 ? angle_sign * 360 - angle_error : angle_error;

    // Update motor
    motor_model_ptr->setMotorSpeedRPM(motor_velocities(motor_index));

    // Calculate control inputs
    float voltage_feedforward = motor_controller_ptr->getVoltageCommand() *
      impl_->nominal_voltage_ * 1000 * impl_->feedforward_voltage_gain_;
    float velocity_feedforward = motor_controller_ptr->getVelocityCommand() *
      impl_->feedforward_velocity_gain_;
    float velocity_feedback =
      (motor_controller_ptr->getVelocityCommand() - motor_velocities(motor_index)) *
      impl_->velocity_gain_;
    float position_feedback = (angle_error) * impl_->position_gain_;

    motor_model_ptr->setMotorEffortMillivolts(
      voltage_feedforward + velocity_feedforward + velocity_feedback + position_feedback);

    motor_torques(motor_index) = motor_model_ptr->getTorqueOutput();
    motor_index++;
  }
  impl_->joint_cmd_torques_ = this->motorToJointTransform(motor_torques);
  // TODO: is interface to publish current joint torque the same as gz joint pid plugin?
  // Nothing is publishing the output joint torques
}

void V5RobotPlugin::applySimJointTorques()
{
  int joint_index = 0;
  for (const std::string & name : impl_->joint_names_) {
    auto link = impl_->model_->GetJoint(name)->GetJointLink(0);
    // For simulatilon stability, only apply torque if larger than a threshhold
    if (std::abs(impl_->joint_cmd_torques_(joint_index)) > 1e-5) {
      link->AddRelativeTorque(
        ignition::math::v6::Vector3d(
          0.0, 0.0,
          impl_->joint_cmd_torques_(joint_index)));
      std::cout << name << " joint_torque: " << impl_->joint_cmd_torques_(joint_index) << std::endl;
    }
    joint_index++;
  }
}

void V5RobotPlugin::OnUpdate()
{
  // Acquire msg update lock
  std::unique_lock update_lock(impl_->actuator_update_callback_mutex);

  // 1) Populate Joint Position and Velocity Vectors
  // 2) Process Sensor Update
  //  2.1) Transform Joint Position/Velocity vectors into actuator/encoder states using Jacobian pseudoinverse
  //  2.3) Populate SensorUpdateMsg
  //  2.4) (Optional) Populate joystick update based on the data from joystick subscriber (That I'm gonna add!)
  //  2.5) Publish
  // 3) Process Actuator Update
  //  3.1) Transform Joint Position/Velocity vectors into actuator space using Jacobian pseudoinverse
  //  3.2) For each motor controller, update position and velocity and async gets setpoints
  //  3.3) For each motor controller, calculate the motor effort (voltage) based on error from 3.2
  //  3.4) For each motor model update velocity from sim
  //  3.5) For each motor model, update the motor effort (voltage)
  //  3.6) Concatenate the simulated torques from each motor model into a vector
  //  3.7) Transform actuator torques to joint torques using  using Jacobian transpose
  //  3.8) Update each joint in gazebo

  this->updateJointStates();

  this->updateMotorController();

  this->applySimJointTorques();

  this->jointToEncoderTransform();

  this->wrapEncoderMsg();

  this->populateSensorMsg();

  impl_->sensor_update_pub_->publish(impl_->sensor_msg_);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(V5RobotPlugin)

}// namespace v5_robot_plugin
