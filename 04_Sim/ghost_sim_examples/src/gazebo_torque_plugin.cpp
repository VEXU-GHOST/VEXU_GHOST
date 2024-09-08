
// #include "ghost_sim_examples/include/gazebo_plugin.hpp"
#include <mutex>
#include <unordered_map>
#include "eigen3/Eigen/Geometry"

#include "ghost_msgs/msg/"
#include "ghost_common/util/angle_util.hpp"
#include "ghost_common/util/parsing_util.hpp"
#include "ghost_msgs/msg/v5_actuator_command.hpp"
#include "ghost_common/v5_robot_config_defs.hpp"
#include "ghost_control/models/dc_motor_model.hpp"
#include "ghost_control/motor_controller.hpp"
#include "gazebo_plugin.hpp"
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <vector>

namespace gazebo
{
  class TankMovePrivate
  {
    public: 
    TankMovePrivate()

        {
        }

    // Pointer to the update event connection
    gazebo::event::ConnectionPtr updateConnection;


    gazebo::physics::ModelPtr model_;

    std::vector<std::string> motor_names_;
    std::vector<std::string> joint_names_;





  // Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;



 Eigen::VectorXd joint_angles_;
  Eigen::VectorXd joint_velocities_;
  Eigen::VectorXd joint_efforts_;
  Eigen::VectorXd joint_cmd_torques_;

  // Incoming high level motor state data
  Eigen::VectorXd motor_positions_;
  Eigen::VectorXd motor_velocities_;
  Eigen::VectorXd motor_efforts_;

  // Motor Controller Gains
  double feedforward_voltage_gain_ = 1.0;
  double feedforward_velocity_gain_ = 1.0;
  double velocity_gain_ = 15.0;
  double position_gain_ = 0.0;
  double nominal_voltage_;


  std::unordered_map<std::string, std::shared_ptr<DCMotorModel>> motor_model_map_;
  std::unordered_map<std::string, std::shared_ptr<MotorController>> motor_controller_map_;

  Eigen::MatrixXd actuator_jacobian_;



 rclcpp::Subscription<ghost_msgs::msg::V5ActuatorCommand>::SharedPtr actuator_command_sub_;




  };

 TankMove::TankMove()
: impl_(std::make_unique<TankMovePrivate>())
{
}


TankMove::~TankMove()
{
}


  
    void TankMove::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
    {
         impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
        auto logger = impl_->ros_node_->get_logger();

      // Store the pointer to the model
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

        // Update Current Limit- Do I need this?
        //motor_model_ptr->setMaxCurrent(motor_cmd_msg.current_limit);
      }
    });


std::vector<std::string> params{
    "joint_names",
    "motor_names",
    "actuator_jacobian",
    "free_speed",
    "stall_torque",
    "free_current",
    "stall_current",
    "nominal_voltage",
    "gear_ratio"    
    };

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

  // Define eigen vector sizes using number of joints
  impl_->joint_angles_.resize(impl_->joint_names_.size());
  impl_->joint_velocities_.resize(impl_->joint_names_.size());
  impl_->joint_efforts_.resize(impl_->joint_names_.size());

  std::vector<double> actuator_jacobian_temp = ghost_common::getVectorFromString<double>(
    sdf->GetElement(
      "actuator_jacobian")->Get<std::string>(), ' ');
  

  // Input Validation
  if (actuator_jacobian_temp.size() != impl_->motor_names_.size() * impl_->joint_names_.size()) {
    std::string err_string =
      "[V5 Robot Plugin], Actuator Jacobian is incorrect size, cannot proceed!";
    RCLCPP_ERROR(logger, err_string.c_str());
    return;
  }


  // Populate Eigen Matrices for each jacobian
  impl_->actuator_jacobian_ = Eigen::Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(
    actuator_jacobian_temp.data(), impl_->joint_names_.size(), impl_->motor_names_.size());


  std::cout << "[V5 Robot Plugin] Actuator Jacobian: \n"
            << impl_->actuator_jacobian_ << std::endl;

  std::cout << "reading motor names in constructor" << std::endl;
  // // Instantiate Motor Models and Motor Controllers for each motor
  for (const std::string & name : impl_->motor_names_) {
    //     // Get config from global definitions
    try {
      ghost_v5_config::MotorConfigStruct config =
        (ghost_v5_config::motor_config_map.at(name)).config;

      impl_->motor_model_map_[name] = std::make_shared<DCMotorModel>(
        impl_->free_speed_,
        impl_->stall_torque_,
        impl_->free_current_,
        impl_->stall_current_,
        impl_->nominal_voltage_.
        impl_->gear_ratio
);

      impl_->motor_controller_map_[name] = std::make_shared<MotorController>(config);

      impl_->nominal_voltage_ = config.motor__max_voltage;
    } catch (const std::exception & e) {
      throw(std::runtime_error("[V5 Robot Plugin] Error: Motor Name <" + name + ">"));
    }
  }



  impl_->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&TankMove::OnUpdate, this));


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


    void TankMove::OnUpdate()
    {

 // Acquire msg update lock
 std::unique_lock update_lock(impl_->actuator_update_callback_mutex);

  this->updateJointStates();

  this->updateMotorController();

  this->applySimJointTorques();



//   for (const std::string & name : impl_->motor_names_) {
//     // Get V5 Motor Motor Model, and Motor Controller for each motor
//     auto motor_model_ptr = impl_->motor_model_map_.at(name);
//     auto motor_controller_ptr = impl_->motor_controller_map_.at(name);
//     auto joint_ptr = impl_->joint_map_.at(joint_name);


//  double position= impl_->joint_ptr->Position(2);
  
//  double velocity = impl_->joint_ptr->GetVelocity(2) * 60 / (2 * 3.14159);

//  double command_voltage = impl_->motor_controller_ptr.updateMotor(position, velocity);

// impl_->motor_model_ptr.setMotorEffort(command_voltage/12000);



// // Publish current joint torque for left side
//   auto torque_msg = std_msgs::msg::Float32{};
//   torque_msg.data = impl_->motor_model_ptr.getTorqueOutput()*4;
//   impl_->output_torque_pub_->publish(torque_msg);

// // Publish current joint speed (RPM) for left side
//   auto speed_msg = std_msgs::msg::Float32{};
//   speed_msg.data = impl_->joint_ptr->GetVelocity(2) * 60 / (2 * 3.14159);
//   impl_->output_speed_pub_->publish(speed_msg);

// //Publish position for left side
//   auto pos_msg = std_msgs::msg::Float32{};
//   pos_msg.data = impl_->joint_ptr->Position(2);
//   impl_->position_pub_->publish(pos_msg);
//   }




    }


  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(TankMove)
}