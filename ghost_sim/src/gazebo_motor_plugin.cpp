/*
 * Filename: gazebo_swerve_plugin
 * Created Date: Monday July 18th 2022
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 * 
 * Last Modified: Saturday September 10th 2022 10:50:34 am
 * Modified By: Maxx Wilson
 */

#include "gazebo_motor_plugin.hpp"
#include "ghost_control/models/dc_motor_model.hpp"

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <vector>
#include <memory>


namespace gazebo_motor_plugin
{
/// Class to hold private data members (PIMPL pattern)
class GazeboMotorPluginPrivate
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
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr input_voltage_sub_;

  /// Publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr output_torque_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr output_speed_pub_;

  /// Latest Actuator Values
  double actuator_input_;
  dc_motor_model::DCMotorModel motor_model_;

  // Plugin params
  std::string joint_name_;
  std::string link_name_;
  
  double free_speed_;
  double stall_torque_;
  double free_current_;
  double stall_current_;
  double nominal_voltage_;
  double gear_ratio_;
};

GazeboMotorPlugin::GazeboMotorPlugin()
: impl_(std::make_unique<GazeboMotorPluginPrivate>())
{
}

GazeboMotorPlugin::~GazeboMotorPlugin()
{
}

void GazeboMotorPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Get ROS Node and Gazebo Model Ptr
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
  auto logger = impl_->ros_node_->get_logger();
  impl_->model_= model;

  // Check for valid plugin config
  std::vector<std::string> params{
    "joint_name",
    "link_name",
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
  
  impl_->free_speed_ = sdf->GetElement("free_speed")->Get<double>();
  impl_->stall_torque_ = sdf->GetElement("stall_torque")->Get<double>();
  impl_->free_current_ = sdf->GetElement("free_current")->Get<double>();
  impl_->stall_current_ = sdf->GetElement("stall_current")->Get<double>();
  impl_->nominal_voltage_ = sdf->GetElement("nominal_voltage")->Get<double>();
  impl_->gear_ratio_ = sdf->GetElement("gear_ratio")->Get<double>();

  // Initialize Subscriptions
  impl_->input_voltage_sub_ = impl_->ros_node_->create_subscription<std_msgs::msg::Float32>(
    "motors/" + impl_->link_name_ + "/input_voltage",
    10,
    [this](const std_msgs::msg::Float32::SharedPtr msg){
      impl_->actuator_input_ = msg->data;
    }
    );

  // Initialize Publisher
  impl_->output_torque_pub_ = impl_->ros_node_->create_publisher<std_msgs::msg::Float32>(
    "motors/" + impl_->link_name_ + "/output_torque",
    10);

  impl_->output_speed_pub_ = impl_->ros_node_->create_publisher<std_msgs::msg::Float32>(
    "motors/" + impl_->link_name_ + "/output_speed",
    10);

  // Initalize motor models
  impl_->motor_model_ = dc_motor_model::DCMotorModel(
    impl_->free_speed_,
    impl_->stall_torque_,
    impl_->free_current_,
    impl_->stall_current_,
    impl_->nominal_voltage_
    );
  impl_->actuator_input_ = 0.0;

  // Set Gear Ratios
  impl_->motor_model_.setGearRatio(impl_->gear_ratio_);

  // Joint and Link Ptrs
  impl_->joint_ = impl_->model_->GetJoint(impl_->joint_name_);
  impl_->link_ = impl_->model_->GetLink(impl_->link_name_);

  // Create a connection so the OnUpdate function is called at every simulation
  // iteration. Remove this call, the connection and the callback if not needed.
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboMotorPlugin::OnUpdate, this));
}

void GazeboMotorPlugin::OnUpdate()
{
  // Update Actuators from last torque command
  impl_->motor_model_.setMotorEffort(impl_->actuator_input_);
  impl_->motor_model_.setMotorSpeedRad(impl_->joint_->GetVelocity(2));
  impl_->link_->AddRelativeTorque(ignition::math::v6::Vector3d(0.0, 0.0, impl_->motor_model_.getTorqueOutput()));

  // Publish current joint torque
  auto torque_msg = std_msgs::msg::Float32{};
  torque_msg.data = impl_->motor_model_.getTorqueOutput();
  impl_->output_torque_pub_->publish(torque_msg);

  // Publish current joint speed (RPM)
  auto speed_msg = std_msgs::msg::Float32{};
  speed_msg.data = impl_->joint_->GetVelocity(2)*60/(2*3.14159);
  impl_->output_speed_pub_->publish(speed_msg);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboMotorPlugin)
}  // namespace gazebo_swerve_plugin