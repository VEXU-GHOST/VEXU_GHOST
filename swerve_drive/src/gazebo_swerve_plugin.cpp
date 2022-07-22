/*
 * Filename: gazebo_swerve_plugin
 * Created Date: Monday July 18th 2022
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 * 
 * Last Modified: Monday July 18th 2022 11:30:35 am
 * Modified By: Maxx Wilson
 */

#include "gazebo_swerve_plugin.hpp"
#include "dc_motor_model.hpp"

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <vector>
#include <memory>


namespace gazebo_swerve_plugin
{
/// Class to hold private data members (PIMPL pattern)
class GazeboSwervePluginPrivate
{
public:
  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_;

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Motor Input Subscriber
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr input_sub_;

  /// Motor Output Publisher
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr output_sub_;

  std::map<std::string, double> actuator_inputs_;
  std::map<std::string, dc_motor_model::DCMotorModel> actuators_;

  gazebo::physics::ModelPtr model_;
};

GazeboSwervePlugin::GazeboSwervePlugin()
: impl_(std::make_unique<GazeboSwervePluginPrivate>())
{
}

GazeboSwervePlugin::~GazeboSwervePlugin()
{
}

void GazeboSwervePlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{

  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
  auto logger = impl_->ros_node_->get_logger();
  impl_->model_= model;

  if(!sdf->HasElement("input_topic")){
    RCLCPP_ERROR(logger, "Swerve plugin missing <input_topic>, cannot proceed");
    return;
  }

  if(!sdf->HasElement("output_topic")){
    RCLCPP_ERROR(logger, "Swerve plugin missing <output_topic>, cannot proceed");
    return;
  }

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  // Initialize Subscriptions
  impl_->input_sub_ = impl_->ros_node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    sdf->GetElement("input_topic")->Get<std::string>(),
    qos.get_subscription_qos("gazebo_swerve_plugin", rclcpp::SystemDefaultsQoS()),
    std::bind(
      &GazeboSwervePlugin::OnInputMsg,
      this,
      std::placeholders::_1
      )
    );

  // Initialize Publisher
  impl_->output_sub_ = impl_->ros_node_->create_publisher<std_msgs::msg::Float32MultiArray>(
    sdf->GetElement("input_topic")->Get<std::string>(), 
    qos.get_subscription_qos("gazebo_swerve_plugin", rclcpp::SystemDefaultsQoS()));

  // Initalize motor models
  impl_->actuators_ = std::map<std::string, dc_motor_model::DCMotorModel>{
    {"pivot_1", dc_motor_model::DCMotorModel(130, 4.5, -0.73, 4.5, 12.8, 100, 2.5)},
    {"pivot_2", dc_motor_model::DCMotorModel(130, 4.5, -0.73, 4.5, 12.8, 100, 2.5)},
    {"pivot_3", dc_motor_model::DCMotorModel(130, 4.5, -0.73, 4.5, 12.8, 100, 2.5)},
    {"driveshaft_1", dc_motor_model::DCMotorModel(130, 4.5, -0.73, 4.5, 12.8, 100, 2.5)},
    {"driveshaft_2", dc_motor_model::DCMotorModel(130, 4.5, -0.73, 4.5, 12.8, 100, 2.5)},
    {"driveshaft_3", dc_motor_model::DCMotorModel(130, 4.5, -0.73, 4.5, 12.8, 100, 2.5)},
  };

  impl_->actuator_inputs_ = std::map<std::string, double>{
    {"pivot_1", 0.0},
    {"pivot_2", 0.0},
    {"pivot_3", 0.0},
    {"driveshaft_1", 0.0},
    {"driveshaft_2", 0.0},
    {"driveshaft_3", 0.0},
  };

  // Set Gear Ratios
  impl_->actuators_["driveshaft_1"].setGearRatio(6.0);
  impl_->actuators_["driveshaft_2"].setGearRatio(6.0);
  impl_->actuators_["driveshaft_3"].setGearRatio(6.0);

  // Create a connection so the OnUpdate function is called at every simulation
  // iteration. Remove this call, the connection and the callback if not needed.
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboSwervePlugin::OnUpdate, this));
}

void GazeboSwervePlugin::OnUpdate()
{

  // Update Actuators from last torque command
  for(const auto& pair : impl_->actuators_){
    std::string joint_name = pair.first;
    impl_->actuators_[joint_name].setMotorInput(
      impl_->actuator_inputs_[joint_name],
      impl_->model_->GetJoint(joint_name)->GetVelocity(2)
    );

    impl_->model_->GetJoint(joint_name)->SetForce(0, impl_->actuators_[joint_name].getTorqueOutput());
  }

  // Publish current joint torques
  auto msg = std_msgs::msg::Float32MultiArray{};
  msg.data = std::vector<float>(6, 0.0);
  msg.data[0] = impl_->actuator_inputs_["pivot_1"];
  msg.data[1] = impl_->actuator_inputs_["pivot_2"];
  msg.data[2] = impl_->actuator_inputs_["pivot_3"];
  msg.data[3] = impl_->actuator_inputs_["driveshaft_1"];
  msg.data[4] = impl_->actuator_inputs_["driveshaft_2"];
  msg.data[5] = impl_->actuator_inputs_["driveshaft_3"];
}

void GazeboSwervePlugin::OnInputMsg(const std_msgs::msg::Float32MultiArray::SharedPtr msg){

  impl_->actuator_inputs_["pivot_1"] = msg->data[0];
  impl_->actuator_inputs_["pivot_2"] = msg->data[1];
  impl_->actuator_inputs_["pivot_3"] = msg->data[2];
  impl_->actuator_inputs_["driveshaft_1"] = msg->data[3];
  impl_->actuator_inputs_["driveshaft_2"] = msg->data[4];
  impl_->actuator_inputs_["driveshaft_3"] = msg->data[5];

}



// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboSwervePlugin)
}  // namespace gazebo_swerve_plugin