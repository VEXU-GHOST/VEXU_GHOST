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
#include <gazebo/physics/Link.hh>
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

  /// Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr wheel_input_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr steering_input_sub_;

  /// Publishers
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr wheel_torque_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr steering_torque_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr wheel_speed_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr steering_speed_pub_;

  /// Latest Actuator Values
  std::vector<double> actuator_inputs_;
  std::vector<std::pair<std::string, std::string>> joint_link_indexes_;
  std::vector<dc_motor_model::DCMotorModel> actuators_;

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

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  // Initialize Subscriptions
  impl_->wheel_input_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Vector3>(
    "wheel_voltage_input",
    qos.get_subscription_qos("gazebo_swerve_plugin", rclcpp::SystemDefaultsQoS()),
    std::bind(
      &GazeboSwervePlugin::OnWheelMsg,
      this,
      std::placeholders::_1
      )
    );

  impl_->steering_input_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Vector3>(
    "steering_voltage_input",
    qos.get_subscription_qos("gazebo_swerve_plugin", rclcpp::SystemDefaultsQoS()),
    std::bind(
      &GazeboSwervePlugin::OnSteeringMsg,
      this,
      std::placeholders::_1
      )
    );

  // Initialize Publisher
  impl_->wheel_torque_pub_ = impl_->ros_node_->create_publisher<geometry_msgs::msg::Vector3>(
    "wheel_torque_output",
    qos.get_subscription_qos("gazebo_swerve_plugin", rclcpp::SystemDefaultsQoS()));

  impl_->steering_torque_pub_ = impl_->ros_node_->create_publisher<geometry_msgs::msg::Vector3>(
    "steering_torque_output", 
    qos.get_subscription_qos("gazebo_swerve_plugin", rclcpp::SystemDefaultsQoS()));

  impl_->wheel_speed_pub_ = impl_->ros_node_->create_publisher<geometry_msgs::msg::Vector3>(
    "wheel_speed_output",
    qos.get_subscription_qos("gazebo_swerve_plugin", rclcpp::SystemDefaultsQoS()));

  impl_->steering_speed_pub_ = impl_->ros_node_->create_publisher<geometry_msgs::msg::Vector3>(
    "steering_speed_output", 
    qos.get_subscription_qos("gazebo_swerve_plugin", rclcpp::SystemDefaultsQoS()));

  // Initalize motor models
  impl_->actuators_ = std::vector<dc_motor_model::DCMotorModel>{
    dc_motor_model::DCMotorModel(130, 4.5, 0.1, 4.5, 12.8),
    dc_motor_model::DCMotorModel(130, 4.5, 0.1, 4.5, 12.8),
    dc_motor_model::DCMotorModel(130, 4.5, 0.1, 4.5, 12.8),
    dc_motor_model::DCMotorModel(130, 4.5, 0.1, 4.5, 12.8),
    dc_motor_model::DCMotorModel(130, 4.5, 0.1, 4.5, 12.8),
    dc_motor_model::DCMotorModel(130, 4.5, 0.1, 4.5, 12.8),
  };

  impl_->actuator_inputs_ = std::vector<double>(6,0.0);

  impl_->joint_link_indexes_ = std::vector<std::pair<std::string, std::string>>{
    std::pair("driveshaft_1", "wheel_1"),
    std::pair("driveshaft_2", "wheel_2"),
    std::pair("driveshaft_3", "wheel_3"),
    std::pair("pivot_1", "mod_1"),
    std::pair("pivot_2", "mod_2"),
    std::pair("pivot_3", "mod_3"),
  };

  // Set Gear Ratios
  // impl_->actuators_[0].setGearRatio(6.0);
  // impl_->actuators_[1].setGearRatio(6.0);
  // impl_->actuators_[2].setGearRatio(6.0);

  // Create a connection so the OnUpdate function is called at every simulation
  // iteration. Remove this call, the connection and the callback if not needed.
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboSwervePlugin::OnUpdate, this));
}

void GazeboSwervePlugin::OnUpdate()
{
  // Update Actuators from last torque command
  for(std::size_t i = 0; i < impl_->actuator_inputs_.size(); i++){
    std::string joint_name = impl_->joint_link_indexes_[i].first;
    std::string link_name = impl_->joint_link_indexes_[i].second;

    impl_->actuators_[i].setMotorEffort(impl_->actuator_inputs_[i]);
    impl_->actuators_[i].setMotorSpeedRad(impl_->model_->GetJoint(joint_name)->GetVelocity(2));
    // FYI: SetTorque method totally breaks rolling of wheels, use AddRelativeTorque
    impl_->model_->GetLink(link_name)->AddRelativeTorque(ignition::math::v6::Vector3d(0.0, 0.0, impl_->actuators_[i].getTorqueOutput()));
  }

  // Publish current joint torques
  auto wheel_torque_msg = geometry_msgs::msg::Vector3{};
  wheel_torque_msg.x = impl_->actuators_[0].getTorqueOutput();
  wheel_torque_msg.y = impl_->actuators_[1].getTorqueOutput();
  wheel_torque_msg.z = impl_->actuators_[2].getTorqueOutput();
  
  impl_->wheel_torque_pub_->publish(wheel_torque_msg);

  auto steering_torque_msg = geometry_msgs::msg::Vector3{};
  steering_torque_msg.x = impl_->actuators_[3].getTorqueOutput();
  steering_torque_msg.y = impl_->actuators_[4].getTorqueOutput();
  steering_torque_msg.z = impl_->actuators_[5].getTorqueOutput();

  impl_->steering_torque_pub_->publish(steering_torque_msg);

  // Publish current joint speeds (RPM)
  auto wheel_speed_msg = geometry_msgs::msg::Vector3{};
  wheel_speed_msg.x = impl_->model_->GetJoint("driveshaft_1")->GetVelocity(2)*60/(2*3.14159);
  wheel_speed_msg.y = impl_->model_->GetJoint("driveshaft_2")->GetVelocity(2)*60/(2*3.14159);
  wheel_speed_msg.z = impl_->model_->GetJoint("driveshaft_3")->GetVelocity(2)*60/(2*3.14159);
  
  impl_->wheel_speed_pub_->publish(wheel_speed_msg);

  auto steering_speed_msg = geometry_msgs::msg::Vector3{};
  steering_speed_msg.x = impl_->model_->GetJoint("pivot_1")->GetVelocity(2)*60/(2*3.14159);
  steering_speed_msg.y = impl_->model_->GetJoint("pivot_2")->GetVelocity(2)*60/(2*3.14159);
  steering_speed_msg.z = impl_->model_->GetJoint("pivot_3")->GetVelocity(2)*60/(2*3.14159);

  impl_->steering_speed_pub_->publish(steering_speed_msg);

}

void GazeboSwervePlugin::OnWheelMsg(const geometry_msgs::msg::Vector3::SharedPtr msg){
  impl_->actuator_inputs_[0] = msg->x;
  impl_->actuator_inputs_[1] = msg->y;
  impl_->actuator_inputs_[2] = msg->z;
}

void GazeboSwervePlugin::OnSteeringMsg(const geometry_msgs::msg::Vector3::SharedPtr msg){
  impl_->actuator_inputs_[3] = msg->x;
  impl_->actuator_inputs_[4] = msg->y;
  impl_->actuator_inputs_[5] = msg->z;
}



// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboSwervePlugin)
}  // namespace gazebo_swerve_plugin