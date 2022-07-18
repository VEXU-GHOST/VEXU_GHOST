/*
 * Filename: gazebo_swerve_plugin
 * Created Date: Monday July 18th 2022
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 * 
 * Last Modified: Monday July 18th 2022 11:31:06 am
 * Modified By: Maxx Wilson
 */

#include <gazebo/physics/Model.hh>
#include <gazebo_plugins/gazebo_ros_template.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace gazebo_plugins
{
/// Class to hold private data members (PIMPL pattern)
class GazeboRosTemplatePrivate
{
public:
  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_;

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;
};

GazeboRosTemplate::GazeboRosTemplate()
: impl_(std::make_unique<GazeboRosTemplatePrivate>())
{
}

GazeboRosTemplate::~GazeboRosTemplate()
{
}

void GazeboRosTemplate::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Create a GazeboRos node instead of a common ROS node.
  // Pass it SDF parameters so common options like namespace and remapping
  // can be handled.
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  // The model pointer gives you direct access to the physics object,
  // for example:
  RCLCPP_INFO(impl_->ros_node_->get_logger(), model->GetName().c_str());

  // Create a connection so the OnUpdate function is called at every simulation
  // iteration. Remove this call, the connection and the callback if not needed.
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosTemplate::OnUpdate, this));
}

void GazeboRosTemplate::OnUpdate()
{
  // Do something every simulation iteration
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosTemplate)
}  // namespace gazebo_plugins