#include <ghost_sim_examples/v5_robot_simulator_plugin.hpp>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo/gazebo.hh>
#include <iostream>

namespace gazebo
{

class V5RobotSimulatorPluginPrivate
{
public:
  V5RobotSimulatorPluginPrivate() = default;
  gazebo::event::ConnectionPtr updateConnection;
  gazebo::physics::ModelPtr model_;
  gazebo_ros::Node::SharedPtr ros_node_;
};


V5RobotSimulatorPlugin::V5RobotSimulatorPlugin()
// : impl_(std::make_unique<V5RobotSimulatorPluginPrivate>())
{
}

void V5RobotSimulatorPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
//   std::cout << "[V5RobotSimulatorPlugin::Load]" << std::endl;
//   impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
//   impl_->model_ = model;
//   impl_->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
//     std::bind(&V5RobotSimulatorPlugin::OnUpdate, this));
}

void V5RobotSimulatorPlugin::OnUpdate()
{
  std::cout << "[V5RobotSimulatorPlugin::OnUpdate]" << std::endl;
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(V5RobotSimulatorPlugin)
} // namespace gazebo
