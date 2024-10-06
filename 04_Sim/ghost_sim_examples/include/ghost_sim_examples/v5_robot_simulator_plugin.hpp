#pragma once
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo_ros/node.hpp>

namespace gazebo
{

class V5RobotSimulatorPluginPrivate;

class V5RobotSimulatorPlugin : public gazebo::ModelPlugin
{
public:
  V5RobotSimulatorPlugin();
  ~V5RobotSimulatorPlugin() override = default;

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

  void OnUpdate();

protected:
  std::unique_ptr<V5RobotSimulatorPluginPrivate> impl_;
};
} // namespace gazebo
