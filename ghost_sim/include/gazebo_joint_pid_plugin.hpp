/*
 * Filename: gazebo_joint_pid_plugin
 * Created Date: Sunday August 7th 2022
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 * 
 * Last Modified: Saturday September 10th 2022 10:50:34 am
 * Modified By: Maxx Wilson
 */

#ifndef GHOST_SIM__GAZEBO_JOINT_PID_PLUGIN_HPP_
#define GHOST_SIM__GAZEBO_JOINT_PID_PLUGIN_HPP_

#include <math.h>
#include <vector>
#include <memory>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "ghost_control/models/dc_motor_model.hpp"

namespace gazebo_joint_pid_plugin
{
// Forward declaration of private data class.
class GazeboJointPIDPluginPrivate;

class GazeboJointPIDPlugin : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboJointPIDPlugin();

  /// Destructor
  ~GazeboJointPIDPlugin();

  /// Gazebo calls this when the plugin is loaded.
  /// \param[in] model Pointer to parent model. Other plugin types will expose different entities,
  /// such as `gazebo::sensors::SensorPtr`, `gazebo::physics::WorldPtr`,
  /// `gazebo::rendering::VisualPtr`, etc.
  /// \param[in] sdf SDF element containing user-defined parameters.
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

protected:
  /// Optional callback to be called at every simulation iteration.
  void OnUpdate();

private:
  /// Recommended PIMPL pattern. This variable should hold all private
  /// data members.
  std::unique_ptr<GazeboJointPIDPluginPrivate> impl_;
};
}  // namespace gazebo_joint_pid_plugin

#endif  // GHOST_SIM__GAZEBO_JOINT_PID_PLUGIN_HPP_