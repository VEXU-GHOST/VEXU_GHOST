/*
 * Filename: gazebo_motor_plugin
 * Created Date: Sunday August 7th 2022
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 * 
 * Last Modified: Sunday August 7th 2022 2:00:44 pm
 * Modified By: Maxx Wilson
 */

#ifndef SWERVE_DRIVE__GAZEBO_MOTOR_PLUGIN_HPP_
#define SWERVE_DRIVE__GAZEBO_MOTOR_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>
#include "std_msgs/msg/float32.hpp"

// For std::unique_ptr, could be removed
#include <memory>

namespace gazebo_motor_plugin
{
// Forward declaration of private data class.
class GazeboMotorPluginPrivate;

class GazeboMotorPlugin : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboMotorPlugin();

  /// Destructor
  ~GazeboMotorPlugin();

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
  std::unique_ptr<GazeboMotorPluginPrivate> impl_;
};
}  // namespace gazebo_motor_plugin

#endif  // SWERVE_DRIVE__GAZEBO_MOTOR_PLUGIN_HPP_