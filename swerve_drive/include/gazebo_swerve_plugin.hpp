/*
 * Filename: gazebo_swerve_plugin
 * Created Date: Monday July 18th 2022
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 * 
 * Last Modified: Monday July 18th 2022 11:31:06 am
 * Modified By: Maxx Wilson
 */

#ifndef SWERVE_DRIVE__GAZEBO_SWERVE_PLUGIN_HPP_
#define SWERVE_DRIVE__GAZEBO_SWERVE_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>
#include <std_msgs/msg/float32_multi_array.hpp>

// For std::unique_ptr, could be removed
#include <memory>

namespace gazebo_swerve_plugin
{
// Forward declaration of private data class.
class GazeboSwervePluginPrivate;

class GazeboSwervePlugin : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboSwervePlugin();

  /// Destructor
  ~GazeboSwervePlugin();

  /// Gazebo calls this when the plugin is loaded.
  /// \param[in] model Pointer to parent model. Other plugin types will expose different entities,
  /// such as `gazebo::sensors::SensorPtr`, `gazebo::physics::WorldPtr`,
  /// `gazebo::rendering::VisualPtr`, etc.
  /// \param[in] sdf SDF element containing user-defined parameters.
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

protected:
  /// Optional callback to be called at every simulation iteration.
  void OnUpdate();

  void OnInputMsg(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

private:
  /// Recommended PIMPL pattern. This variable should hold all private
  /// data members.
  std::unique_ptr<GazeboSwervePluginPrivate> impl_;
};
}  // namespace swerve_drive

#endif  // SWERVE_DRIVE__GAZEBO_SWERVE_PLUGIN_HPP_