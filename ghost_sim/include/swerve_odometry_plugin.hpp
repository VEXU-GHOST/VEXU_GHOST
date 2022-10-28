/*
 * Filename: odometry_plugin
 * Created Date: Monday October 24th 2022
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 * 
 * Last Modified: Monday October 24th 2022 2:19:16 pm
 * Modified By: Maxx Wilson
 */

#ifndef GHOST_SIM__SWERVE_ODOMETRY_PLUGIN_HPP
#define GHOST_SIM__SWERVE_ODOMETRY_PLUGIN_HPP
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
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace swerve_odometry_plugin
{
// Forward declaration of private data class.
class SwerveOdometryPluginPrivate;

class SwerveOdometryPlugin : public gazebo::ModelPlugin
{
public:
  /// Constructor
  SwerveOdometryPlugin();

  /// Destructor
  ~SwerveOdometryPlugin();

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
  std::unique_ptr<SwerveOdometryPluginPrivate> impl_;
};
}  // namespace swerve_odometry_plugin

#endif  // GHOST_SIM__SWERVE_ODOMETRY_PLUGIN_HPP