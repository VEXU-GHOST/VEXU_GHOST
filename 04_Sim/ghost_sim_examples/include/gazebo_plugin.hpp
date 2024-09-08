#ifndef GHOST_SIM__EXAMPLES_GAZEBO_PLUGIN_HPP_
#define GHOST_SIM__EXAMPLES_GAZEBO_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>
#include "std_msgs/msg/float32.hpp"

// For std::unique_ptr, could be removed
#include <memory>

namespace gazebo
{

// Forward declaration of private data class.
class TankMovePrivate;

class TankMove : public gazebo::ModelPlugin
{
public:
  /// Constructor
  TankMove();

  /// Destructor
  ~TankMove();

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

protected:
  /// Optional callback to be called at every simulation iteration.
  void OnUpdate();

private:
  /// Recommended PIMPL pattern. This variable should hold all private
  /// data members.
  std::unique_ptr<TankMovePrivate> impl_;
};

}  

#endif  
