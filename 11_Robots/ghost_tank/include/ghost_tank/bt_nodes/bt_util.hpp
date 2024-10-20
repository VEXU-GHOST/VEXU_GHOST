#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ghost_tank/tank_model.hpp"

namespace BT_Util {
    
// template <typename T>
// T get_input(std::string key);

template<typename T>
T get_input(BT::TreeNode* node, std::string key)
{
  BT::Expected<T> input = node->getInput<T>(key);
  // Check if expected is valid. If not, throw its error
  if (!input) {
    throw BT::RuntimeError(
            "missing required input [" + key + "]: ",
            input.error() );
  }
  return input.value();
}


} // namespace ghost_tank
