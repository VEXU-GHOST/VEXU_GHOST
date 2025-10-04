#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ghost_tank/tank_model.hpp"

namespace BT_Util {
    
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

template<typename T>
T get_input(BT::TreeNode* node, std::string key, T default_value)
{
  BT::Expected<T> input = node->getInput<T>(key);
  // Check if expected is valid. If not, return default value
  if (!input) {
    return default_value;
  }
  return input.value();
}

template<typename T>
bool get_from_blackboard(BT::Blackboard::Ptr blackboard, std::string key, T &value){
    if(!blackboard->get(key, value)){
        std::cout << key << " not found in blackboard" << std::endl;
        return false;
    }
    return true;
}

template<typename T>
T get_from_blackboard(BT::Blackboard::Ptr blackboard, std::string key){
  T value;
    if(!blackboard->get(key, value)){
        std::cout << key << " not found in blackboard" << std::endl;
    }
    return value;
}

template<typename T>
void get_from_blackboard(BT::Blackboard::Ptr blackboard, std::string key, T &value, T default_value){
    if(!blackboard->get(key, value)){
        std::cout << key << " not found in blackboard" << std::endl;
        value = default_value;
    }
}

template<typename T>
void put_in_blackboard(BT::Blackboard::Ptr blackboard, std::string key, T value){
    blackboard->set<T>(key, value);
}

} // namespace BT_Util