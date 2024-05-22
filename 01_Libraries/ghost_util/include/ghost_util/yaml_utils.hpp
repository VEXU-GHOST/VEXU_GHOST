#include <iostream>
#include "yaml-cpp/yaml.h"

namespace ghost_util
{

template<typename T>
bool loadYAMLParam(YAML::Node node, std::string param_name, T & var, bool verbose)
{
  if (node[param_name]) {
    var = node[param_name].as<T>();
    if (verbose) {
      std::cout << "Loaded <" << param_name << "> with value " << var << std::endl;
    }
    return true;
  } else if (verbose) {
    std::cout << "Failed to Load <" + param_name << ">" << std::endl;
  }
  return false;
}

} // namespace ghost_util
