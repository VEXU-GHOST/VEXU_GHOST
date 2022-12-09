#include <string>
#include <rclcpp/rclcpp.hpp>

#include "particle_filter/particle_filter_node.hpp"

namespace ghost_modules {

// Entry point for particle filter thread
void ghost_particle_filter_main(std::string config_file);

} // namespace ghost_modules