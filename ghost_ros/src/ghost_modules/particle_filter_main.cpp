#include "ghost_modules/particle_filter_main.hpp"

namespace ghost_modules {

// Particle Filter Main Thread
void ghost_particle_filter_main(std::string config_file){
    rclcpp::spin(std::make_shared<particle_filter::ParticleFilterNode>(config_file));
}

} // namespace ghost_modules