#include "v5_sim_node.hpp"

int main(int argc, char * argv[]){

  rclcpp::init(argc, argv);

  // Create and run V5SimNode until program interruption or termination
  auto repo_base_dir = std::string(getenv("HOME")) + "/VEXU_GHOST/";
  rclcpp::spin(std::make_shared<v5_sim_node::V5SimNode>(repo_base_dir + "ghost_sim/config/v5_sim_config.yaml"));

  rclcpp::shutdown();
  return 0;

}