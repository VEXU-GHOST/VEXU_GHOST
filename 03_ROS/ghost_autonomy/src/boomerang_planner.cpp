#include "ghost_autonomy/boomerang_planner.hpp"

namespace ghost_autonomy
{
BoomerangPlanner::BoomerangPlanner()
    :rclcpp::Node("boomerang_planner"){

}

} // namespace ghost_autonomy

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto boomerang_planner = std::make_shared<ghost_autonomy::BoomerangPlanner>();
  rclcpp::spin(boomerang_planner);
  rclcpp::shutdown();
  return 0;
}
