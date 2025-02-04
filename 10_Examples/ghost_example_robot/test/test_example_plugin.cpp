#include <pluginlib/class_loader.hpp>
#include <ghost_ros_interfaces/competition/v5_robot_base.hpp>

#include <iostream>

#include "gtest/gtest.h"

using ghost_ros_interfaces::V5RobotBase;

TEST(TestExamplePlugin, testLoadPlugin) {
  // Pass name of plugin which is derived from V5RobotBase (e.g. my_robot_pkg::MyRobotPlugin)
  std::string plugin_name = "ghost_example_robot::GhostExampleRobot";

  pluginlib::ClassLoader<V5RobotBase> robot_class_loader("ghost_ros_interfaces",
    "ghost_ros_interfaces::V5RobotBase");
  std::shared_ptr<V5RobotBase> v5_robot_base_ptr;

  // Test we can construct plugin
  EXPECT_NO_THROW(v5_robot_base_ptr = robot_class_loader.createSharedInstance(plugin_name));

  // Test we can configure plugin
  EXPECT_NO_THROW(v5_robot_base_ptr->configure());

  // Test competition states
  EXPECT_NO_THROW(v5_robot_base_ptr->initialize());
  EXPECT_NO_THROW(v5_robot_base_ptr->disabled());
  EXPECT_NO_THROW(v5_robot_base_ptr->autonomous(0.0));
  EXPECT_NO_THROW(v5_robot_base_ptr->teleop(0.0));
}

int main(int argc, char ** argv)
{
  // This is some hackery to add ros parameters necessary for loading the plugin
  // Normally these are passed by something like 'ros2 run my_node --ros-args -p param_name:=param_val'
  std::string ros_args = "--ros-args";
  std::string dash_p = "-p";

  std::string param = "robot_config_yaml_path:=" + std::string(getenv("VEXU_HOME")) +
    "/10_Examples/ghost_example_robot/config/example_hardware_config.yaml";

  constexpr int argc_appended = 4;
  char * argv_appended[argc_appended];
  argv_appended[0] = argv[0];
  argv_appended[1] = &ros_args[0];
  argv_appended[2] = &dash_p[0];
  argv_appended[3] = &param[0];

  rclcpp::init(argc_appended, argv_appended);

  // Standard GTest main
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
