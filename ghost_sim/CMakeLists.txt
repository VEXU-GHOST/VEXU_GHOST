cmake_minimum_required(VERSION 3.5)
project(ghost_sim)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_BUILD_TYPE "DEBUG")

if(CMAKE_BUILD_TYPE EQUAL "DEBUG")
  add_compile_options(-g -O0)
endif(CMAKE_BUILD_TYPE EQUAL "DEBUG")

add_compile_options(-Wno-dev)

########################
##### DEPENDENCIES #####
########################
# Base
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# ROS
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Gazebo
find_package(gazebo REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_plugins REQUIRED)

# Custom
find_package(ghost_msgs REQUIRED)
find_package(ghost_control REQUIRED)

# Include
include_directories(include)
link_directories(${gazebo_dev_LIBRARY_DIRS})

###################
##### TARGETS #####
###################
# Gazebo Motor Plugin
add_library(gazebo_motor_plugin SHARED src/gazebo_motor_plugin.cpp)
ament_target_dependencies(gazebo_motor_plugin
  rclcpp
  std_msgs
  gazebo
  gazebo_dev
  gazebo_msgs
  gazebo_ros
  gazebo_plugins
  ghost_control
)
target_link_libraries(gazebo_motor_plugin
  ghost_control::dc_motor_model)
target_include_directories(gazebo_motor_plugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_export_libraries(gazebo_motor_plugin)

# Gazebo Joint PID Plugin
add_library(gazebo_joint_pid_plugin SHARED src/gazebo_joint_pid_plugin.cpp)
ament_target_dependencies(gazebo_joint_pid_plugin
  rclcpp
  std_msgs
  geometry_msgs
  gazebo
  gazebo_dev
  gazebo_msgs
  gazebo_ros
  gazebo_plugins
  ghost_control
)
target_link_libraries(gazebo_joint_pid_plugin
  ghost_control::dc_motor_model
)
target_include_directories(gazebo_joint_pid_plugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_export_libraries(gazebo_joint_pid_plugin)

# Gazebo Swerve Odometry Plugin
add_library(swerve_odometry_plugin SHARED src/swerve_odometry_plugin.cpp)
ament_target_dependencies(swerve_odometry_plugin
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  gazebo
  gazebo_dev
  gazebo_msgs
  gazebo_ros
  gazebo_plugins
  ghost_control
)
target_include_directories(swerve_odometry_plugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_export_libraries(swerve_odometry_plugin)

##V5 Simulator Node
add_library(v5_sim_node SHARED src/v5_sim_node.cpp)
ament_target_dependencies(v5_sim_node
  rclcpp
  sensor_msgs
  ghost_msgs
)
target_link_libraries(v5_sim_node
  yaml-cpp
)
target_include_directories(v5_sim_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_export_targets(v5_sim_node HAS_LIBRARY_TARGET)
install(
  TARGETS v5_sim_node
  EXPORT v5_sim_node
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

add_executable(ground_truth_pose_publisher
  src/ground_truth_pose_publisher.cpp
)
ament_target_dependencies(ground_truth_pose_publisher
  rclcpp
  tf2_msgs
  geometry_msgs
  gazebo_msgs
  ghost_msgs
)
install(TARGETS
  ground_truth_pose_publisher
  DESTINATION lib/${PROJECT_NAME})

add_executable(v5_sim_node_main
  src/v5_sim_node_main.cpp
)
target_link_libraries(v5_sim_node_main
  v5_sim_node
)
install(TARGETS
v5_sim_node_main
  DESTINATION lib/${PROJECT_NAME})

add_executable(test_v5_sim_node
  test/test_v5_sim_node.cpp
)
target_link_libraries(test_v5_sim_node
  yaml-cpp
  v5_sim_node
)
install(TARGETS
  test_v5_sim_node
  DESTINATION lib/${PROJECT_NAME})

# ################
# ### Install ####
# ################
install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  urdf
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()
