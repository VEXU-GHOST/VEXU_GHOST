cmake_minimum_required(VERSION 3.5)
project(ghost_ros)

#########################
#### Compile Options ####
#########################
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compile time flags to add debug symbols and disable code optimization (for using debugger)
set(CMAKE_BUILD_TYPE "DEBUG")

if(CMAKE_BUILD_TYPE EQUAL "DEBUG")
  add_compile_options(-g -O0 -DGHOST_DEBUG_VERBOSE)
endif(CMAKE_BUILD_TYPE EQUAL "DEBUG")

######################
#### Dependencies ####
######################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(ghost_serial REQUIRED)
find_package(yaml-cpp REQUIRED)

find_library(v5_serial_node REQUIRED)

include_directories(include)
include_directories(src/shared)
add_subdirectory(src/shared)

###################
##### Library #####
###################
# Vector Map Library
add_library(vector_map SHARED src/vector_map/vector_map.cpp)
target_link_libraries(vector_map
  amrl_shared_lib
  gflags
)
ament_target_dependencies(vector_map
  Eigen3
)
target_include_directories(vector_map
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(
  TARGETS vector_map
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Particle Filter Library
add_library(particle_filter_ros SHARED src/particle_filter/particle_filter_node.cpp src/particle_filter/particle_filter.cpp)
ament_target_dependencies(particle_filter_ros
  rclcpp
  tf2_msgs
  sensor_msgs
  visualization_msgs
  Eigen3
)
target_link_libraries(particle_filter_ros
  amrl_shared_lib
  vector_map
  yaml-cpp
)
target_include_directories(particle_filter_ros
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(
  TARGETS particle_filter_ros
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Basic Swerve Controller
add_executable(basic_swerve_controller src/basic_swerve_controller.cpp)
ament_target_dependencies(basic_swerve_controller
  rclcpp
  std_msgs
  geometry_msgs
  Eigen3
)

install(TARGETS
  basic_swerve_controller
  DESTINATION lib/${PROJECT_NAME})

#####################
##### Examples ######
#####################
# ROS Publisher Example
add_executable(ros_publisher_example src/examples/ros_publisher_example.cpp)
ament_target_dependencies(ros_publisher_example
  rclcpp
  sensor_msgs
  geometry_msgs
)

install(TARGETS
  ros_publisher_example
  DESTINATION lib/${PROJECT_NAME})

# ROS Subscriber Example
add_executable(ros_subscriber_example src/examples/ros_subscriber_example.cpp)
ament_target_dependencies(ros_subscriber_example
  rclcpp
  sensor_msgs
  geometry_msgs
)

install(TARGETS
  ros_subscriber_example
  DESTINATION lib/${PROJECT_NAME})

# YAML Load Example
add_executable(yaml_load_example src/examples/yaml_load.cpp)
target_link_libraries(yaml_load_example
  yaml-cpp
)

install(TARGETS
  yaml_load_example
  DESTINATION lib/${PROJECT_NAME})

########################
#### GHOST ROS Main ####
########################
add_executable(ghost_ros_main
  src/main.cpp
)
ament_target_dependencies(ghost_ros_main
  rclcpp
  ghost_serial
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2_msgs
  nav_msgs
)
target_link_libraries(ghost_ros_main
  amrl_shared_lib
  yaml-cpp
  particle_filter_ros
  ghost_serial::v5_serial_node
)

install(TARGETS
  ghost_ros_main
  DESTINATION lib/${PROJECT_NAME})

#################
#### Testing ####
#################
ament_add_gtest(test_stamped_queue test/test_stamped_queue.cpp)
target_link_libraries(test_stamped_queue
  gtest
)

#################
#### Install ####
#################
install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(dc_motor_model)

ament_package()
