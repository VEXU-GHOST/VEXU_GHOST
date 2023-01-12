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
# set(CMAKE_BUILD_TYPE "DEBUG")
set(CMAKE_BUILD_TYPE "RELEASE")

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

find_package(yaml-cpp REQUIRED)

find_package(ghost_msgs REQUIRED)
find_package(ghost_serial REQUIRED)
find_package(ghost_estimation REQUIRED)

find_library(v5_serial_node REQUIRED)

include_directories(include)

###############################
##### Robot Configuration #####
###############################
add_library(v5_serial_msg_config SHARED
  src/robot_config/v5_serial_msg_config.cpp
)
target_include_directories(v5_serial_msg_config
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_targets(v5_serial_msg_config HAS_LIBRARY_TARGET)
install(
  TARGETS v5_serial_msg_config
  EXPORT v5_serial_msg_config
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
  )

#####################
##### ROS Nodes #####
#####################
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

############################
#### Jetson Serial Node ####
############################
add_executable(jetson_v5_serial_node
  src/ros_nodes/jetson_v5_serial_node.cpp
)
ament_target_dependencies(jetson_v5_serial_node
  rclcpp
  sensor_msgs
  ghost_msgs
  ghost_serial
)
target_link_libraries(jetson_v5_serial_node
  ghost_serial::jetson_serial_base
  ghost_serial::cobs
  v5_serial_msg_config
  yaml-cpp
)
target_include_directories(jetson_v5_serial_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
  jetson_v5_serial_node
  DESTINATION lib/${PROJECT_NAME})

##############################
#### Ghost Estimator Node ####
##############################
add_executable(ghost_estimator_node
  src/ros_nodes/ghost_estimator_node.cpp
)
ament_target_dependencies(ghost_estimator_node
  rclcpp
  tf2_msgs
  sensor_msgs
  visualization_msgs
  ghost_msgs
  ghost_estimation
  Eigen3
)
target_link_libraries(ghost_estimator_node
  ghost_estimation::amrl_shared_lib
  ghost_estimation::vector_map
  ghost_estimation::particle_filter
  yaml-cpp
)
target_include_directories(ghost_estimator_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
  ghost_estimator_node
  DESTINATION lib/${PROJECT_NAME})

##################################
#### Robot State Machine Node ####
##################################
add_executable(robot_state_machine_node
  src/ros_nodes/robot_state_machine_node.cpp
)
ament_target_dependencies(robot_state_machine_node
  rclcpp
  ghost_msgs
  Eigen3
)
target_link_libraries(robot_state_machine_node
  yaml-cpp
  v5_serial_msg_config
)
target_include_directories(robot_state_machine_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
robot_state_machine_node
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

ament_package()
