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
find_package(ghost_common REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(yaml-cpp REQUIRED)

find_package(ghost_msgs REQUIRED)
find_package(ghost_serial REQUIRED)
find_package(ghost_estimation REQUIRED)
find_package(ghost_util REQUIRED)

include_directories(include)

#####################################
##### Foxglove Diagnostics Node #####
#####################################
add_executable(foxglove_diagnostics_node
  src/ros_nodes/foxglove_diagnostics_node.cpp
)
ament_target_dependencies(foxglove_diagnostics_node
  rclcpp
  sensor_msgs
  ghost_msgs
)
target_include_directories(foxglove_diagnostics_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
  foxglove_diagnostics_node
  DESTINATION lib/${PROJECT_NAME})

##############################
#### Ghost Estimator Node ####
##############################
# add_executable(ghost_estimator_node
#   src/ros_nodes/ghost_estimator_node.cpp
# )
# ament_target_dependencies(ghost_estimator_node
#   rclcpp
#   tf2_msgs
#   sensor_msgs
#   visualization_msgs
#   ghost_common
#   ghost_msgs
#   ghost_util
#   ghost_estimation
#   Eigen3
# )
# target_link_libraries(ghost_estimator_node
#   ghost_estimation::amrl_shared_lib
#   ghost_estimation::vector_map
#   ghost_estimation::particle_filter
#   yaml-cpp
# )
# target_include_directories(ghost_estimator_node
#   PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# install(TARGETS
#   ghost_estimator_node
#   DESTINATION lib/${PROJECT_NAME})

##################################
#### Robot State Machine Node ####
##################################
# add_executable(robot_state_machine_node
#   src/ros_nodes/robot_state_machine_node.cpp
# )
# ament_target_dependencies(robot_state_machine_node
#   rclcpp
#   Eigen3
#   std_msgs
#   ghost_msgs
#   visualization_msgs
#   ghost_estimation
#   ghost_util
# )
# target_link_libraries(robot_state_machine_node
#   ghost_estimation::amrl_shared_lib
#   ghost_util::angle_util
#   yaml-cpp
#   v5_serial_msg_config
# )
# target_include_directories(robot_state_machine_node
#   PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# install(TARGETS
# robot_state_machine_node
# DESTINATION lib/${PROJECT_NAME})

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
