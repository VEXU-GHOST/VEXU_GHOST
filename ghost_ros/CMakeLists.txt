cmake_minimum_required(VERSION 3.5)
project(ghost_ros)

# ########################
# ### Compile Options ####
# ########################
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_BUILD_TYPE EQUAL "DEBUG")
  add_compile_options(-g -O2)
endif(CMAKE_BUILD_TYPE EQUAL "DEBUG")

# #####################
# ### Dependencies ####
# #####################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)
include_directories(src/shared)
add_subdirectory(src/shared)

# ##################
# #### Library #####
# ##################
# DC Motor Model Library
add_library(dc_motor_model SHARED src/dc_motor_model.cpp)
target_include_directories(dc_motor_model
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_targets(dc_motor_model HAS_LIBRARY_TARGET)

install(
  TARGETS dc_motor_model
  EXPORT dc_motor_model
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Particle Filter Library
add_library(particle_filter SHARED src/particle_filter/particle_filter.cpp)
target_link_libraries(particle_filter
  amrl_shared_lib
  yaml-cpp
)
ament_target_dependencies(particle_filter
  Eigen3
)
target_include_directories(particle_filter
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(
  TARGETS particle_filter
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
)

install(TARGETS
  ros_publisher_example
  DESTINATION lib/${PROJECT_NAME})

# ROS Subscriber Example
add_executable(ros_subscriber_example src/examples/ros_subscriber_example.cpp)
ament_target_dependencies(ros_subscriber_example
  rclcpp
  sensor_msgs
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

# ########################
# ### GHOST ROS Main ####
# ########################
add_executable(ghost_ros_main src/main.cpp)
ament_target_dependencies(ghost_ros_main
  rclcpp
  std_msgs
)

target_link_libraries(ghost_ros_main
  amrl_shared_lib
  yaml-cpp
)

install(TARGETS
  ghost_ros_main
  DESTINATION lib/${PROJECT_NAME})

# ################
# ### Testing ####
# ################
ament_add_gtest(test_dc_motor_model test/test_dc_motor_model.cpp)
target_link_libraries(test_dc_motor_model
  dc_motor_model
  gtest
)

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
  rviz
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(dc_motor_model)

ament_package()
