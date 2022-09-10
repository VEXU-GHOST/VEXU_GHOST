cmake_minimum_required(VERSION 3.5)
project(swerve_drive)

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

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-g -O0)

######################
#### Dependencies ####
######################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include)
link_directories(${gazebo_dev_LIBRARY_DIRS})

############################
#### Simulation Targets ####
############################
add_library(dc_motor_model SHARED src/simulator_plugins/dc_motor_model.cpp)
target_include_directories(dc_motor_model
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_export_libraries(dc_motor_model)

add_library(gazebo_motor_plugin SHARED src/simulator_plugins/gazebo_motor_plugin.cpp)
ament_target_dependencies(gazebo_motor_plugin
  rclcpp
  std_msgs
  gazebo
  gazebo_dev
  gazebo_msgs
  gazebo_ros
  gazebo_plugins
)
target_link_libraries(gazebo_motor_plugin
  dc_motor_model)
target_include_directories(gazebo_motor_plugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_export_libraries(gazebo_motor_plugin)

add_library(gazebo_joint_pid_plugin SHARED src/simulator_plugins/gazebo_joint_pid_plugin.cpp)
ament_target_dependencies(gazebo_joint_pid_plugin
  rclcpp
  std_msgs
  geometry_msgs
  gazebo
  gazebo_dev
  gazebo_msgs
  gazebo_ros
  gazebo_plugins
)
target_link_libraries(gazebo_joint_pid_plugin
  dc_motor_model)
target_include_directories(gazebo_joint_pid_plugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_export_libraries(gazebo_joint_pid_plugin)

#########################
#### Latency Testing ####
#########################
add_executable(latency_test_publisher src/latency_tests/latency_test_publisher.cpp)
ament_target_dependencies(latency_test_publisher rclcpp sensor_msgs)

install(TARGETS
  latency_test_publisher
  DESTINATION lib/${PROJECT_NAME})

add_executable(latency_test_subscriber src/latency_tests/latency_test_subscriber.cpp)
ament_target_dependencies(latency_test_subscriber rclcpp sensor_msgs)

install(TARGETS
  latency_test_subscriber
  DESTINATION lib/${PROJECT_NAME})

#################
#### Testing ####
#################
ament_add_gtest(test_dc_motor_model test/test_dc_motor_model.cpp)
target_link_libraries(test_dc_motor_model
dc_motor_model
gtest)

# ament_add_gtest(test_gazebo_swerve_plugin test/test_gazebo_swerve_plugin.cpp)
# ament_target_dependencies(test_gazebo_swerve_plugin
#   ament_index_cpp
# )
# target_link_libraries(test_gazebo_swerve_plugin
#   gazebo_swerve_plugin
#   gazebo_test_fixture
#   gtest)

#################
#### Install ####
#################
install(
  TARGETS 
    dc_motor_model
    gazebo_motor_plugin
    gazebo_joint_pid_plugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY
  launch
  urdf
  rviz
  scripts
  DESTINATION share/${PROJECT_NAME})

ament_package()
