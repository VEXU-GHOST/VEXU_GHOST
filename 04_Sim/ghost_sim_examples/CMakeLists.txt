cmake_minimum_required(VERSION 3.8)
project(ghost_sim_examples)


set(Ghost_DIR "$ENV{VEXU_HOME}/cmake")
add_compile_options(-Wall -Wextra -Wpedantic)


find_package(Ghost CONFIG REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(ghost_msgs REQUIRED)
find_package(ghost_control REQUIRED)
find_package(gazebo REQUIRED)
find_package(ghost_control REQUIRED)
find_package(ghost_util REQUIRED)
find_package(ghost_v5_interfaces REQUIRED)
find_package(ghost_estimation REQUIRED)

set(DEPENDENCIES
ament_cmake
rclcpp
std_msgs
gazebo_ros
ghost_msgs
ghost_control
ghost_util
gazebo
ghost_control
ghost_v5_interfaces
)

# foreach(pkg ${DEPENDENCIES})
#   find_package(${pkg} REQUIRED)
# endforeach()
ament_export_dependencies(${DEPENDENCIES})


include_directories(
  include  
  ${GAZEBO_INCLUDE_DIRS}
)


link_directories(${GAZEBO_LIBRARY_DIRS})


add_definitions(${GAZEBO_CXX_FLAGS})


add_library(gazebo_torque_plugin SHARED src/gazebo_torque_plugin.cpp)
ament_target_dependencies(gazebo_torque_plugin ${DEPENDENCIES})
target_link_libraries(gazebo_torque_plugin ${GAZEBO_LIBRARIES})


install(DIRECTORY
  launch
  urdf
  rviz
  world
  test
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
