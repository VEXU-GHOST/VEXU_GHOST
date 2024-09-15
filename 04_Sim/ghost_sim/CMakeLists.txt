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
set(DEPENDENCIES
  ament_cmake
  ament_cmake_gtest
  Eigen3
  yaml-cpp
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_msgs
  nav_msgs
  visualization_msgs
  gazebo_dev
  gazebo_msgs
  gazebo_ros
  gazebo_plugins
  ghost_msgs
  ghost_control
)

foreach(pkg ${DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()
ament_export_dependencies(${DEPENDENCIES})

# Set Include Directories
set(INCLUDE
include
${PYTHON_INCLUDE_DIRS}
)

###############################
####   Tank MPC RVIZ Node  ####
###############################
add_executable(tank_mpc_viz
  src/tank_mpc_viz.cpp
)
ament_target_dependencies(tank_mpc_viz
  ${DEPENDENCIES}
)
install(TARGETS
tank_mpc_viz
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(tank_mpc_viz
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
  tank_mpc_viz
  DESTINATION lib/${PROJECT_NAME})


##############################
####   Bag Sim Time Node  ####
##############################
add_executable(bag_set_pose_time
  src/bag_set_pose_time.cpp
)
ament_target_dependencies(bag_set_pose_time
  rclcpp
  geometry_msgs
)
install(TARGETS
bag_set_pose_time
  DESTINATION lib/${PROJECT_NAME})

include_directories(${INCLUDE})
ament_export_include_directories(${INCLUDE})

include_directories(${GTEST_INCLUDE_DIRS})

link_directories(${gazebo_dev_LIBRARY_DIRS})

###################
##### TARGETS #####
###################
# Gazebo Motor Plugin

# add_library(gazebo_motor_plugin SHARED src/gazebo_motor_plugin.cpp)
# ament_target_dependencies(gazebo_motor_plugin
#   ${DEPENDENCIES}
# )
# target_link_libraries(gazebo_motor_plugin
#   ghost_control::dc_motor_model)
# target_include_directories(gazebo_motor_plugin
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>)
# ament_export_libraries(gazebo_motor_plugin)


# Gazebo Joint PID Plugin
# add_library(gazebo_joint_pid_plugin SHARED src/gazebo_joint_pid_plugin.cpp)
# ament_target_dependencies(gazebo_joint_pid_plugin
#   rclcpp
#   std_msgs
#   geometry_msgs
#   gazebo_dev
#   gazebo_msgs
#   gazebo_ros
#   gazebo_plugins
#   ghost_msgs
#   ghost_control
# )
# target_link_libraries(gazebo_joint_pid_plugin
#   ghost_control::dc_motor_model
# )
# target_include_directories(gazebo_joint_pid_plugin
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>)
# ament_export_libraries(gazebo_joint_pid_plugin)
enable_testing()
ament_add_gtest(test_diff_drive test/test_diff_drive.cpp
    # Long timeout because has to run gazebo several times
    TIMEOUT
    120)

target_link_libraries(test_diff_drive
  gazebo_test_fixture
)

ament_target_dependencies(test_diff_drive
  "builtin_interfaces"
  "gazebo_dev"
  "gazebo_msgs"
  "geometry_msgs"
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "nav_msgs"
)

# Gazebo V5 Robot Plugin
# add_library(v5_robot_plugin SHARED src/v5_robot_plugin.cpp)
# ament_target_dependencies(v5_robot_plugin
#   ${DEPENDENCIES}
# )
# target_include_directories(v5_robot_plugin
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>)
# ament_export_libraries(v5_robot_plugin)



install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  urdf
  worlds
  rviz
  config
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)


ament_package()
