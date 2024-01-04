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
  ghost_common
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

include_directories(${INCLUDE})
ament_export_include_directories(${INCLUDE})

link_directories(${gazebo_dev_LIBRARY_DIRS})


###################
##### TARGETS #####
###################
# Gazebo Motor Plugin
add_library(gazebo_motor_plugin SHARED src/gazebo_motor_plugin.cpp)
ament_target_dependencies(gazebo_motor_plugin
  ${DEPENDENCIES}
)
target_link_libraries(gazebo_motor_plugin
  ghost_control::dc_motor_model)
target_include_directories(gazebo_motor_plugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_export_libraries(gazebo_motor_plugin)



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

# Gazebo V5 Robot Plugin
add_library(v5_robot_plugin SHARED src/v5_robot_plugin.cpp)
ament_target_dependencies(v5_robot_plugin
  ${DEPENDENCIES}
)
target_include_directories(v5_robot_plugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_export_libraries(v5_robot_plugin)

add_executable(ground_truth_pose_publisher
  src/ground_truth_pose_publisher.cpp
)
ament_target_dependencies(ground_truth_pose_publisher
  ${DEPENDENCIES}
)
install(TARGETS
  ground_truth_pose_publisher
  DESTINATION lib/${PROJECT_NAME})

# Node to test new V5ActuactorCommand communication
add_executable(test_publisher_v5_actuator_cmd
  test/test_publisher_v5_actuator_cmd.cpp
)
ament_target_dependencies(test_publisher_v5_actuator_cmd
  ${DEPENDENCIES}
)
install(TARGETS
  test_publisher_v5_actuator_cmd
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
