cmake_minimum_required(VERSION 3.8)
project(ghost_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(ghost_estimation REQUIRED)
find_package(ghost_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ghost_msgs REQUIRED)

include_directories(include)

##############################
#### Ghost EKF-PF Node ####
##############################
add_executable(ekf_pf_node
  src/ekf_pf_node.cpp
)
ament_target_dependencies(ekf_pf_node
  rclcpp
  tf2_msgs
  sensor_msgs
  visualization_msgs
  nav_msgs
  ghost_msgs
  ghost_estimation
  ghost_util
  Eigen3
)
target_link_libraries(ekf_pf_node
  ghost_estimation::amrl_shared_lib
  ghost_estimation::vector_map
  ghost_estimation::particle_filter
  yaml-cpp
)
target_include_directories(ekf_pf_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
ekf_pf_node
  DESTINATION lib/${PROJECT_NAME})

##############################
#### Ghost PF-EKF Node ####
##############################
# add_executable(pf_ekf_node
#   src/pf_ekf_node.cpp
# )
# ament_target_dependencies(pf_ekf_node
#   rclcpp
#   tf2_msgs
#   sensor_msgs
#   visualization_msgs
#   nav_msgs
#   ghost_msgs
#   ghost_estimation
#   ghost_util
#   Eigen3
# )
# target_link_libraries(pf_ekf_node
#   ghost_estimation::amrl_shared_lib
#   ghost_estimation::vector_map
#   ghost_estimation::particle_filter
#   yaml-cpp
# )
# target_include_directories(pf_ekf_node
#   PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# install(TARGETS
# pf_ekf_node
#   DESTINATION lib/${PROJECT_NAME})

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
