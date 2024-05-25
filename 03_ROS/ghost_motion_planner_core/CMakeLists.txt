cmake_minimum_required(VERSION 3.8)
project(ghost_motion_planner_core)

# Add Global CMake Config for Ghost Packages
set(Ghost_DIR "$ENV{VEXU_HOME}/cmake")
find_package(Ghost CONFIG REQUIRED)

######################
#### Dependencies ####
######################
# Set Package Dependencies
set(DEPENDENCIES
  ament_cmake
  ament_cmake_gtest
  ghost_v5_interfaces
  ghost_msgs
  ghost_ros_interfaces
  # ghost_serial
  ghost_util
  nav_msgs
  # pluginlib
  rclcpp
  # yaml-cpp
)

foreach(pkg ${DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()
ament_export_dependencies(${DEPENDENCIES})

# Set Include Directories
set(INCLUDE
  include
)

include_directories(${INCLUDE})
ament_export_include_directories(${INCLUDE})

############################
#### Motion Planner Library ####
############################
add_library(motion_planner SHARED
src/motion_planner.cpp
)
target_include_directories(motion_planner
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(motion_planner
  ${DEPENDENCIES}
)
ament_export_targets(motion_planner HAS_LIBRARY_TARGET)
install(
  TARGETS motion_planner
  EXPORT motion_planner
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#####################
#### Executables ####
#####################
# Jetson Serial Node
# add_executable(jetson_v5_serial_node
#   src/serial/jetson_v5_serial_node.cpp
# )
# ament_target_dependencies(jetson_v5_serial_node
#   ${DEPENDENCIES}
# )
# target_link_libraries(jetson_v5_serial_node
#   ghost_serial::jetson_serial_base
#   ghost_serial::cobs
#   msg_helpers
#   yaml-cpp
# )
# target_include_directories(jetson_v5_serial_node
#   PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# install(TARGETS
#   jetson_v5_serial_node
#   DESTINATION lib/${PROJECT_NAME})

#################
#### Install ####
#################
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
