cmake_minimum_required(VERSION 3.8)
project(ghost_autonomy)

set(Ghost_DIR "$ENV{VEXU_HOME}/cmake")
find_package(Ghost CONFIG REQUIRED)

# find dependencies
set(DEPENDENCIES
  ament_cmake
  rclcpp
  behaviortree_cpp
  behaviortree_ros2
  pluginlib
  std_msgs
  ghost_msgs
  ghost_v5_interfaces
  tf2
  ghost_util
)

foreach(pkg ${DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()
ament_export_dependencies(${DEPENDENCIES})

# Set Include Directories
set(INCLUDE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

include_directories(${INCLUDE})
ament_export_include_directories(${INCLUDE})

# test robot config publisher
add_executable(publish_test
  src/publish_test.cpp
)
ament_target_dependencies(publish_test
  ${DEPENDENCIES}
)
target_include_directories(publish_test
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
publish_test
  DESTINATION lib/${PROJECT_NAME})

#################
#### Install ####
#################
install(DIRECTORY include/
  DESTINATION include
)



ament_package()
