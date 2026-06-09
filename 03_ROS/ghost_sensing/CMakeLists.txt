cmake_minimum_required(VERSION 3.8)
project(ghost_sensing)

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
  rclcpp
  visualization_msgs
  ghost_util
  ghost_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  yaml-cpp
  Eigen3
  builtin_interfaces
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

#####################
#### Executables ####
#####################
# IMU Filtering Node
add_executable(imu_filter_node
  src/imu_filter_node.cpp
)
ament_target_dependencies(imu_filter_node
  ${DEPENDENCIES}
)
target_include_directories(imu_filter_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
  imu_filter_node
  DESTINATION lib/${PROJECT_NAME})

# Ball Colour Classifier — consumes ghost_msgs/ColorSensorState from the sensor
# host and republishes a ghost_msgs/BallColor on <input_topic>/class.
add_executable(ball_color_classifier
  src/ball_color_classifier.cpp
)
ament_target_dependencies(ball_color_classifier
  ${DEPENDENCIES}
)
target_include_directories(ball_color_classifier
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
  ball_color_classifier
  DESTINATION lib/${PROJECT_NAME})

#################
#### Install ####
#################
install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
