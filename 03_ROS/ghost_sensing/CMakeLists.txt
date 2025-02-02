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
  std_msgs
  sensor_msgs
  geometry_msgs
  ghost_msgs
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

# TCS Color Sensor
add_executable(tcs_color_sensor
  src/driver_tcs34725_interface.cpp
  src/driver_tcs34725.cpp
  src/tcs_color_sensor.cpp
)
ament_target_dependencies(tcs_color_sensor
  ${DEPENDENCIES}
)
target_link_libraries(tcs_color_sensor i2c)
target_include_directories(tcs_color_sensor
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
  tcs_color_sensor
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
