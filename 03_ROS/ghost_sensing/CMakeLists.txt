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
