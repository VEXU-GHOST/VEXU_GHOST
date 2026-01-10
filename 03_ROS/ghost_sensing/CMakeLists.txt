cmake_minimum_required(VERSION 3.8)
project(ghost_sensing)

# Add Global CMake Config for Ghost Packages
set(Ghost_DIR "$ENV{VEXU_HOME}/cmake")
find_package(Ghost CONFIG REQUIRED)

######################
#### Dependencies ####
######################
# Add all package dependencies to one list
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

# Iterates through the list and finds the packages
foreach(pkg ${DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()

ament_export_dependencies(${DEPENDENCIES})

######################
#### Include dirs ####
######################
set(INCLUDE
  include
)

include_directories(${INCLUDE})
ament_export_include_directories(${INCLUDE})

####################
#### Library(s) ####
####################
add_library(ghost_sensing SHARED
  src/I2C_interfacing.cpp
  src/TCA9536.cpp
)
target_include_directories(ghost_sensing
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ghost_sensing
  ${DEPENDENCIES}
)
ament_export_libraries(ghost_sensing)
ament_export_targets(ghost_sensing HAS_LIBRARY_TARGET)
install(
  TARGETS ghost_sensing
  EXPORT ghost_sensing
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# --- Node executable(s) ---
add_executable(gpio_expander src/gpio_expander_node.cpp)
ament_target_dependencies(gpio_expander rclcpp)

# Link to ghost_sensing library target
target_link_libraries(gpio_expander ghost_sensing)

install(TARGETS gpio_expander DESTINATION lib/${PROJECT_NAME})

#################
##### Install ####
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

#################
##### Tests #####
#################
# Python-based runtime test exists under test/color_sensor_test.py; enable Python tests if desired.

ament_package()
