cmake_minimum_required(VERSION 3.5)
project(ghost_estimation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set(CMAKE_BUILD_TYPE "DEBUG")
set(CMAKE_BUILD_TYPE "RELEASE")

if(CMAKE_BUILD_TYPE EQUAL "DEBUG")
  add_compile_options(-g -O0 -DGHOST_DEBUG_VERBOSE)
endif(CMAKE_BUILD_TYPE EQUAL "DEBUG")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

######################
### AMRL Share Lib ###
######################
include_directories(src/shared)
add_subdirectory(src/shared)
ament_export_targets(amrl_shared_lib HAS_LIBRARY_TARGET)
install(
  TARGETS amrl_shared_lib
  EXPORT amrl_shared_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

###############
### Filters ###
###############
add_library(first_order_low_pass_filter SHARED
src/filters/first_order_low_pass_filter.cpp
)
target_include_directories(first_order_low_pass_filter
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_targets(first_order_low_pass_filter HAS_LIBRARY_TARGET)
install(
  TARGETS first_order_low_pass_filter
  EXPORT first_order_low_pass_filter
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

add_library(second_order_low_pass_filter SHARED
src/filters/second_order_low_pass_filter.cpp
)
target_include_directories(second_order_low_pass_filter
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_targets(second_order_low_pass_filter HAS_LIBRARY_TARGET)
install(
  TARGETS second_order_low_pass_filter
  EXPORT second_order_low_pass_filter
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#######################
### Particle Filter ###
#######################
# Vector Map Library
add_library(vector_map SHARED src/vector_map/vector_map.cpp)
target_link_libraries(vector_map
  amrl_shared_lib
  gflags
)
ament_target_dependencies(vector_map
  Eigen3
)
target_include_directories(vector_map
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  ament_export_targets(vector_map HAS_LIBRARY_TARGET)
install(
  TARGETS vector_map
  EXPORT vector_map
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Particle Filter Library
add_library(particle_filter SHARED src/particle_filter/particle_filter.cpp)
ament_target_dependencies(particle_filter
  rclcpp
  tf2_msgs
  sensor_msgs
  visualization_msgs
  Eigen3
)
target_link_libraries(particle_filter
  amrl_shared_lib
  vector_map
  yaml-cpp
)
target_include_directories(particle_filter
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_targets(particle_filter HAS_LIBRARY_TARGET)
install(
  TARGETS particle_filter
  EXPORT particle_filter
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#######################
#### Disc Detector ####
#######################
# Install Python modules
ament_python_install_package(${PROJECT_NAME})
 
# Install Python executables
install(PROGRAMS
  src/disc_detector/disc_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

#################
#### Install ####
#################
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY 
  src/shared/math
  src/shared/util
  DESTINATION include
  FILES_MATCHING PATTERN "*.h*"
)

ament_package()
