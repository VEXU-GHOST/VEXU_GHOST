cmake_minimum_required(VERSION 3.5)
project(ghost_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set(CMAKE_BUILD_TYPE "DEBUG")
set(CMAKE_BUILD_TYPE "RELEASE")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(Eigen3 REQUIRED)

#################
### Libraries ###
#################
# DC Motor Model Library
add_library(dc_motor_model SHARED src/models/dc_motor_model.cpp)
target_include_directories(dc_motor_model
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_targets(dc_motor_model HAS_LIBRARY_TARGET)

install(
  TARGETS dc_motor_model
  EXPORT dc_motor_model
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Mechanical Differential Library
add_library(mechanical_differential SHARED src/models/mechanical_differential.cpp)
target_include_directories(mechanical_differential
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_targets(mechanical_differential HAS_LIBRARY_TARGET)
ament_target_dependencies(mechanical_differential
  Eigen3
)
install(
  TARGETS mechanical_differential
  EXPORT mechanical_differential
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

###############
### Testing ###
###############
ament_add_gtest(test_dc_motor_model test/test_dc_motor_model.cpp)
target_link_libraries(test_dc_motor_model
  dc_motor_model
  gtest
)

ament_add_gtest(test_mechanical_differential test/test_mechanical_differential.cpp)
target_link_libraries(test_mechanical_differential
  mechanical_differential
  gtest
)

###############
### Install ###
###############
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
