cmake_minimum_required(VERSION 3.5)
project(swerve_drive)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-g -O0)

# find dependencies
find_package(ament_cmake REQUIRED)

include_directories(include)

#### Targets ####
add_library(dc_motor_model SHARED src/dc_motor_model.cpp)
target_include_directories(dc_motor_model
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_export_targets(export__dc_motor_model HAS_LIBRARY_TARGET)
install(
  TARGETS dc_motor_model
  EXPORT export__dc_motor_model
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#### Testing ####
find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(test_dc_motor_model test/test_dc_motor_model.cpp)
target_link_libraries(test_dc_motor_model
dc_motor_model
gtest)
target_include_directories(dc_motor_model
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

#### Install Share ####
install(DIRECTORY launch urdf rviz scripts
  DESTINATION share/${PROJECT_NAME})


ament_package()
