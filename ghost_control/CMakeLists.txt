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

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(matplotlib_cpp REQUIRED)
find_package(PythonLibs REQUIRED)

find_library(casadi REQUIRED)

include_directories(${PYTHON_INCLUDE_DIRS})

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

######################
### Casadi Example ###
######################
add_executable(casadi_example_model_generation src/casadi_examples/casadi_example_model_generation.cpp)
ament_target_dependencies(casadi_example_model_generation
  Eigen3
)
target_link_libraries(casadi_example_model_generation
  casadi
  matplotlib_cpp::matplotlib_cpp
  ${PYTHON_LIBRARIES}
  )
target_include_directories(casadi_example_model_generation
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
  casadi_example_model_generation
  DESTINATION lib/${PROJECT_NAME})

###################
### Executables ###
###################
# Main to test Diff Swerve State Space Modeling for MPC
add_executable(swerve_model_test_main src/models/swerve_model_test_main.cpp)
ament_target_dependencies(swerve_model_test_main
  Eigen3
)
target_link_libraries(swerve_model_test_main
casadi)
target_include_directories(swerve_model_test_main
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
  swerve_model_test_main
  DESTINATION lib/${PROJECT_NAME})

# Main to generate Problem Formulation for MDS MPC
add_executable(casadi_swerve_model_generation src/models/casadi_swerve_model_generation.cpp)
ament_target_dependencies(casadi_swerve_model_generation
  Eigen3
)
target_link_libraries(casadi_swerve_model_generation
casadi)
target_include_directories(casadi_swerve_model_generation
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
  casadi_swerve_model_generation
  DESTINATION lib/${PROJECT_NAME})

###############
### Testing ###
###############
ament_add_gtest(test_dc_motor_model test/test_dc_motor_model.cpp)
target_link_libraries(test_dc_motor_model
  dc_motor_model
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
