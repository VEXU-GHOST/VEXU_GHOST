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

# Set Package Dependencies
set(DEPENDENCIES
  ament_cmake
  ament_cmake_gtest
  eigen3_cmake_module
  Eigen3
  matplotlib_cpp
  yaml-cpp
  PythonLibs
  casadi
  ghost_estimation
  ghost_common
  )

foreach(pkg ${DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()
ament_export_dependencies(${DEPENDENCIES})

# Set Include Directories
set(INCLUDE
include
${PYTHON_INCLUDE_DIRS}
)

include_directories(${INCLUDE})
ament_export_include_directories(${INCLUDE})

# set(CMAKE_BUILD_TYPE "DEBUG")
set(CMAKE_BUILD_TYPE "RELEASE")

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

# Ghost Motor Library
add_library(motor_controller SHARED src/motor_controller.cpp)
ament_target_dependencies(motor_controller
${DEPENDENCIES}
)
target_include_directories(motor_controller
PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
ament_export_targets(motor_controller HAS_LIBRARY_TARGET)
install(
  TARGETS motor_controller
  EXPORT motor_controller
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
  ${DEPENDENCIES}
)
install(
  TARGETS mechanical_differential
  EXPORT mechanical_differential
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Casadi Collocation Model
add_library(casadi_collocation_model SHARED src/models/casadi_collocation_model.cpp)
target_link_libraries(casadi_collocation_model
  yaml-cpp
  )
target_include_directories(casadi_collocation_model
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_targets(casadi_collocation_model HAS_LIBRARY_TARGET)

install(
  TARGETS casadi_collocation_model
  EXPORT casadi_collocation_model
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
  casadi_collocation_model
  casadi
  yaml-cpp
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
add_executable(swerve_model_test_main src/swerve_model_test_main.cpp)
ament_target_dependencies(swerve_model_test_main
  ${DEPENDENCIES}
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
add_executable(casadi_swerve_model_generation src/casadi_swerve_model_generation.cpp)
ament_target_dependencies(casadi_swerve_model_generation
  ${DEPENDENCIES}
)
target_link_libraries(casadi_swerve_model_generation
  casadi_collocation_model
  casadi
)
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
