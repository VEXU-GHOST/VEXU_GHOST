cmake_minimum_required(VERSION 3.5)
project(ghost_planners)

# Add Global CMake Config for Ghost Packages
set(Ghost_DIR "$ENV{HOME}/VEXU_GHOST/cmake")
find_package(Ghost CONFIG REQUIRED)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++17
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 17)
# endif()

# Set Package Dependencies
set(DEPENDENCIES
  ament_cmake
  ament_cmake_gtest
  ament_index_cpp
  eigen3_cmake_module
  Eigen3
  urdf
  matplotlib_cpp
  yaml-cpp
  PythonLibs
  casadi
  ghost_estimation
  ghost_control
  # ghost_v5_interfaces
  )

foreach(pkg ${DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()
ament_export_dependencies(${DEPENDENCIES})

# Set Include Directories
set(INCLUDE
include
# ${PYTHON_INCLUDE_DIRS}
)

include_directories(${INCLUDE})
ament_export_include_directories(${INCLUDE})

# set(CMAKE_BUILD_TYPE "DEBUG")
# set(CMAKE_BUILD_TYPE "RELEASE")

#################
### Libraries ###
#################
# # Casadi Collocation Model
# add_library(casadi_collocation_model SHARED src/models/casadi_collocation_model.cpp)
# target_link_libraries(casadi_collocation_model
#   yaml-cpp
#   )
# target_include_directories(casadi_collocation_model
#   PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# ament_export_targets(casadi_collocation_model HAS_LIBRARY_TARGET)

# install(
#   TARGETS casadi_collocation_model
#   EXPORT casadi_collocation_model
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

# robot trajectory class
add_library(robot_trajectory SHARED src/robot_trajectory.cpp)
# target_link_libraries(robot_trajectory
#   DEPENDENCIES
#   )
target_include_directories(robot_trajectory
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(robot_trajectory
  ${DEPENDENCIES}
  )
ament_export_targets(robot_trajectory HAS_LIBRARY_TARGET)
install(
  TARGETS robot_trajectory
  EXPORT robot_trajectory
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

######################
### Casadi Example ###
######################
# add_executable(casadi_example_model_generation src/casadi_examples/casadi_example_model_generation.cpp)
# ament_target_dependencies(casadi_example_model_generation
#   ${DEPENDENCIES}
#   )
#   target_link_libraries(casadi_example_model_generation
#   casadi_collocation_model
#   casadi
#   yaml-cpp
#   matplotlib_cpp::matplotlib_cpp
#   ${PYTHON_LIBRARIES}
#   )
# target_include_directories(casadi_example_model_generation
#   PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# install(TARGETS
#   casadi_example_model_generation
#   DESTINATION lib/${PROJECT_NAME})

###################
### Executables ###
###################
# # Main to test Diff Swerve State Space Modeling for MPC
# add_executable(swerve_model_test_main src/swerve_model_test_main.cpp)
# ament_target_dependencies(swerve_model_test_main
#   ${DEPENDENCIES}
# )
# target_link_libraries(swerve_model_test_main
#   casadi
# )
# target_include_directories(swerve_model_test_main
#   PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# install(TARGETS
#   swerve_model_test_main
#   DESTINATION lib/${PROJECT_NAME})

# # Main to generate Problem Formulation for MDS MPC
# add_executable(casadi_swerve_model_generation src/casadi_swerve_model_generation.cpp)
# ament_target_dependencies(casadi_swerve_model_generation
#   ${DEPENDENCIES}
# )
# target_link_libraries(casadi_swerve_model_generation
#   casadi_collocation_model
#   casadi
# )
# target_include_directories(casadi_swerve_model_generation
#   PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# install(TARGETS
#   casadi_swerve_model_generation
#   DESTINATION lib/${PROJECT_NAME})

###############
### Install ###
###############
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
