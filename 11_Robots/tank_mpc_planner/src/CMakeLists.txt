cmake_minimum_required(VERSION 3.5)
project(tank_mpc_planner)

# Add Global CMake Config for Ghost Packages
set(Ghost_DIR "$ENV{VEXU_HOME}/cmake")
find_package(Ghost CONFIG REQUIRED)

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
  ghost_ros_interfaces
  ghost_util
  rclcpp
  ghost_msgs
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
# set(CMAKE_BUILD_TYPE "RELEASE")

###################
### Executables ###
###################
# Main to generate Problem Formulation for MDS MPC
add_executable(casadi_tank_model_generation src/casadi_tank_model_generation.cpp)
ament_target_dependencies(casadi_tank_model_generation
  ${DEPENDENCIES}
)
target_link_libraries(casadi_tank_model_generation
  casadi
  matplotlib_cpp::matplotlib_cpp
  ${PYTHON_LIBRARIES}
)
target_include_directories(casadi_tank_model_generation
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
  casadi_tank_model_generation
  DESTINATION lib/${PROJECT_NAME})

###############
### Testing ###
###############

###############
### Install ###
###############
# install(
#  DIRECTORY include/
#  DESTINATION include
# )

ament_package()
