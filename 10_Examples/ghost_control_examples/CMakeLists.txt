cmake_minimum_required(VERSION 3.8)
project(ghost_control_examples)

# Set Package Dependencies
set(DEPENDENCIES
  ament_cmake
  casadi
  matplotlib_cpp
  yaml-cpp
  PythonLibs
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

##################################
### Basic Optimization Example ###
##################################
add_executable(casadi_ipopt_example src/casadi_ipopt_example.cpp)
ament_target_dependencies(casadi_ipopt_example
  ${DEPENDENCIES}
  )
  target_link_libraries(casadi_ipopt_example
  casadi
  )
target_include_directories(casadi_ipopt_example
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
  casadi_ipopt_example
  DESTINATION lib/${PROJECT_NAME})

#####################################
### Trajectory Generation Example ###
#####################################
add_executable(casadi_trajectory_generation src/casadi_trajectory_generation.cpp)
ament_target_dependencies(casadi_trajectory_generation
  ${DEPENDENCIES}
  )
  target_link_libraries(casadi_trajectory_generation
  casadi
  yaml-cpp
  matplotlib_cpp::matplotlib_cpp
  ${PYTHON_LIBRARIES}
  )
target_include_directories(casadi_trajectory_generation
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
casadi_trajectory_generation
  DESTINATION lib/${PROJECT_NAME})

include_directories(${INCLUDE})
ament_export_include_directories(${INCLUDE})

#####################################
### Trajectory Generation Example ###
#####################################

ament_package()
