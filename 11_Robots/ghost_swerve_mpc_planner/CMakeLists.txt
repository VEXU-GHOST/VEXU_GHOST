cmake_minimum_required(VERSION 3.5)
project(ghost_swerve_mpc_planner)

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
  ghost_planners
  ghost_util
  rclcpp
  ghost_msgs
)

foreach(pkg ${DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()

ament_export_dependencies(${DEPENDENCIES})

# # Set Include Directories
# set(INCLUDE
#   include
# )

# include_directories(${INCLUDE})
# ament_export_include_directories(${INCLUDE})


# ##################
# ## Executables ###
# ##################
# Main to generate Problem Formulation for MDS MPC
add_executable(swerve_mpc_planner_node src/swerve_mpc_planner_node.cpp)
ament_target_dependencies(swerve_mpc_planner_node
  ${DEPENDENCIES}
)
target_link_libraries(swerve_mpc_planner_node
  casadi
  matplotlib_cpp::matplotlib_cpp
  ${PYTHON_LIBRARIES}
)
target_include_directories(swerve_mpc_planner_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
  swerve_mpc_planner_node
  DESTINATION lib/${PROJECT_NAME})

# ##############
# ## Testing ###
# ##############

##############
## Install ###
##############
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
