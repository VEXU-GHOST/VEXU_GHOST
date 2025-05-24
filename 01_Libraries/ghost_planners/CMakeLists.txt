cmake_minimum_required(VERSION 3.5)
project(ghost_planners)

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
  ghost_util
  rclcpp
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

add_library(boomerang SHARED src/boomerang.cpp)
target_include_directories(boomerang
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(boomerang
  ${DEPENDENCIES}
)
# ament_export_libraries(
#   boomerang
# )
ament_export_targets(boomerang HAS_LIBRARY_TARGET)
install(
  TARGETS boomerang
  EXPORT boomerang
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

add_library(bezier_curve SHARED src/bezier_curve.cpp)
target_include_directories(bezier_curve
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(bezier_curve
  ${DEPENDENCIES}
)
# ament_export_libraries(
#   bezier_curve
# )
ament_export_targets(bezier_curve HAS_LIBRARY_TARGET)
install(
  TARGETS bezier_curve
  EXPORT bezier_curve
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
  ${DEPENDENCIES}
  )
  target_link_libraries(casadi_example_model_generation
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

###############
### Testing ###
###############

# Robot Trajectory Tests
set(DEVICE_TESTS
  test_robot_trajectory
  test_tank_robot_trajectory
)
foreach(TEST ${DEVICE_TESTS})
  ament_add_gtest(${TEST} test/${TEST}.cpp)
  ament_target_dependencies(${TEST} ${DEPENDENCIES})
  target_link_libraries(${TEST}
    gtest_main
    robot_trajectory
  )
endforeach()

###############
### Install ###
###############
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
