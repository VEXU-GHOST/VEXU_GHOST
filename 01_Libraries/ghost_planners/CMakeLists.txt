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
# Trajectory class
add_library(trajectory SHARED src/trajectory.cpp)
target_include_directories(trajectory
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(trajectory
  ${DEPENDENCIES}
  )
ament_export_targets(trajectory HAS_LIBRARY_TARGET)
install(
  TARGETS trajectory
  EXPORT trajectory
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

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

###############
### Testing ###
###############

# Trajectory Tests
ament_add_gtest(test_trajectory test/test_trajectory.cpp)
ament_target_dependencies(test_trajectory ${DEPENDENCIES})
target_link_libraries(test_trajectory
  gtest_main
  trajectory
)

# Robot Trajectory Tests
ament_add_gtest(test_robot_trajectory test/test_robot_trajectory.cpp)
ament_target_dependencies(test_robot_trajectory ${DEPENDENCIES})
target_link_libraries(test_robot_trajectory
  gtest_main
  robot_trajectory
)

###############
### Install ###
###############
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
