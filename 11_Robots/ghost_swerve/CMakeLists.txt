cmake_minimum_required(VERSION 3.8)
project(ghost_swerve)

# Add Global CMake Config for Ghost Packages
set(Ghost_DIR "$ENV{HOME}/VEXU_GHOST/cmake")
find_package(Ghost CONFIG REQUIRED)

# Adds all our package dependencies to one list
set(DEPENDENCIES
  ament_cmake
  ghost_ros_interfaces
  ghost_util
  ghost_estimation
  geometry_msgs
  sensor_msgs
  matplotlib_cpp
  PythonLibs
  visualization_msgs
  nav_msgs
  ghost_msgs
  pluginlib
  tf2
  ghost_planners
  ghost_motion_planner_core
  Eigen3
  behaviortree_cpp
  behaviortree_ros2
  )

# Iterates through the list and finds the packages!
foreach(pkg ${DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()

# Tell any packages that depend on us what dependencies we need.
ament_export_dependencies(${DEPENDENCIES})

set(INCLUDE
  include
  ${PYTHON_INCLUDE_DIRS}
)

include_directories(${INCLUDE})

###################
##### Targets #####
###################
add_library(swerve_model SHARED src/swerve_model.cpp)
target_include_directories(swerve_model
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(swerve_model
  ${DEPENDENCIES}
)
ament_export_libraries(
  swerve_model
)
ament_export_targets(swerve_model HAS_LIBRARY_TARGET)
install(
  TARGETS swerve_model
  EXPORT swerve_model
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

add_library(swerve_robot_plugin SHARED src/swerve_robot_plugin.cpp)
target_include_directories(swerve_robot_plugin
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  target_link_libraries(swerve_robot_plugin
    swerve_model
    swerve_tree
  )
ament_target_dependencies(swerve_robot_plugin
  ${DEPENDENCIES}
)
ament_export_libraries(
  swerve_robot_plugin
)
ament_export_targets(swerve_robot_plugin HAS_LIBRARY_TARGET)
install(
TARGETS swerve_robot_plugin
EXPORT swerve_robot_plugin
LIBRARY DESTINATION lib
ARCHIVE DESTINATION lib
RUNTIME DESTINATION bin
INCLUDES DESTINATION include
)

add_executable(swerve_motion_planner
  src/swerve_motion_planner.cpp
)
ament_target_dependencies(swerve_motion_planner
  ${DEPENDENCIES}
)
target_link_libraries(swerve_motion_planner
  swerve_model
)
target_include_directories(swerve_motion_planner
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
  swerve_motion_planner
  DESTINATION lib/${PROJECT_NAME})

# bt node libraries
set(NODES
  checkForRestart
  loggingNode
  moveToPose
  swipeTail
  intakeCmd
  climb
  autoDone
)

foreach(NODE ${NODES})
  add_library(${NODE} SHARED
    src/bt_nodes/${NODE}.cpp
  )
  target_include_directories(${NODE}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${NODE}
    ${DEPENDENCIES}
  )
  ament_export_targets(${NODE} HAS_LIBRARY_TARGET)
  install(
    TARGETS ${NODE}
    EXPORT ${NODE}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
  )
endforeach()

# run tree library
add_library(swerve_tree SHARED
  src/swerve_tree.cpp
)
target_include_directories(swerve_tree
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(swerve_tree
  ${DEPENDENCIES}
)
target_link_libraries(swerve_tree
  ${NODES}
)
ament_export_targets(swerve_tree HAS_LIBRARY_TARGET)
install(
  TARGETS swerve_tree
  EXPORT swerve_tree
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

###################
###### Tests ######
###################
set(TEST_FILES
  test_base_swerve_model
  test_coaxial_swerve_model
  test_differential_swerve_model
  test_swerve_icr
)

foreach(TEST ${TEST_FILES})
  ament_add_gtest(${TEST} test/${TEST}.cpp)
  ament_target_dependencies(${TEST} ${DEPENDENCIES})
  target_link_libraries(${TEST}
    swerve_model
    gtest_main
  )
  target_include_directories(${TEST} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>
  $<INSTALL_INTERFACE:include>)
endforeach()


###################
##### Install #####
###################
install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  urdf
  config
  launch
  DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(ghost_ros_interfaces plugins.xml)

ament_export_include_directories(${INCLUDE})
ament_package()
