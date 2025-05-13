cmake_minimum_required(VERSION 3.8)
project(ghost_tank)

# Add Global CMake Config for Ghost Packages
set(Ghost_DIR "$ENV{VEXU_HOME}/cmake")
find_package(Ghost CONFIG REQUIRED)

# Adds all our package dependencies to one list
set(DEPENDENCIES
  ament_cmake
  ghost_ros_interfaces
  ghost_util
  ghost_estimation
  ghost_planners
  ghost_motion_planner_core
  geometry_msgs
  sensor_msgs
  nav_msgs
  ghost_msgs
  visualization_msgs
  matplotlib_cpp
  PythonLibs
  pluginlib
  tf2
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
add_library(tank_model SHARED src/tank_model.cpp)
target_include_directories(tank_model
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(tank_model
  ${DEPENDENCIES}
)
ament_export_libraries(
  tank_model
)
ament_export_targets(tank_model HAS_LIBRARY_TARGET)
install(
  TARGETS tank_model
  EXPORT tank_model
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

add_library(tank_odom SHARED src/tank_odom.cpp)
target_include_directories(tank_odom
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(tank_odom
  ${DEPENDENCIES}
)
ament_export_libraries(
  tank_odom
)
ament_export_targets(tank_odom HAS_LIBRARY_TARGET)
install(
  TARGETS tank_odom
  EXPORT tank_odom
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

add_library(pdcontrol SHARED src/pdcontrol.cpp)
target_include_directories(pdcontrol
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pdcontrol
  ${DEPENDENCIES}
)
ament_export_libraries(
  pdcontrol
)
ament_export_targets(pdcontrol HAS_LIBRARY_TARGET)
install(
  TARGETS pdcontrol
  EXPORT pdcontrol
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

add_library(tank_robot_plugin SHARED src/tank_robot_plugin.cpp)
target_include_directories(tank_robot_plugin
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  target_link_libraries(tank_robot_plugin
    tank_model
    tank_odom
    tank_tree
    pdcontrol
  )
ament_target_dependencies(tank_robot_plugin
  ${DEPENDENCIES}
)
ament_export_libraries(
  tank_robot_plugin
)
ament_export_targets(tank_robot_plugin HAS_LIBRARY_TARGET)
install(
TARGETS tank_robot_plugin
EXPORT tank_robot_plugin
LIBRARY DESTINATION lib
ARCHIVE DESTINATION lib
RUNTIME DESTINATION bin
INCLUDES DESTINATION include
)

add_library(alpha_jerry_plugin SHARED src/alpha_jerry_plugin.cpp)
target_include_directories(alpha_jerry_plugin
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  target_link_libraries(alpha_jerry_plugin
    tank_model
    tank_odom
    tank_tree
    pdcontrol
  )
ament_target_dependencies(alpha_jerry_plugin
  ${DEPENDENCIES}
)
ament_export_libraries(
  alpha_jerry_plugin
)
ament_export_targets(alpha_jerry_plugin HAS_LIBRARY_TARGET)
install(
TARGETS alpha_jerry_plugin
EXPORT alpha_jerry_plugin
LIBRARY DESTINATION lib
ARCHIVE DESTINATION lib
RUNTIME DESTINATION bin
INCLUDES DESTINATION include
)

# bt node libraries
set(NODES
  loggingNode
  autoDone
  autonTimer
  moveToPoseBezier
  moveToPoseBoomerang
  moveToPosePurepursuit
  biteCmd
  clampCmd
  climbCmd
  shutoffNode
  intakeCmd
  isHanging
  goalRushCmd
  setMirrored
  conveyorCmd
  waitCmd
  neutralStakeCmd
  setColorTarget
  goalRushDetected
  goalDetected
  boundaryCheck
  moveVoltage
  turnToPoint
  moveToPoint
  moveScissor
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
  target_link_libraries(${NODE}
    pdcontrol
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
add_library(tank_tree SHARED
  src/tank_tree.cpp
)
target_include_directories(tank_tree
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(tank_tree
  ${DEPENDENCIES}
)
target_link_libraries(tank_tree
  ${NODES}
)
ament_export_targets(tank_tree HAS_LIBRARY_TARGET)
install(
  TARGETS tank_tree
  EXPORT tank_tree
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

###################
###### Tests ######
###################
set(TEST_FILES
)

foreach(TEST ${TEST_FILES})
  ament_add_gtest(${TEST} test/${TEST}.cpp)
  ament_target_dependencies(${TEST} ${DEPENDENCIES})
  target_link_libraries(${TEST}
    tank_odom
    gtest_main
  )
  target_include_directories(${TEST} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>
  $<INSTALL_INTERFACE:include>)
endforeach()

ament_add_gtest(test_tank_odom test/test_tank_odom.cpp)
target_link_libraries(test_tank_odom
  gtest
  tank_odom
)


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