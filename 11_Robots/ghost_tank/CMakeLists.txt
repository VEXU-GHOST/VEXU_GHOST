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

add_library(tank_robot_plugin SHARED src/tank_robot_plugin.cpp)
target_include_directories(tank_robot_plugin
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  target_link_libraries(tank_robot_plugin
    tank_model
    tank_tree
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

add_executable(cubic_motion_planner
  src/cubic_motion_planner.cpp
)
ament_target_dependencies(cubic_motion_planner
  ${DEPENDENCIES}
)
target_link_libraries(cubic_motion_planner

)
target_include_directories(cubic_motion_planner
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
  cubic_motion_planner
  DESTINATION lib/${PROJECT_NAME})

add_executable(trapezoid_motion_planner
  src/trapezoid_motion_planner.cpp
)
ament_target_dependencies(trapezoid_motion_planner
  ${DEPENDENCIES}
)
target_link_libraries(trapezoid_motion_planner

)
target_include_directories(trapezoid_motion_planner
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
  trapezoid_motion_planner
  DESTINATION lib/${PROJECT_NAME})

# bt node libraries
set(NODES
  loggingNode
  moveToPoseCubic
  moveToPoseTrapezoid
  intakeCmd
  autoDone
  autonTimer
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
    tank_model
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
