cmake_minimum_required(VERSION 3.8)
project(ghost_example_robot)

# Add Global CMake Config for Ghost Packages
set(Ghost_DIR "$ENV{VEXU_HOME}/cmake")
find_package(Ghost CONFIG REQUIRED)

# Adds all our package dependencies to one list
set(DEPENDENCIES
  ament_cmake
  rclcpp
  ghost_ros_interfaces
  pluginlib
  )

# Iterates through the list and finds the packages!
foreach(pkg ${DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()

ament_export_dependencies(${DEPENDENCIES})

set(INCLUDE
  include
)

include_directories(${INCLUDE})

####################
##### Targets ######
####################
add_library(ghost_example_robot SHARED src/ghost_example_robot.cpp)
target_include_directories(ghost_example_robot
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ghost_example_robot
  ${DEPENDENCIES}
)
ament_export_libraries(
  ghost_example_robot
)
ament_export_targets(ghost_example_robot HAS_LIBRARY_TARGET)
install(
TARGETS ghost_example_robot
EXPORT ghost_example_robot
LIBRARY DESTINATION lib
ARCHIVE DESTINATION lib
RUNTIME DESTINATION bin
INCLUDES DESTINATION include
)

#################
##### Tests #####
#################
ament_add_gtest(test_example_plugin test/test_example_plugin.cpp)
ament_target_dependencies(test_example_plugin
  ${DEPENDENCIES}
)
target_link_libraries(test_example_plugin
  ghost_example_robot
  gtest
)
target_include_directories(test_example_plugin
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

###################
##### Install #####
###################
install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(ghost_ros_interfaces plugins.xml)

ament_export_include_directories(${INCLUDE})
ament_package()
