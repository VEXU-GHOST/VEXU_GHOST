cmake_minimum_required(VERSION 3.8)
project(ghost_robot_plugin_examples)

# Add Global CMake Config for Ghost Packages
set(Ghost_DIR "$ENV{VEXU_HOME}/cmake")
find_package(Ghost CONFIG REQUIRED)

# Adds all our package dependencies to one list
set(DEPENDENCIES
  ament_cmake
  rclcpp
  ghost_ros_interfaces
  )

# Iterates through the list and finds the packages!
foreach(pkg ${DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()

####################
##### Targets ######
####################
add_library(ghost_robot_plugin_example SHARED src/ghost_robot_plugin_example.cpp)
target_include_directories(ghost_robot_plugin_example
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ghost_robot_plugin_example
  ${DEPENDENCIES}
)
ament_export_libraries(
  ghost_robot_plugin_example
)
ament_export_targets(ghost_robot_plugin_example HAS_LIBRARY_TARGET)
install(
TARGETS ghost_robot_plugin_example
EXPORT ghost_robot_plugin_example
LIBRARY DESTINATION lib
ARCHIVE DESTINATION lib
RUNTIME DESTINATION bin
INCLUDES DESTINATION include
)

###################
##### Install #####
###################
install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(ghost_robot_plugin_examples plugins.xml)

ament_export_include_directories(${INCLUDE})

ament_package()
