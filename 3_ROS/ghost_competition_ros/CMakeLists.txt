cmake_minimum_required(VERSION 3.8)
project(ghost_competition_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set Package Dependencies
set(DEPENDENCIES
  ament_cmake
  rclcpp
  pluginlib
  ghost_msgs
  )

foreach(pkg ${DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()
ament_export_dependencies(${DEPENDENCIES})

# Set Include Directories
set(INCLUDE
include
)

include_directories(${INCLUDE})
ament_export_include_directories(${INCLUDE})

# set(CMAKE_BUILD_TYPE "DEBUG")
set(CMAKE_BUILD_TYPE "RELEASE")

#################
### Libraries ###
#################
# V5 Robot Base
add_library(v5_robot_base SHARED src/v5_robot_base.cpp)
ament_target_dependencies(v5_robot_base
${DEPENDENCIES}
)
target_include_directories(v5_robot_base
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_targets(v5_robot_base HAS_LIBRARY_TARGET)
install(
  TARGETS v5_robot_base
  EXPORT v5_robot_base
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

###################
### Executables ###
###################
add_executable(robot_state_machine_node src/robot_state_machine_node.cpp)
ament_target_dependencies(robot_state_machine_node
  ${DEPENDENCIES}
  )
target_include_directories(robot_state_machine_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
  robot_state_machine_node
  DESTINATION lib/${PROJECT_NAME})

###############
### Testing ###
###############

###############
### Install ###
###############
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
