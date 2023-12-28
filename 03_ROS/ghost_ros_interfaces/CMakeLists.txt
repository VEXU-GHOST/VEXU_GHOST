cmake_minimum_required(VERSION 3.8)
project(ghost_ros_interfaces)

# Add Global CMake Config for Ghost Packages
set(Ghost_DIR "$ENV{HOME}/VEXU_GHOST/cmake")
find_package(Ghost CONFIG REQUIRED)

######################
#### Dependencies ####
######################
# Set Package Dependencies
set(DEPENDENCIES
  ament_cmake
  ament_cmake_gtest
  rclcpp
  ghost_v5_interfaces
  ghost_msgs
  yaml-cpp
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

###################
#### Libraries ####
###################
add_library(msg_helpers SHARED
src/msg_helpers/msg_helpers.cpp
)
target_include_directories(msg_helpers
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(msg_helpers
  ${DEPENDENCIES}
)
ament_export_targets(msg_helpers HAS_LIBRARY_TARGET)
install(
  TARGETS msg_helpers
  EXPORT msg_helpers
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#################
#### Testing ####
#################
set(TESTS
  test_robot_hardware_interface_ros
  test_device_interfaces_ros
)
foreach(TEST ${TESTS})
  ament_add_gtest(${TEST} test/${TEST}.cpp)
  ament_target_dependencies(${TEST} ${DEPENDENCIES})
  target_link_libraries(${TEST}
    gtest_main
    msg_helpers
  )
endforeach()

ament_package()
