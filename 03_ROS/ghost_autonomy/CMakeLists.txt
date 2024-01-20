cmake_minimum_required(VERSION 3.8)
project(ghost_autonomy)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()
# Add Global CMake Config for Ghost Packages
set(Ghost_DIR "$ENV{HOME}/VEXU_GHOST/cmake")
find_package(Ghost CONFIG REQUIRED)

# find dependencies
set(DEPENDENCIES
  ament_cmake
  rclcpp
  behaviortree_cpp
  behaviortree_ros2
  pluginlib
  std_msgs
  ghost_msgs
  ghost_v5_interfaces
  tf2
)

foreach(pkg ${DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()
ament_export_dependencies(${DEPENDENCIES})

# Set Include Directories
set(INCLUDE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

include_directories(${INCLUDE})
ament_export_include_directories(${INCLUDE})

# bt node libraries
set(NODES
  driveForward
  loggingNode
  moveToPose
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
add_library(run_tree SHARED
  src/run_tree.cpp
)
target_include_directories(run_tree
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(run_tree
  ${DEPENDENCIES}
)
target_link_libraries(run_tree
  ${NODES}
)
ament_export_targets(run_tree HAS_LIBRARY_TARGET)
install(
  TARGETS run_tree
  EXPORT run_tree
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# test robot config publisher
add_executable(publish_test
  src/publish_test.cpp
)
ament_target_dependencies(publish_test
  ${DEPENDENCIES}
)
target_include_directories(publish_test
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
publish_test
  DESTINATION lib/${PROJECT_NAME})

#################
#### Install ####
#################
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
