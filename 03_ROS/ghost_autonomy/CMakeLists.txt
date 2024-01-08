cmake_minimum_required(VERSION 3.8)
project(ghost_autonomy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(DEPENDENCIES
  ament_cmake
  rclcpp
  behaviortree_cpp
  behaviortree_ros2
  pluginlib
  std_msgs
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

set(NODES
  run_tree
)
add_executable(${NODES} src/${NODES}.cpp)
ament_target_dependencies(${NODES}
  ${DEPENDENCIES}
)
install(TARGETS
  ${NODES}
  DESTINATION lib/${PROJECT_NAME})

# install(FILES
#   bt.xml
#   DESTINATION share/${PROJECT_NAME})

# add_executable(run_tree src/run_tree.cpp)
#   ament_target_dependencies(run_tree
#     ${DEPENDENCIES}
#   )

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
