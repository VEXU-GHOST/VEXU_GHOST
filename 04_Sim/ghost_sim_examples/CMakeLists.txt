cmake_minimum_required(VERSION 3.8)
project(ghost_sim_examples)

		
# Add Global CMake Config for Ghost Packages
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
set(Ghost_DIR "$ENV{HOME}/VEXU_GHOST/cmake")
  add_compile_options(-Wall -Wextra -Wpedantic)
find_package(Ghost CONFIG REQUIRED)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# Adds all our package dependencies to one list
# further dependencies manually.
set(DEPENDENCIES
# find_package(<dependency> REQUIRED)
  ament_cmake
  )
# Iterates through the list and finds the packages!
if(BUILD_TESTING)
foreach(pkg ${DEPENDENCIES})
  find_package(${pkg} REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endforeach()
endif()
# Tell any packages that depend on us what dependencies we need.
ament_export_dependencies(${DEPENDENCIES})
install(DIRECTORY
  launch
  urdf
  rviz
  world
  DESTINATION share/${PROJECT_NAME})
# This generates the ROS2 package and should always be the last line.
ament_package()