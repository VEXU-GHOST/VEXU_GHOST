cmake_minimum_required(VERSION 3.8)
project(ghost_over_under)

# Add Global CMake Config for Ghost Packages
set(Ghost_DIR "$ENV{HOME}/VEXU_GHOST/cmake")
find_package(Ghost CONFIG REQUIRED)

# Adds all our package dependencies to one list
set(DEPENDENCIES
  ament_cmake
  )

# Iterates through the list and finds the packages!
foreach(pkg ${DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()

# Tell any packages that depend on us what dependencies we need.
ament_export_dependencies(${DEPENDENCIES})

set(INCLUDE
  include
)

include_directories(${INCLUDE})

ament_export_include_directories(${INCLUDE})

####################
##### Install ######
####################
install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME})

# This generates the ROS2 package and should always be the last line.
ament_package()
