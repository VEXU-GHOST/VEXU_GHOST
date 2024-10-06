cmake_minimum_required(VERSION 3.8)
project(ghost_sim_examples)

# Add Global CMake Config for Ghost Packages
set(Ghost_DIR "$ENV{VEXU_HOME}/cmake")
find_package(Ghost CONFIG REQUIRED)

# Adds all our package dependencies to one list
set(DEPENDENCIES
ament_cmake
rclcpp
std_msgs
gazebo_ros
ghost_msgs
ghost_control
ghost_util
gazebo
ghost_control
ghost_v5_interfaces
)

# Iterates through the list and finds the packages!
foreach(pkg ${DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()

ament_export_dependencies(${DEPENDENCIES})

# Add any include directories to a list
# Usually this is just "include", but for some depedencies there could be extras.
set(INCLUDE
  include  
  ${GAZEBO_INCLUDE_DIRS}
)

# Tells the package where all our header files live
include_directories(${INCLUDE})

# Tells any packages that depend on us what include files we use.
ament_export_include_directories(${INCLUDE})

### Gazebo ###
link_directories(${GAZEBO_LIBRARY_DIRS})
add_definitions(${GAZEBO_CXX_FLAGS})

add_library(v5_robot_simulator_plugin SHARED src/v5_robot_simulator_plugin.cpp)
ament_target_dependencies(v5_robot_simulator_plugin ${DEPENDENCIES})
target_link_libraries(v5_robot_simulator_plugin ${GAZEBO_LIBRARIES})
target_include_directories(v5_robot_simulator_plugin PUBLIC ${GHOST_SIM_EXAMPLES_INCLUDE_DIRS})

###############
### Install ###
###############
install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  urdf
  rviz
  world
  test
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
