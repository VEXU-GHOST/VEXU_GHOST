cmake_minimum_required(VERSION 3.5)
project(ghost_util)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake    REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp         REQUIRED)
find_package(sensor_msgs    REQUIRED)
find_package(geometry_msgs  REQUIRED)
find_package(yaml-cpp       REQUIRED)

include_directories(include)

#####################
##### Examples ######
#####################
# ROS Publisher Example
add_executable(ros_publisher_example src/examples/ros_publisher_example.cpp)
ament_target_dependencies(ros_publisher_example
  rclcpp
  sensor_msgs
  geometry_msgs
)

install(TARGETS
  ros_publisher_example
  DESTINATION lib/${PROJECT_NAME})

# ROS Subscriber Example
add_executable(ros_subscriber_example src/examples/ros_subscriber_example.cpp)
ament_target_dependencies(ros_subscriber_example
  rclcpp
  sensor_msgs
  geometry_msgs
)

install(TARGETS
  ros_subscriber_example
  DESTINATION lib/${PROJECT_NAME})

# YAML Load Example
add_executable(yaml_load_example src/examples/yaml_load.cpp)
target_link_libraries(yaml_load_example
  yaml-cpp
)

install(TARGETS
  yaml_load_example
  DESTINATION lib/${PROJECT_NAME})

####################
#### Ghost Util ####
####################
add_library(angle_util SHARED
  src/angle_util.cpp
)
target_include_directories(angle_util
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_targets(angle_util HAS_LIBRARY_TARGET)
install(
  TARGETS angle_util
  EXPORT angle_util
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#################
#### Testing ####
#################
ament_add_gtest(test_angle_util test/test_angle_util.cpp)
target_link_libraries(test_angle_util
  angle_util
  gtest
)

###############
### Install ###
###############
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
