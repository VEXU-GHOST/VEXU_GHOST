cmake_minimum_required(VERSION 3.8)
project(ghost_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

#####################
##### Examples ######
#####################
# ROS Publisher Example
add_executable(ros_publisher_example src/ros_publisher_example.cpp)
ament_target_dependencies(ros_publisher_example
  rclcpp
  sensor_msgs
  geometry_msgs
)

install(TARGETS
  ros_publisher_example
  DESTINATION lib/${PROJECT_NAME})

# ROS Subscriber Example
add_executable(ros_subscriber_example src/ros_subscriber_example.cpp)
ament_target_dependencies(ros_subscriber_example
  rclcpp
  sensor_msgs
  geometry_msgs
)

install(TARGETS
  ros_subscriber_example
  DESTINATION lib/${PROJECT_NAME})

# YAML Load Example
add_executable(yaml_load_example src/yaml_load.cpp)
target_link_libraries(yaml_load_example
  yaml-cpp
)

install(TARGETS
  yaml_load_example
  DESTINATION lib/${PROJECT_NAME})

ament_package()
