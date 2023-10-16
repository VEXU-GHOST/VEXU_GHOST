cmake_minimum_required(VERSION 3.8)
project(ghost_serial_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ghost_msgs REQUIRED)
find_package(ghost_common REQUIRED)
find_package(ghost_serial REQUIRED)
find_package(ghost_estimation REQUIRED)

############################
#### Jetson Serial Node ####
############################
# add_executable(jetson_v5_serial_node
#   src/jetson_v5_serial_node.cpp
# )
# ament_target_dependencies(jetson_v5_serial_node
#   rclcpp
#   tf2_msgs
#   visualization_msgs
#   sensor_msgs
#   ghost_msgs
#   ghost_common
#   ghost_serial
#   ghost_estimation
# )
# target_link_libraries(jetson_v5_serial_node
#   ghost_serial::jetson_serial_base
#   ghost_serial::cobs
#   # v5_serial_msg_config
#   yaml-cpp
# )
# target_include_directories(jetson_v5_serial_node
#   PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# install(TARGETS
#   jetson_v5_serial_node
#   DESTINATION lib/${PROJECT_NAME})

ament_package()
